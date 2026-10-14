// ============================================================================
//  Bench probe: motion test run. Drives the real MotionControl / MotionPrimitives code with
//  parameters that serial commands set live, logs each move into RAM and dumps it afterwards.
//  Not part of the mission build. docs: none - throwaway tooling for tuning RobotSpec.h.
//
//  The motors NEVER move on boot or reset. Only a command starts a move, and any byte received
//  while moving aborts it. Hard limits that do not depend on the controller or the host:
//    duty cap, time cap, distance cap, front cut-off, and a pushed-but-not-moving cut-off.
//
//  Commands (one line, end with newline):
//    h                       help
//    p                       print the tunables
//    set <name> <value>      change a tunable (names in `p`)
//    w                       where: odometer, heading, encoders and the three ultrasonics
//    b                       recapture the gyro bias (robot held still) and zero the heading
//    o <L> <R> <ms>          open loop: fixed signed duty on each wheel (max +-DUTY_CAP, 2000 ms)
//    s <mm>                  closed-loop straight, signed (max +-DIST_CAP_MM)
//    t <deg>                 closed-loop turn in place, CCW positive
//    x                       stop (also: any byte during a move)
// ============================================================================

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "Heading.h"
#include "MotionControl.h"
#include "MotionPrimitives.h"
#include "Perception.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Scheduler.h"
#include "WallSense.h"
#include "hal/encoders.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"

namespace
{

// ---- Hard limits -------------------------------------------------------------------------
constexpr int16_t DUTY_CAP = 400;       // any wheel, any move
constexpr uint16_t MOVE_MAX_MS = 4000;  // any move
constexpr uint16_t OPEN_MAX_MS = 2000;  // open-loop moves
constexpr float DIST_CAP_MM = 750.0f;   // odometer travel from where the move began
constexpr uint16_t FRONT_CUT_MM = 130;  // front reading (axle to wall) at or below: stop
constexpr int16_t PUSH_DUTY = 250;      // pushing at least this hard...
constexpr uint16_t PUSH_STILL_MS = 600; // ...with no movement this long: stop
constexpr float STILL_MM_S = 5.0f;
constexpr float STILL_DPS = 5.0f;
constexpr uint16_t PUSH_GRACE_MS = 300; // a move may start from rest

// ---- Log -------------------------------------------------------------------------------
constexpr uint8_t LOG_MAX = 200; // samples, one per LOG_EVERY ticks: 4 s at 20 ms
constexpr uint8_t LOG_EVERY = 2;

struct Sample {
    int16_t dist;     // 0.1 mm
    int16_t distRef;  // 0.1 mm
    int16_t speed;    // mm/s
    int16_t speedRef; // mm/s
    int16_t hdg;      // 0.1 deg
    int16_t hdgRef;   // 0.1 deg
    int16_t yaw;      // 0.1 deg/s
    int16_t dutyL;
    int16_t dutyR;
    int16_t left; // ultrasonic, mm (0 = no echo)
    int16_t right;
    int16_t trim; // wall-steering offset added to the held heading, 0.1 deg
};

Sample g_log[LOG_MAX];
uint8_t g_logCount = 0;

// ---- Tunables ----------------------------------------------------------------------------
struct Tune {
    MotionControlParams ctl;
    PlanParams plan;
    float vMax = 150.0f; // mm/s cruise for `s`
    float wall = 0.0f;   // 1 = steer `s` moves on the side walls
    float side = 220.0f; // side reading below this = wall
    float front = 260.0f;
    float center = 117.0f; // side reading when centred
    float kpw = 0.0f;      // deg of heading offset per mm of lateral error
    float kdw = 0.0f;
    float smax = 10.0f;
};
Tune g_tune;
WallSteer g_steer;

struct Name {
    const char *key;
    float *value;
};
const Name g_names[] = {
    {"kpd", &g_tune.ctl.kpDist},
    {"kid", &g_tune.ctl.kiDist},
    {"kdd", &g_tune.ctl.kdDist},
    {"kpt", &g_tune.ctl.kpTurn},
    {"kit", &g_tune.ctl.kiTurn},
    {"kdt", &g_tune.ctl.kdTurn},
    {"ffv", &g_tune.ctl.ffPwmPerMmS},
    {"ffd", &g_tune.ctl.ffPwmPerDps},
    {"v", &g_tune.vMax},
    {"acc", &g_tune.plan.accelMmS2},
    {"rate", &g_tune.plan.turnRateMaxDps},
    {"tacc", &g_tune.plan.turnAccelDps2},
    {"stol", &g_tune.plan.stepToleranceMm},
    {"ttol", &g_tune.plan.turnToleranceDeg},
    {"wall", &g_tune.wall},
    {"side", &g_tune.side},
    {"front", &g_tune.front},
    {"cen", &g_tune.center},
    {"kpw", &g_tune.kpw},
    {"kdw", &g_tune.kdw},
    {"smax", &g_tune.smax},
};

void tuneDefaults()
{
    g_tune.ctl = MotionControlParams{};
    g_tune.ctl.pwmTop = MOTOR_PWM_TOP;
    // Start points, not answers. The wheel-speed sweep (docs/calibration/, wheels free) gives
    // about 1.12 mm/s per duty count above duty ~200, so ~0.9 duty per mm/s; a pivot moves each
    // wheel at (track / 2) * omega = 1.48 mm/s per deg/s, so ~1.3 duty per deg/s. minPwm stays 0
    // here: toDuty() adds it as an offset to every non-zero command.
    g_tune.ctl.ffPwmPerMmS = 1.15f;
    g_tune.ctl.ffPwmPerDps = 1.3f;
    g_tune.ctl.kpDist = 4.0f;
    g_tune.ctl.kiDist = 2.0f;
    g_tune.ctl.kpTurn = 4.0f;
    g_tune.ctl.kiTurn = 8.0f;
    g_tune.ctl.kdTurn = 0.5f;
    g_tune.plan.accelMmS2 = 500.0f;
    g_tune.plan.turnRateMaxDps = 120.0f;
    g_tune.plan.turnAccelDps2 = 400.0f;
    g_tune.plan.stepToleranceMm = 3.0f;
    g_tune.plan.turnToleranceDeg = 4.0f;
    g_tune.plan.turnSettleMs = 150;
    g_tune.plan.moveTimeoutMs = MOVE_MAX_MS;
    g_tune.vMax = 150.0f;
}

WallParams wallParams()
{
    WallParams w;
    w.sideMm = static_cast<uint16_t>(g_tune.side);
    w.frontMm = static_cast<uint16_t>(g_tune.front);
    w.centerDistMm = static_cast<uint16_t>(g_tune.center);
    w.kp = g_tune.kpw;
    w.kd = g_tune.kdw;
    w.steerMaxDeg = g_tune.smax;
    return w;
}

// ---- Robot state -------------------------------------------------------------------------
RobotState g_state;
Perceiver g_perceiver;
Perception g_per;
PerceiveParams g_pp;
bool g_imuPresent = false;
GyroBias g_bias;
Heading g_heading;

enum class Mode : uint8_t { Idle, Open, Straight, Turn };
Mode g_mode = Mode::Idle;

MotionControl g_mc;
Straight g_straight;
Turn g_turn;
int16_t g_openL = 0;
int16_t g_openR = 0;
uint16_t g_openMs = 0;

uint16_t g_elapsedMs = 0;
uint16_t g_stillMs = 0;
float g_startDistMm = 0.0f;
uint32_t g_tick = 0;
const char *g_reason = "";

int16_t clampDuty(int16_t d)
{
    return d > DUTY_CAP ? DUTY_CAP : (d < -DUTY_CAP ? -DUTY_CAP : d);
}

int16_t absDuty(int16_t d)
{
    return d < 0 ? static_cast<int16_t>(-d) : d;
}

int16_t toTenths(float v)
{
    const float t = v * 10.0f;
    return static_cast<int16_t>(t > 32000.0f ? 32000.0f : (t < -32000.0f ? -32000.0f : t));
}

void calibrateGyroBias()
{
    for (uint8_t attempt = 1; attempt <= GYRO_BIAS_MAX_ATTEMPTS; ++attempt) {
        gyroBiasReset(g_bias);
        for (uint16_t i = 0; i < GYRO_BIAS_SAMPLES; ++i) {
            ImuRaw sample;
            if (imuRead(sample)) {
                gyroBiasAccumulate(g_bias, sample.gyro[IMU_GYRO_AXIS_YAW]);
            }
            delay(1);
        }
        if (gyroBiasFinish(g_bias)) {
            headingSet(g_heading, 0.0f);
            Serial.print(F("# gyro bias "));
            Serial.print(g_bias.counts);
            Serial.print(F(" counts, spread "));
            Serial.println(g_bias.spread);
            return;
        }
        Serial.print(F("# gyro bias attempt "));
        Serial.print(attempt);
        Serial.println(F(" rejected - hold the robot still"));
    }
    Serial.println(F("# gyro bias FAILED: no heading, closed-loop moves are refused"));
}

void readSensors()
{
    encodersRead(g_state.encoderCountL, g_state.encoderCountR);
    if (g_imuPresent && imuRead(g_state.imu)) {
        headingUpdate(g_heading, g_bias, g_state.imu.gyro[IMU_GYRO_AXIS_YAW],
                      CONTROL_LOOP_PERIOD_MS);
    }
    g_state.headingDeg = g_heading.deg;
    ultrasonicUpdate();
    g_state.distFrontMm = ultrasonicDistanceMm(Ultrasonic::Front);
    g_state.distLeftMm = ultrasonicDistanceMm(Ultrasonic::Left);
    g_state.distRightMm = ultrasonicDistanceMm(Ultrasonic::Right);
    g_state.usValid = ultrasonicValidMask();
}

void printTunables()
{
    for (const Name &n : g_names) {
        Serial.print(n.key);
        Serial.print('=');
        Serial.print(*n.value, 3);
        Serial.print(' ');
    }
    Serial.print(F("minl="));
    Serial.print(g_tune.ctl.minPwmL);
    Serial.print(F(" minr="));
    Serial.println(g_tune.ctl.minPwmR);
    Serial.print(F("# caps: duty "));
    Serial.print(DUTY_CAP);
    Serial.print(F(", move "));
    Serial.print(MOVE_MAX_MS);
    Serial.print(F(" ms, dist "));
    Serial.print(DIST_CAP_MM, 0);
    Serial.print(F(" mm, front cut "));
    Serial.print(FRONT_CUT_MM);
    Serial.println(F(" mm"));
}

void help()
{
    Serial.println(F("# h | p | w | set <name> <v> | b | o <L> <R> <ms> | s <mm> | t <deg> | x"));
    Serial.println(
        F("# names: kpd kid kdd kpt kit kdt ffv ffd v acc rate tacc stol ttol minl minr"));
}

void beginMove(Mode mode)
{
    motionControlReset(g_mc);
    wallSteerReset(g_steer);
    g_logCount = 0;
    g_elapsedMs = 0;
    g_stillMs = 0;
    g_tick = 0;
    g_startDistMm = g_per.motion.distMm;
    g_reason = "";
    g_mode = mode;
    motorsEnable(true);
}

void endMove(const char *reason)
{
    motorSet(Motor::Left, 0);
    motorSet(Motor::Right, 0);
    motorsEnable(false);
    const Mode was = g_mode;
    g_mode = Mode::Idle;
    g_reason = reason;

    Serial.print(F("# end "));
    Serial.print(was == Mode::Open ? F("open")
                                   : (was == Mode::Straight ? F("straight") : F("turn")));
    Serial.print(F(" reason="));
    Serial.print(reason);
    Serial.print(F(" ms="));
    Serial.print(g_elapsedMs);
    Serial.print(F(" dist="));
    Serial.print(g_per.motion.distMm - g_startDistMm, 1);
    Serial.print(F(" heading="));
    Serial.print(g_per.motion.headingDeg, 1);
    Serial.print(F(" encL="));
    Serial.print(g_state.encoderCountL);
    Serial.print(F(" encR="));
    Serial.println(g_state.encoderCountR);
    Serial.println(
        F("# t_ms,dist,distRef,speed,speedRef,hdg,hdgRef,yaw,dutyL,dutyR,left,right,trim "
          "(dist/hdg/yaw/trim x10)"));
    for (uint8_t i = 0; i < g_logCount; ++i) {
        const Sample &s = g_log[i];
        Serial.print(static_cast<uint16_t>(i) * LOG_EVERY * CONTROL_LOOP_PERIOD_MS);
        Serial.print(',');
        Serial.print(s.dist);
        Serial.print(',');
        Serial.print(s.distRef);
        Serial.print(',');
        Serial.print(s.speed);
        Serial.print(',');
        Serial.print(s.speedRef);
        Serial.print(',');
        Serial.print(s.hdg);
        Serial.print(',');
        Serial.print(s.hdgRef);
        Serial.print(',');
        Serial.print(s.yaw);
        Serial.print(',');
        Serial.print(s.dutyL);
        Serial.print(',');
        Serial.print(s.dutyR);
        Serial.print(',');
        Serial.print(s.left);
        Serial.print(',');
        Serial.print(s.right);
        Serial.print(',');
        Serial.println(s.trim);
    }
    Serial.println(F("# done"));
}

void logSample(const MotionTarget &t, int16_t dutyL, int16_t dutyR, float trimDeg)
{
    if (g_tick % LOG_EVERY != 0 || g_logCount >= LOG_MAX) {
        return;
    }
    Sample &s = g_log[g_logCount++];
    s.dist = toTenths(g_per.motion.distMm - g_startDistMm);
    s.distRef = toTenths(t.distRefMm - g_startDistMm);
    s.speed = static_cast<int16_t>(g_per.motion.speedMmS);
    s.speedRef = static_cast<int16_t>(t.speedRefMmS);
    s.hdg = toTenths(g_per.motion.headingDeg);
    s.hdgRef = toTenths(t.headingRefDeg);
    s.yaw = toTenths(g_per.motion.yawRateDps);
    s.dutyL = dutyL;
    s.dutyR = dutyR;
    s.left = static_cast<int16_t>(g_state.distLeftMm);
    s.right = static_cast<int16_t>(g_state.distRightMm);
    s.trim = toTenths(trimDeg);
}

// Returns a reason to stop, or nullptr.
const char *hardLimit(int16_t dutyL, int16_t dutyR)
{
    if (g_elapsedMs >= MOVE_MAX_MS) {
        return "time-cap";
    }
    const float travelled = g_per.motion.distMm - g_startDistMm;
    if (travelled > DIST_CAP_MM || travelled < -DIST_CAP_MM) {
        return "dist-cap";
    }
    if ((g_state.usValid & 1u) != 0 && g_state.distFrontMm <= FRONT_CUT_MM &&
        g_per.motion.speedMmS > 20.0f) {
        return "front-wall";
    }
    const int16_t peak = absDuty(dutyL) > absDuty(dutyR) ? absDuty(dutyL) : absDuty(dutyR);
    const float sp = g_per.motion.speedMmS < 0 ? -g_per.motion.speedMmS : g_per.motion.speedMmS;
    const float yw =
        g_per.motion.yawRateDps < 0 ? -g_per.motion.yawRateDps : g_per.motion.yawRateDps;
    if (g_elapsedMs > PUSH_GRACE_MS && peak >= PUSH_DUTY && sp < STILL_MM_S && yw < STILL_DPS) {
        g_stillMs = static_cast<uint16_t>(g_stillMs + CONTROL_LOOP_PERIOD_MS);
        if (g_stillMs >= PUSH_STILL_MS) {
            return "pushed-but-still";
        }
    } else {
        g_stillMs = 0;
    }
    return nullptr;
}

void stepMove()
{
    g_elapsedMs = static_cast<uint16_t>(g_elapsedMs + CONTROL_LOOP_PERIOD_MS);
    const MotionFeedback &fb = g_per.motion;
    MotionTarget target;
    target.distRefMm = fb.distMm;
    target.headingRefDeg = fb.headingDeg;
    int16_t l = 0;
    int16_t r = 0;
    float trim = 0.0f;
    const char *reason = nullptr;

    if (g_mode == Mode::Open) {
        l = g_openL;
        r = g_openR;
        if (g_elapsedMs >= g_openMs) {
            reason = "open-done";
        }
    } else {
        MoveStatus st;
        if (g_mode == Mode::Straight) {
            if (g_tune.wall > 0.5f) {
                const WallParams wp = wallParams();
                trim = wallSteerTrim(g_steer, wallView(g_state, wp), wp, CONTROL_LOOP_PERIOD_MS);
            }
            st = straightStep(g_straight, g_tune.plan, fb, trim, CONTROL_LOOP_PERIOD_MS, target);
        } else {
            st = turnStep(g_turn, g_tune.plan, fb, CONTROL_LOOP_PERIOD_MS, target);
        }
        const MotionOutput out =
            motionControlStep(g_mc, g_tune.ctl, target, fb, CONTROL_LOOP_PERIOD_MS);
        l = clampDuty(out.left);
        r = clampDuty(out.right);
        if (st == MoveStatus::Done) {
            reason = "arrived";
        } else if (st == MoveStatus::Timeout) {
            reason = "move-timeout";
        }
    }

    if (reason == nullptr) {
        reason = hardLimit(l, r);
    }
    logSample(target, l, r, trim);
    ++g_tick;
    if (reason != nullptr) {
        endMove(reason);
        return;
    }
    motorSet(Motor::Left, l);
    motorSet(Motor::Right, r);
}

// ---- Commands ----------------------------------------------------------------------------
bool needHeading()
{
    if (!g_imuPresent || !g_bias.ready) {
        Serial.println(F("# refused: no IMU heading (closed loop needs it)"));
        return false;
    }
    return true;
}

void handleLine(char *line)
{
    char *cmd = strtok(line, " \t");
    if (cmd == nullptr) {
        return;
    }
    if (strcmp(cmd, "h") == 0) {
        help();
    } else if (strcmp(cmd, "p") == 0) {
        printTunables();
    } else if (strcmp(cmd, "x") == 0) {
        Serial.println(F("# idle"));
    } else if (strcmp(cmd, "b") == 0) {
        calibrateGyroBias();
    } else if (strcmp(cmd, "w") == 0) {
        Serial.print(F("# where odo_mm="));
        Serial.print(g_per.motion.distMm, 1);
        Serial.print(F(" hdg="));
        Serial.print(g_per.motion.headingDeg, 1);
        Serial.print(F(" encL="));
        Serial.print(g_state.encoderCountL);
        Serial.print(F(" encR="));
        Serial.print(g_state.encoderCountR);
        Serial.print(F(" front="));
        Serial.print(g_state.distFrontMm);
        Serial.print(F(" left="));
        Serial.print(g_state.distLeftMm);
        Serial.print(F(" right="));
        Serial.print(g_state.distRightMm);
        Serial.print(F(" valid="));
        Serial.println(g_state.usValid);
    } else if (strcmp(cmd, "set") == 0) {
        const char *key = strtok(nullptr, " \t");
        const char *val = strtok(nullptr, " \t");
        if (key == nullptr || val == nullptr) {
            Serial.println(F("# usage: set <name> <value>"));
            return;
        }
        const float v = static_cast<float>(atof(val));
        if (strcmp(key, "minl") == 0 || strcmp(key, "minr") == 0) {
            const uint16_t m = static_cast<uint16_t>(v < 0 ? 0 : (v > DUTY_CAP ? DUTY_CAP : v));
            (key[3] == 'l' ? g_tune.ctl.minPwmL : g_tune.ctl.minPwmR) = m;
            printTunables();
            return;
        }
        for (const Name &n : g_names) {
            if (strcmp(key, n.key) == 0) {
                *n.value = v;
                printTunables();
                return;
            }
        }
        Serial.println(F("# unknown name"));
    } else if (strcmp(cmd, "o") == 0) {
        const char *a = strtok(nullptr, " \t");
        const char *b = strtok(nullptr, " \t");
        const char *c = strtok(nullptr, " \t");
        if (a == nullptr || b == nullptr || c == nullptr) {
            Serial.println(F("# usage: o <L> <R> <ms>"));
            return;
        }
        g_openL = clampDuty(static_cast<int16_t>(atoi(a)));
        g_openR = clampDuty(static_cast<int16_t>(atoi(b)));
        const long ms = atol(c);
        g_openMs = static_cast<uint16_t>(ms < 50 ? 50 : (ms > OPEN_MAX_MS ? OPEN_MAX_MS : ms));
        Serial.print(F("# open loop L="));
        Serial.print(g_openL);
        Serial.print(F(" R="));
        Serial.print(g_openR);
        Serial.print(F(" for "));
        Serial.print(g_openMs);
        Serial.println(F(" ms"));
        beginMove(Mode::Open);
    } else if (strcmp(cmd, "s") == 0) {
        const char *a = strtok(nullptr, " \t");
        if (a == nullptr || !needHeading()) {
            return;
        }
        float mm = static_cast<float>(atof(a));
        mm = mm > DIST_CAP_MM ? DIST_CAP_MM : (mm < -DIST_CAP_MM ? -DIST_CAP_MM : mm);
        beginMove(Mode::Straight);
        straightBegin(g_straight, g_per.motion, mm, g_tune.vMax, 0.0f, g_per.motion.headingDeg);
        Serial.print(F("# straight "));
        Serial.print(mm, 0);
        Serial.println(F(" mm"));
    } else if (strcmp(cmd, "t") == 0) {
        const char *a = strtok(nullptr, " \t");
        if (a == nullptr || !needHeading()) {
            return;
        }
        float deg = static_cast<float>(atof(a));
        deg = deg > 360.0f ? 360.0f : (deg < -360.0f ? -360.0f : deg);
        beginMove(Mode::Turn);
        turnBegin(g_turn, g_per.motion, headingWrapDeg(g_per.motion.headingDeg + deg), deg);
        Serial.print(F("# turn "));
        Serial.print(deg, 0);
        Serial.println(F(" deg"));
    } else {
        Serial.println(F("# unknown command (h for help)"));
    }
}

// One line, built a byte at a time. While a move runs, any byte aborts it instead.
void pollSerial()
{
    static char line[48];
    static uint8_t len = 0;
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (g_mode != Mode::Idle) {
            endMove("aborted-by-host");
            len = 0;
            continue;
        }
        if (ch == '\n' || ch == '\r') {
            line[len] = '\0';
            len = 0;
            handleLine(line);
        } else if (len < sizeof(line) - 1) {
            line[len++] = ch;
        }
    }
}

} // namespace

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    motorsInit(); // EN low: the wheels coast until a command starts a move
    encodersInit();
    lineSensorsInit();
    g_imuPresent = imuInit();
    if (g_imuPresent) {
        calibrateGyroBias();
    } else {
        Serial.println(F("# IMU MISSING: only open-loop moves"));
    }
    ultrasonicInit();

    tuneDefaults();
    g_pp.mmPerCount = MM_PER_ENCODER_COUNT;
    g_pp.dtMs = CONTROL_LOOP_PERIOD_MS;
    perceiverReset(g_perceiver);
    Serial.println(F("# motion probe ready - motors off. h for help"));
    printTunables();
}

void loop()
{
    static uint32_t nextTickMs = 0;
    const uint32_t nowMs = millis();
    if (!tickDue(nowMs, nextTickMs, CONTROL_LOOP_PERIOD_MS)) {
        return;
    }
    g_state.timestampMs = nowMs;
    readSensors();
    perceive(g_perceiver, g_state, g_pp, g_per);
    pollSerial();
    if (g_mode != Mode::Idle) {
        stepMove();
    }
}
