// ============================================================================
//  Bench probe: motion test run. Drives the real MotionControl / MotionPrimitives code with
//  parameters that serial commands set live, logs each move into RAM and dumps it afterwards.
//  Not part of the mission build. docs: none - throwaway tooling for tuning RobotSpec.h.
//
//  The motors NEVER move on boot or reset. Only a command starts a move, and any byte received
//  while moving aborts it. Hard limits that do not depend on the controller or the host:
//    duty cap, time cap, distance cap (not in a line run), front cut-off, and a
//    pushed-but-not-moving cut-off.
//
//  Commands (one line, end with newline):
//    h                       help
//    p                       print the tunables
//    set <name> <value>      change a tunable (names in `p`)
//    w                       where: odometer, heading, encoders and the three ultrasonics
//    b                       recapture the gyro bias (robot held still) and zero the heading
//    o <L> <R> <ms>          open loop: fixed signed duty on each wheel (max +-DUTY_CAP, 2000 ms)
//    s <mm>                  closed-loop straight, signed (max +-DIST_CAP_MM)
//    k / f                   start / finish a line calibration capture (slide over white and black)
//    K <mm> [v]              calibrate by driving: a slow straight, white floor onto black
//    m [v] [mm]              approach: a slow straight (default 70 mm/s) that runs the marker and
//    gap
//                            detector live, then stops once the gap is behind the array
//    L [mm]                  follow the line with the real follower (needs k/f first)
//    i                       the 8 raw IR counts right now (check what is under the array first)
//    r <mm> [v]              scan: slow straight logging the odometer with the 8 IR channels
//    c                       one mission cell step: wall vote, front-anchored stop
//    a <heading> <dir>       turn to an exact heading, dir +1 CCW / -1 CW (what the mission does)
//    t <deg>                 closed-loop turn in place, CCW positive
//    x                       stop (also: any byte during a move)
// ============================================================================

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "Heading.h"
#include "LineFollow.h"
#include "LineSense.h"
#include "MotionControl.h"
#include "MotionPrimitives.h"
#include "NavParams.h"
#include "Perception.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Scheduler.h"
#include "StepCell.h"
#include "WallSense.h"
#include "hal/encoders.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"
#include "line_cal_store.h"

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

// A scan (`r`) logs the IR array instead: the odometer and the 8 raw counts / 4 per sample. It
// reuses the log's memory.
constexpr uint8_t SCAN_MAX = 200; // one per 3 ticks, 6 s
constexpr uint8_t SCAN_EVERY = 3;
constexpr uint16_t SCAN_MAX_MS = 14000; // a scan may run this long (incl. the hold)
constexpr uint16_t SCAN_TIMEOUT_MS = 8000;
constexpr int16_t SCAN_HOLD_MS = 6000; // after the end of a scan the motors hold the position
struct ScanSample {
    int16_t dist; // 0.1 mm
    uint8_t ir[IR_CHANNEL_COUNT];
    int8_t tilt10; // tilt in 0.1 deg, clamped to +-12.7
};
// A line-follow run logs one of these every LINE_EVERY ticks.
constexpr uint8_t LINE_EVERY_DEFAULT = 16; // 160 ms, so 200 samples = 32 s
struct LineSample {
    int16_t dist; // mm
    int16_t err;  // line error, 0.1 mm (0 with no line in view)
    int16_t speed;
    uint8_t mask;  // black channels, bit = channel (0 = the robot's right end)
    uint8_t flags; // bit0 line in view, bit1 all black, bit2 all white, bit3 air; bits 4-6 the mode
    int8_t tilt10;
};
union LogStore {
    Sample s[LOG_MAX];
    ScanSample c[SCAN_MAX];
    LineSample l[SCAN_MAX];
};
static_assert(sizeof(ScanSample) * SCAN_MAX <= sizeof(Sample) * LOG_MAX, "scan log fits the log");
LogStore g_store;
uint8_t g_logCount = 0;
bool g_scan = false; // the running straight is a scan

// Approach (`m`): the marker / gap detector run live on a slow straight.
struct ApEvent {
    int16_t pos;  // array sensing line, mm from where the axle started
    uint8_t kind; // 0 black 1 white 2 air (a class change), 10 marker, 11 gap, 12 stop
    uint8_t ir[IR_CHANNEL_COUNT]; // raw / 4
};
constexpr uint8_t AP_MAX = 40;
ApEvent g_ap[AP_MAX];
uint8_t g_apCount = 0;
bool g_approach = false;
FloorPattern g_pat;
FloorClass g_apLast = FloorClass::Mixed;
bool g_apMarker = false;
bool g_apGap = false;
float g_apMarkerCentre = 0.0f;
float g_apAfterGapMm = -1.0f; // array position where the floor stopped being air after the gap
BridgeHandover g_apBridge;
float g_apFollowMm = 0.0f; // m <v> <mm>: follow the line this far after the handover; 0 = just stop
bool g_apRun = false;      // this run was started by `m` (its events are printed at the end)
float g_apHandoverMm = -1.0f;
float g_scanRestoreV = 0.0f;
uint16_t g_scanRestoreTimeoutMs = 0;
int16_t g_holdLeftMs = -1; // < 0: not holding yet
float g_holdMm = 0.0f;
float g_holdHeadingDeg = 0.0f;
uint16_t g_ir[IR_CHANNEL_COUNT];

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
    float lead = static_cast<float>(WALL_SAMPLE_LEAD_MM);
    float win = static_cast<float>(WALL_SAMPLE_WINDOW_MM);
    float fstop = static_cast<float>(FRONT_STOP_MM);
    // Line following (`L`). lslow is the slowdown percent, llost the lost-line timeout in ms.
    float kpl = 0.0f;       // deg of heading offset per mm of line error
    float kdl = 0.0f;       // ... per mm/s
    float lsmax = 15.0f;    // clamp on that offset, deg
    float lv = 100.0f;      // speed on the line, mm/s
    float lacc = 300.0f;    // its acceleration
    float lslow = 60.0f;    // speed kept at the largest error, percent
    float lslowerr = 20.0f; // the error at which that is reached, mm
    float lsearch = 60.0f;  // spin rate when the line is lost, deg/s
    float llost = 3000.0f;  // lost-line timeout, ms
    float lendlost = 80.0f; // no line (white) this far after it was seen = the line has ended, mm
    float lramp =
        0.0f; // tilt above which an all-white array is ignored (the lifted array), deg; 0 = never
    float lsspeed = 0.0f; // forward speed while the line is out of sight, mm/s; 0 = stop
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
    {"kih", &g_tune.ctl.kiHold},
    {"ffr", &g_tune.ctl.ffScaleR},
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
    {"lead", &g_tune.lead},
    {"win", &g_tune.win},
    {"fstop", &g_tune.fstop},
    {"kpl", &g_tune.kpl},
    {"kdl", &g_tune.kdl},
    {"lsmax", &g_tune.lsmax},
    {"lv", &g_tune.lv},
    {"lacc", &g_tune.lacc},
    {"lslow", &g_tune.lslow},
    {"lslowerr", &g_tune.lslowerr},
    {"lsearch", &g_tune.lsearch},
    {"llost", &g_tune.llost},
    {"lendlost", &g_tune.lendlost},
    {"lramp", &g_tune.lramp},
    {"lsspeed", &g_tune.lsspeed},
};

void tuneDefaults()
{
    // The values in RobotSpec.h, so a probe run tests what the mission will use. `set` changes them
    // live; nothing is written back.
    g_tune.ctl = MotionControlParams{};
    g_tune.ctl.pwmTop = MOTOR_PWM_TOP;
    g_tune.ctl.ffPwmPerMmS = FEEDFORWARD_PWM_PER_MM_S;
    g_tune.ctl.ffPwmPerDps = FEEDFORWARD_PWM_PER_DPS;
    g_tune.ctl.ffScaleR = FEEDFORWARD_RIGHT_SCALE;
    g_tune.ctl.kpDist = KP_DISTANCE;
    g_tune.ctl.kiDist = KI_DISTANCE;
    g_tune.ctl.kdDist = KD_DISTANCE;
    g_tune.ctl.kpTurn = KP_TURN;
    g_tune.ctl.kiTurn = KI_TURN;
    g_tune.ctl.kdTurn = KD_TURN;
    g_tune.ctl.kiHold = KI_TURN_HOLD;
    g_tune.plan.accelMmS2 = ACCEL_MM_S2;
    g_tune.plan.turnRateMaxDps = TURN_RATE_MAX_DPS;
    g_tune.plan.turnAccelDps2 = TURN_ACCEL_DPS2;
    g_tune.plan.stepToleranceMm = STEP_TOLERANCE_MM;
    g_tune.plan.turnToleranceDeg = TURN_TOLERANCE_DEG;
    g_tune.plan.turnSettleMs = TURN_SETTLE_MS;
    g_tune.plan.moveTimeoutMs = MOVE_TIMEOUT_MS;
    g_tune.vMax = SEARCH_SPEED_MM_S;
    g_tune.wall = 0.0f;
    g_tune.side = static_cast<float>(WALL_PRESENT_SIDE_MM);
    g_tune.front = static_cast<float>(WALL_PRESENT_FRONT_MM);
    g_tune.center = static_cast<float>(WALL_CENTER_DIST_MM);
    g_tune.kpw = KP_WALL;
    g_tune.kdw = KD_WALL;
    g_tune.smax = STEER_MAX_DEG;
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

enum class Mode : uint8_t { Idle, Open, Straight, Turn, Cell, Line };
Mode g_mode = Mode::Idle;

MotionControl g_mc;
Straight g_straight;
Turn g_turn;
StepCell g_cell;
bool g_cellReported = false;
LineFollow g_line;
uint8_t g_lineEvery = LINE_EVERY_DEFAULT;
float g_lineLimitMm = 0.0f; // L <mm>: stop the run after this much travel; 0 = no limit
LineStatus g_lineStatus = LineStatus::Following;
LineCalibration g_cal;
bool g_capturing = false;
uint16_t g_flatSkipped = 0; // sweeps of a capture that looked like an unpowered array
// Static calibration: W and B average the array over 1.5 s standing still on white and on black.
constexpr uint16_t STATIC_TICKS = 150;
char g_staticWhich = 0; // 'W', 'B', or 0 = not measuring
uint16_t g_staticN = 0;
uint32_t g_staticSum[IR_CHANNEL_COUNT];
uint32_t g_staticSumSq[IR_CHANNEL_COUNT];
uint16_t g_meanW[IR_CHANNEL_COUNT];
uint16_t g_meanB[IR_CHANNEL_COUNT];
bool g_haveW = false;
bool g_haveB = false;
bool g_calDrive = false; // the running straight is a calibration drive (K)
float g_calRestoreV = 0.0f;
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
    Serial.println(F(
        "# h | p | w | i | k | f | K <mm> [v] | m [v] [mm] | L [mm] | c | r <mm> [v] | set <name> "
        "<v> | b | o <L> "
        "<R> <ms> | s <mm> "
        "| t <deg> | x"));
    Serial.println(
        F("# names: kpd kid kdd kpt kit kdt ffv ffd v acc rate tacc stol ttol minl minr"));
}

StepCellParams cellParams()
{
    StepCellParams p;
    p.plan = g_tune.plan;
    p.wall = wallParams();
    p.pitchMm = static_cast<float>(TILE_PITCH_MM);
    p.cruiseMmS = g_tune.vMax;
    p.sampleLeadMm = g_tune.lead;
    p.sampleWindowMm = g_tune.win;
    p.frontStopMm = g_tune.fstop;
    return p;
}

// A real calibration has a white level well above the ambient-only ~130 and channels that differ
// from each other by tens of counts; eight identical low whites is an array without its emitter
// supply.
bool calibrationLooksReal(const LineCalibration &cal)
{
    uint16_t lo = 0xFFFF;
    uint16_t hi = 0;
    uint8_t usable = 0;
    for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
        lo = cal.white[k] < lo ? cal.white[k] : lo;
        hi = cal.white[k] > hi ? cal.white[k] : hi;
        usable = static_cast<uint8_t>(usable + (cal.scale[k] != 0 ? 1 : 0));
    }
    return usable > 0 && lo >= 400 && (hi - lo) >= 15;
}

// The two means are in: build the calibration from them, check it, save it.
void buildStaticCalibration()
{
    // A channel that has not seen black (the 24 mm line only covers about 3 channels at a time)
    // takes the typical black of this robot: the tape reads 1005-1020, the black area 1016-1023.
    constexpr uint16_t DEFAULT_BLACK = 1020;
    for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
        g_cal.white[k] = g_meanW[k];
        g_cal.black[k] = g_meanB[k] > g_meanW[k] + 50u ? g_meanB[k] : DEFAULT_BLACK;
        g_cal.scale[k] = 0;
    }
    const uint8_t usable = lineCalibrationFinish(g_cal);
    if (!calibrationLooksReal(g_cal)) {
        Serial.println(F("# REJECTED: whites too low or alike, or no range - measure again"));
        return;
    }
    perceiverSetCalibration(g_perceiver, g_cal);
    lineCalSave(g_cal);
    Serial.println(F("# calibration from the two means (saved to EEPROM): ch,white,black,range"));
    for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
        Serial.print(k);
        Serial.print(',');
        Serial.print(g_cal.white[k]);
        Serial.print(',');
        Serial.print(g_cal.black[k]);
        Serial.print(',');
        Serial.println(static_cast<int>(g_cal.black[k]) - static_cast<int>(g_cal.white[k]));
    }
    Serial.print(F("# usable mask 0x"));
    Serial.println(usable, HEX);
}

void finishStatic()
{
    const char which = g_staticWhich;
    g_staticWhich = 0;
    uint16_t means[IR_CHANNEL_COUNT];
    uint16_t lo = 0xFFFF;
    uint16_t hi = 0;
    float worstSd = 0.0f;
    Serial.print(F("# "));
    Serial.print(which == 'W' ? F("white") : F("black"));
    Serial.print(F(" means:"));
    for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
        const float mean = static_cast<float>(g_staticSum[k]) / static_cast<float>(g_staticN);
        const float var =
            static_cast<float>(g_staticSumSq[k]) / static_cast<float>(g_staticN) - mean * mean;
        const float sd = var > 0.0f ? sqrtf(var) : 0.0f;
        worstSd = sd > worstSd ? sd : worstSd;
        means[k] = static_cast<uint16_t>(mean + 0.5f);
        lo = means[k] < lo ? means[k] : lo;
        hi = means[k] > hi ? means[k] : hi;
        Serial.print(' ');
        Serial.print(means[k]);
    }
    Serial.print(F("   worst sd "));
    Serial.println(worstSd, 1);
    if (worstSd > 6.0f) {
        Serial.println(F("# NOT STILL: the readings moved by more than 6 counts - measure again"));
        return;
    }
    if (which == 'W') {
        if (lo < 400) {
            Serial.println(F("# REJECTED: white this low means the array is unpowered or covered"));
            return;
        }
        g_haveW = true;
    } else {
        // Keep the highest mean each channel has shown over the black measurements so far.
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            g_meanB[k] = means[k] > g_meanB[k] ? means[k] : g_meanB[k];
        }
        g_haveB = true;
    }
    if (which == 'W') {
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            g_meanW[k] = means[k];
        }
    }
    if (g_haveW && g_haveB) {
        buildStaticCalibration();
    } else {
        Serial.println(g_haveW ? F("# now measure BLACK (B)") : F("# now measure WHITE (W)"));
    }
}

LineParams lineParams()
{
    LineParams p;
    p.kp = g_tune.kpl;
    p.kd = g_tune.kdl;
    p.steerMaxDeg = g_tune.lsmax;
    p.speedMmS = g_tune.lv;
    p.accelMmS2 = g_tune.lacc;
    p.slowdownErrMm = g_tune.lslowerr;
    p.slowdownPct = static_cast<uint8_t>(
        g_tune.lslow < 0.0f ? 0.0f : (g_tune.lslow > 100.0f ? 100.0f : g_tune.lslow));
    p.searchRateDps = g_tune.lsearch;
    p.lostTimeoutMs = static_cast<uint16_t>(g_tune.llost);
    p.endLostMm = g_tune.lendlost;
    p.rampTiltDeg = g_tune.lramp;
    p.searchSpeedMmS = g_tune.lsspeed;
    return p;
}

void beginMove(Mode mode)
{
    g_cellReported = false;
    g_scan = false;
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

void handleLine(char *line);

constexpr float AP_STOP_AFTER_GAP_MM = 8.0f; // the array has been off the gap this far
constexpr float AP_NO_GAP_MM = 300.0f;       // a marker with no gap this far past its centre

void apRecord(float pos, uint8_t kind)
{
    if (g_apCount >= AP_MAX) {
        return;
    }
    ApEvent &e = g_ap[g_apCount++];
    e.pos = static_cast<int16_t>(pos);
    e.kind = kind;
    for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
        e.ir[k] = static_cast<uint8_t>(g_ir[k] >> 2);
    }
}

void apReport()
{
    Serial.print(F("# marker "));
    Serial.print(g_apMarker ? F("FOUND, box centre at ") : F("not found"));
    if (g_apMarker) {
        Serial.print(g_apMarkerCentre, 0);
        Serial.print(F(" mm"));
    }
    Serial.print(F("   gap "));
    Serial.println(g_apGap ? F("FOUND") : F("not found"));
    Serial.println(F("# events: array_pos_mm,kind,ir0..ir7 (raw/4)  kind 0 black 1 white 2 air "
                     "10 marker 11 gap 12 stop 13 handover"));
    for (uint8_t i = 0; i < g_apCount; ++i) {
        Serial.print(g_ap[i].pos);
        Serial.print(',');
        Serial.print(g_ap[i].kind);
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            Serial.print(',');
            Serial.print(g_ap[i].ir[k]);
        }
        Serial.println();
    }
}

void endMove(const char *reason)
{
    motorSet(Motor::Left, 0);
    motorSet(Motor::Right, 0);
    motorsEnable(false);
    const Mode was = g_mode;
    const bool wasScan = g_scan;
    const bool wasCal = g_calDrive;
    if (g_calDrive) {
        g_tune.vMax = g_calRestoreV;
        g_tune.plan.moveTimeoutMs = g_scanRestoreTimeoutMs;
        g_calDrive = false;
    }
    g_mode = Mode::Idle;
    g_reason = reason;
    if (was == Mode::Line) {
        g_tune.plan.moveTimeoutMs = g_scanRestoreTimeoutMs;
    }
    if (g_scan) {
        g_tune.vMax = g_scanRestoreV;
        g_tune.plan.moveTimeoutMs = g_scanRestoreTimeoutMs;
        g_scan = false;
    }
    const bool wasApproach = g_apRun;
    if (g_apRun) {
        g_tune.vMax = g_scanRestoreV;
        g_tune.plan.moveTimeoutMs = g_scanRestoreTimeoutMs;
        g_approach = false;
        g_apRun = false;
    }
    g_lineEvery = LINE_EVERY_DEFAULT;

    Serial.print(F("# end "));
    Serial.print(was == Mode::Open
                     ? F("open")
                     : (was == Mode::Straight
                            ? F("straight")
                            : (was == Mode::Cell ? F("cell")
                                                 : (was == Mode::Line ? F("line") : F("turn")))));
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
    if (was == Mode::Line) {
        if (wasApproach) {
            apReport();
            Serial.print(F("# handover at array position "));
            Serial.print(g_apHandoverMm, 0);
            Serial.println(F(" mm"));
        }
        Serial.println(F("# line dist_mm,err_mm,speed,mask,line,black,white,air,mode,tilt"));
        for (uint8_t i = 0; i < g_logCount; ++i) {
            const LineSample &c = g_store.l[i];
            Serial.print(c.dist);
            Serial.print(',');
            Serial.print(c.err / 10.0f, 1);
            Serial.print(',');
            Serial.print(c.speed);
            Serial.print(',');
            for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
                Serial.print(((c.mask >> k) & 1u) ? '#' : '.');
            }
            Serial.print(',');
            Serial.print(c.flags & 1u);
            Serial.print(',');
            Serial.print((c.flags >> 1) & 1u);
            Serial.print(',');
            Serial.print((c.flags >> 2) & 1u);
            Serial.print(',');
            Serial.print((c.flags >> 3) & 1u);
            Serial.print(',');
            Serial.print((c.flags >> 4) & 7u);
            Serial.print(',');
            Serial.println(c.tilt10 / 10.0f, 1);
        }
        Serial.println(F("# done"));
        return;
    }
    if (wasApproach && was != Mode::Line) {
        apReport();
        Serial.println(F("# done"));
        return;
    }
    if (was == Mode::Straight && wasScan) {
        Serial.println(F("# scan dist_mm,ir0..ir7 (raw counts / 4),tilt (deg), one row per 30 ms"));
        for (uint8_t i = 0; i < g_logCount; ++i) {
            const ScanSample &c = g_store.c[i];
            Serial.print(c.dist / 10.0f, 1);
            for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
                Serial.print(',');
                Serial.print(c.ir[k]);
            }
            Serial.print(',');
            Serial.println(c.tilt10 / 10.0f, 1);
        }
        Serial.println(F("# done"));
        return;
    }
    Serial.println(
        F("# t_ms,dist,distRef,speed,speedRef,hdg,hdgRef,yaw,dutyL,dutyR,left,right,trim "
          "(dist/hdg/yaw/trim x10)"));
    for (uint8_t i = 0; i < g_logCount; ++i) {
        const Sample &s = g_store.s[i];
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
    if (wasCal) {
        char finish[] = "f"; // end the capture the drive was making
        handleLine(finish);
    }
}

void logSample(const MotionTarget &t, int16_t dutyL, int16_t dutyR, float trimDeg)
{
    if (g_tick % LOG_EVERY != 0 || g_logCount >= LOG_MAX) {
        return;
    }
    Sample &s = g_store.s[g_logCount++];
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
    // A line run has no time or distance cap: it runs until the follower reports the end or the
    // line lost (or the front-wall / duty / pushed-but-still cut-offs below, or `x`).
    const bool line = g_mode == Mode::Line;
    if (!line) {
        if (g_elapsedMs >= ((g_scan || g_approach || g_calDrive) ? SCAN_MAX_MS : MOVE_MAX_MS)) {
            return "time-cap";
        }
        const float travelled = g_per.motion.distMm - g_startDistMm;
        if (travelled > DIST_CAP_MM || travelled < -DIST_CAP_MM) {
            return "dist-cap";
        }
    }
    if ((g_mode == Mode::Straight || g_mode == Mode::Line) && (g_state.usValid & 1u) != 0 &&
        g_state.distFrontMm <= FRONT_CUT_MM && g_per.motion.speedMmS > 20.0f) {
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
        if (g_mode == Mode::Cell) {
            const StepCellParams cp = cellParams();
            st = stepCellStep(g_cell, cp, fb, wallView(g_state, cp.wall), CONTROL_LOOP_PERIOD_MS,
                              target);
            if (g_cell.wallsReady && !g_cellReported) {
                g_cellReported = true;
                Serial.print(F("# walls front="));
                Serial.print(g_cell.walls.front);
                Serial.print(F(" left="));
                Serial.print(g_cell.walls.left);
                Serial.print(F(" right="));
                Serial.print(g_cell.walls.right);
                Serial.print(F(" unreliable="));
                Serial.print(g_cell.walls.unreliable);
                Serial.print(F(" at dist="));
                Serial.println(fb.distMm - g_startDistMm, 0);
            }
        } else if (g_mode == Mode::Straight) {
            if (g_tune.wall > 0.5f) {
                const WallParams wp = wallParams();
                trim = wallSteerTrim(g_steer, wallView(g_state, wp), wp, CONTROL_LOOP_PERIOD_MS);
            }
            if (g_scan && g_holdLeftMs >= 0) {
                // The scan is over: hold this position, so the robot does not roll on a slope.
                target.distRefMm = g_holdMm;
                target.headingRefDeg = g_holdHeadingDeg;
                g_holdLeftMs = static_cast<int16_t>(g_holdLeftMs - CONTROL_LOOP_PERIOD_MS);
                st = g_holdLeftMs <= 0 ? MoveStatus::Done : MoveStatus::Running;
            } else {
                st =
                    straightStep(g_straight, g_tune.plan, fb, trim, CONTROL_LOOP_PERIOD_MS, target);
                if (g_approach && reason == nullptr) {
                    const float pos = fb.distMm - g_startDistMm + IR_ARRAY_FORWARD_OFFSET_MM;
                    float centre = 0.0f;
                    const PatternEvent ev =
                        floorPatternStep(g_pat, g_per.floor, pos, specPattern(), centre);
                    if (g_per.floor != FloorClass::Mixed && g_per.floor != g_apLast) {
                        g_apLast = g_per.floor;
                        apRecord(pos, static_cast<uint8_t>(g_per.floor));
                    }
                    if (ev == PatternEvent::Marker) {
                        g_apMarker = true;
                        g_apMarkerCentre = centre;
                        apRecord(pos, 10);
                    } else if (ev == PatternEvent::Gap) {
                        g_apGap = true;
                        apRecord(pos, 11);
                    }
                    if (g_apFollowMm > 0.0f &&
                        bridgeHandoverStep(g_apBridge, g_apGap, g_per.floor, g_per.line, pos,
                                           BRIDGE_HANDOVER_MIN_MM)) {
                        // The line is in view past the gap: the line follower takes over,
                        // mid-stride.
                        apRecord(pos, 13);
                        g_apHandoverMm = pos;
                        g_approach = false;
                        g_mode = Mode::Line;
                        g_lineEvery = 8;
                        g_logCount = 0;
                        g_lineLimitMm = (fb.distMm - g_startDistMm) + g_apFollowMm;
                        lineFollowBegin(g_line, fb, true);
                        const bool blockedNow =
                            (g_state.usValid & 1u) != 0 && g_state.distFrontMm <= FRONT_CUT_MM;
                        g_lineStatus =
                            lineFollowStep(g_line, lineParams(), fb, g_per.line, g_per.tiltDeg,
                                           blockedNow, CONTROL_LOOP_PERIOD_MS, target);
                        st = MoveStatus::Running;
                    } else if (g_apGap && g_apFollowMm <= 0.0f) {
                        if (g_per.floor == FloorClass::Air) {
                            g_apAfterGapMm = -1.0f;
                        } else if (g_apAfterGapMm < 0.0f) {
                            g_apAfterGapMm = pos;
                        } else if (pos - g_apAfterGapMm >= AP_STOP_AFTER_GAP_MM) {
                            apRecord(pos, 12);
                            reason = "gap-done";
                        }
                    } else if (g_apMarker && pos - g_apMarkerCentre >= AP_NO_GAP_MM) {
                        apRecord(pos, 12);
                        reason = g_apGap ? "no-line-after-gap" : "no-gap";
                    }
                }
                if (g_scan && st == MoveStatus::Done) {
                    g_holdLeftMs = SCAN_HOLD_MS;
                    g_holdMm = target.distRefMm;
                    g_holdHeadingDeg = target.headingRefDeg;
                    st = MoveStatus::Running;
                }
            }
        } else if (g_mode == Mode::Line) {
            const bool blocked = (g_state.usValid & 1u) != 0 && g_state.distFrontMm <= FRONT_CUT_MM;
            g_lineStatus = lineFollowStep(g_line, lineParams(), fb, g_per.line, g_per.tiltDeg,
                                          blocked, CONTROL_LOOP_PERIOD_MS, target);
            st = MoveStatus::Running;
            if (g_lineStatus == LineStatus::Ended) {
                st = MoveStatus::Done;
            } else if (g_lineStatus == LineStatus::Lost) {
                st = MoveStatus::Timeout;
            }
        } else {
            st = turnStep(g_turn, g_tune.plan, fb, CONTROL_LOOP_PERIOD_MS, target);
        }
        const MotionOutput out =
            motionControlStep(g_mc, g_tune.ctl, target, fb, CONTROL_LOOP_PERIOD_MS);
        l = clampDuty(out.left);
        r = clampDuty(out.right);
        if (st == MoveStatus::Done) {
            reason = g_mode == Mode::Line ? "line-ended" : "arrived";
        } else if (st == MoveStatus::Timeout) {
            reason = g_mode == Mode::Line ? "line-lost" : "move-timeout";
        }
    }

    if (reason == nullptr && g_mode == Mode::Line && g_lineLimitMm > 0.0f &&
        (g_per.motion.distMm - g_startDistMm) >= g_lineLimitMm) {
        reason = "dist-limit";
    }
    if (reason == nullptr) {
        reason = hardLimit(l, r);
    }
    if (g_mode == Mode::Line) {
        if (g_tick % g_lineEvery == 0) {
            if (g_logCount >= SCAN_MAX) { // a long run keeps the LAST samples
                memmove(&g_store.l[0], &g_store.l[1], sizeof(LineSample) * (SCAN_MAX - 1));
                --g_logCount;
            }
            LineSample &c = g_store.l[g_logCount++];
            const float mm = g_per.motion.distMm - g_startDistMm;
            c.dist =
                static_cast<int16_t>(mm > 32000.0f ? 32000.0f : (mm < -32000.0f ? -32000.0f : mm));
            c.err = toTenths(g_per.line.errMm);
            c.speed = static_cast<int16_t>(g_per.motion.speedMmS);
            c.mask = g_perceiver.mask;
            const uint8_t mode = static_cast<uint8_t>(g_line.mode);
            c.flags = static_cast<uint8_t>(
                (g_per.line.onLine ? 1u : 0u) | (g_per.line.allBlack ? 2u : 0u) |
                (g_per.line.allWhite ? 4u : 0u) | (g_per.floor == FloorClass::Air ? 8u : 0u) |
                (mode << 4));
            const float t10 = g_per.tiltDeg * 10.0f;
            c.tilt10 = static_cast<int8_t>(t10 > 127.0f ? 127.0f : t10);
        }
    } else if (g_scan) {
        if (g_tick % SCAN_EVERY == 0 && g_logCount < SCAN_MAX) {
            ScanSample &c = g_store.c[g_logCount++];
            c.dist = toTenths(g_per.motion.distMm - g_startDistMm);
            const float t10 = g_per.tiltDeg * 10.0f;
            c.tilt10 = static_cast<int8_t>(t10 > 127.0f ? 127.0f : (t10 < -127.0f ? -127.0f : t10));
            for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
                c.ir[k] = static_cast<uint8_t>(g_ir[k] >> 2);
            }
        }
    } else {
        logSample(target, l, r, trim);
    }
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
    } else if (strcmp(cmd, "c") == 0) {
        if (!needHeading()) {
            return;
        }
        beginMove(Mode::Cell);
        stepCellBegin(g_cell, cellParams(), g_per.motion, g_per.motion.headingDeg);
        Serial.println(F("# cell step"));
    } else if (strcmp(cmd, "a") == 0) {
        // Absolute turn: end exactly on <heading>, going CCW (+1) or CW (-1), like the mission's.
        const char *a = strtok(nullptr, " \t");
        const char *b = strtok(nullptr, " \t");
        if (a == nullptr || b == nullptr || !needHeading()) {
            return;
        }
        const float target = headingWrapDeg(static_cast<float>(atof(a)));
        const float dir = atof(b) >= 0 ? 1.0f : -1.0f;
        float sweep = headingWrapDeg(target - g_per.motion.headingDeg);
        if (dir > 0.0f && sweep <= 0.0f) {
            sweep += 360.0f;
        } else if (dir < 0.0f && sweep >= 0.0f) {
            sweep -= 360.0f;
        }
        beginMove(Mode::Turn);
        turnBegin(g_turn, g_per.motion, target, sweep);
        Serial.print(F("# turn to "));
        Serial.print(target, 1);
        Serial.print(F(" sweep "));
        Serial.println(sweep, 1);
    } else if (strcmp(cmd, "k") == 0) {
        lineCalibrationReset(g_cal);
        g_flatSkipped = 0;
        g_capturing = true;
        Serial.println(F("# capturing: slide over a large white AND a large black area, then f"));
    } else if (strcmp(cmd, "W") == 0 || strcmp(cmd, "B") == 0) {
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            g_staticSum[k] = 0;
            g_staticSumSq[k] = 0;
        }
        g_staticN = 0;
        g_staticWhich = cmd[0];
        Serial.print(F("# measuring "));
        Serial.print(cmd[0] == 'W' ? F("WHITE") : F("BLACK"));
        Serial.println(F(": hold still, array flat over the surface (1.5 s)"));
    } else if (strcmp(cmd, "P") == 0) {
        Serial.println(F("# calibration in use: ch,white,black,scale"));
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            Serial.print(k);
            Serial.print(',');
            Serial.print(g_perceiver.cal.white[k]);
            Serial.print(',');
            Serial.print(g_perceiver.cal.black[k]);
            Serial.print(',');
            Serial.println(g_perceiver.cal.scale[k]);
        }
        Serial.print(F("# usable=0x"));
        Serial.print(g_perceiver.usable, HEX);
        Serial.print(F(" norm now:"));
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            Serial.print(' ');
            Serial.print(g_perceiver.norm[k]);
        }
        Serial.println();
    } else if (strcmp(cmd, "K") == 0) {
        // Calibrate by driving: a slow closed-loop straight over the white floor and onto black,
        // capturing as it goes (wheels down, constant height), then the capture is finished.
        const char *a = strtok(nullptr, " \t");
        const char *b = strtok(nullptr, " \t");
        if (a == nullptr || !needHeading()) {
            return;
        }
        float mm = static_cast<float>(atof(a));
        mm = mm < 50.0f ? 50.0f : (mm > 400.0f ? 400.0f : mm);
        const float v = b != nullptr ? static_cast<float>(atof(b)) : 40.0f;
        lineCalibrationReset(g_cal);
        g_flatSkipped = 0;
        g_capturing = true;
        g_calRestoreV = g_tune.vMax;
        g_scanRestoreTimeoutMs = g_tune.plan.moveTimeoutMs;
        g_tune.plan.moveTimeoutMs = SCAN_TIMEOUT_MS;
        g_tune.vMax = v < 20.0f ? 20.0f : (v > 100.0f ? 100.0f : v);
        beginMove(Mode::Straight);
        g_calDrive = true;
        straightBegin(g_straight, g_per.motion, mm, g_tune.vMax, 0.0f, g_per.motion.headingDeg);
        Serial.print(F("# calibration drive "));
        Serial.print(mm, 0);
        Serial.print(F(" mm at "));
        Serial.print(g_tune.vMax, 0);
        Serial.println(F(" mm/s"));
    } else if (strcmp(cmd, "f") == 0) {
        if (!g_capturing) {
            Serial.println(F("# no capture running (k first)"));
            return;
        }
        g_capturing = false;
        const uint8_t usable = lineCalibrationFinish(g_cal);
        Serial.print(F("# sweeps ignored as an unpowered array: "));
        Serial.println(g_flatSkipped);
        if (!calibrationLooksReal(g_cal)) {
            Serial.println(
                F("# REJECTED: the whites are too low or all alike - not saved. Check the "
                  "array has power, keep it over the floor, and repeat."));
            lineCalibrationReset(g_cal);
            return;
        }
        perceiverSetCalibration(g_perceiver, g_cal);
        lineCalSave(g_cal);
        Serial.println(
            F("# calibration (saved to EEPROM): ch,white(low),black(high),range,usable"));
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            Serial.print(k);
            Serial.print(',');
            Serial.print(g_cal.white[k]);
            Serial.print(',');
            Serial.print(g_cal.black[k]);
            Serial.print(',');
            Serial.print(static_cast<int>(g_cal.black[k]) - static_cast<int>(g_cal.white[k]));
            Serial.print(',');
            Serial.println((usable >> k) & 1u);
        }
        Serial.print(F("# usable mask 0x"));
        Serial.println(usable, HEX);
    } else if (strcmp(cmd, "L") == 0) {
        // Follow the line with the real follower, with live-settable parameters.
        if (!needHeading()) {
            return;
        }
        if (g_perceiver.usable == 0) {
            Serial.println(F("# refused: no line calibration (k, slide, f first)"));
            return;
        }
        const char *limit = strtok(nullptr, " \t");
        g_lineLimitMm = limit != nullptr ? static_cast<float>(atof(limit)) : 0.0f;
        g_scanRestoreTimeoutMs = g_tune.plan.moveTimeoutMs;
        g_tune.plan.moveTimeoutMs = 0; // the follower has its own limits
        beginMove(Mode::Line);
        lineFollowBegin(g_line, g_per.motion, true);
        g_lineStatus = LineStatus::Following;
        Serial.println(F("# line follow"));
    } else if (strcmp(cmd, "i") == 0) {
        Serial.print(F("# ir"));
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            Serial.print(' ');
            Serial.print(g_ir[k]);
        }
        Serial.print(F(" | mask "));
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            Serial.print(((g_perceiver.mask >> k) & 1u) ? '#' : '.');
        }
        Serial.print(F(" | "));
        Serial.print(g_per.floor == FloorClass::Air
                         ? F("air")
                         : (g_per.line.allBlack
                                ? F("allBlack")
                                : (g_per.line.allWhite ? F("allWhite") : F("onLine"))));
        Serial.print(F(" err="));
        Serial.print(g_per.line.errMm, 1);
        Serial.print(F(" usable=0x"));
        Serial.println(g_perceiver.usable, HEX);
    } else if (strcmp(cmd, "r") == 0) {
        // Scan: a slow closed-loop straight that logs the odometer with the 8 IR channels.
        const char *a = strtok(nullptr, " \t");
        const char *b = strtok(nullptr, " \t");
        if (a == nullptr || !needHeading()) {
            return;
        }
        float mm = static_cast<float>(atof(a));
        mm = mm > DIST_CAP_MM ? DIST_CAP_MM : (mm < 0.0f ? 0.0f : mm);
        const float v = b != nullptr ? static_cast<float>(atof(b)) : 60.0f;
        g_scanRestoreV = g_tune.vMax;
        g_scanRestoreTimeoutMs = g_tune.plan.moveTimeoutMs;
        g_tune.plan.moveTimeoutMs = SCAN_TIMEOUT_MS;
        g_holdLeftMs = -1;
        g_tune.vMax = v < 20.0f ? 20.0f : (v > 150.0f ? 150.0f : v);
        beginMove(Mode::Straight);
        g_scan = true;
        straightBegin(g_straight, g_per.motion, mm, g_tune.vMax, 0.0f, g_per.motion.headingDeg);
        Serial.print(F("# scan "));
        Serial.print(mm, 0);
        Serial.print(F(" mm at "));
        Serial.print(g_tune.vMax, 0);
        Serial.println(F(" mm/s"));
    } else if (strcmp(cmd, "m") == 0) {
        // Approach: drive slowly forward and watch for the marker, then the gap.
        const char *a = strtok(nullptr, " \t");
        if (!needHeading() || g_perceiver.usable == 0) {
            Serial.println(F("# need a heading and a line calibration"));
            return;
        }
        const float v = a != nullptr ? static_cast<float>(atof(a)) : 70.0f;
        const char *f = strtok(nullptr, " \t");
        g_apFollowMm = f != nullptr ? static_cast<float>(atof(f)) : 0.0f;
        g_apFollowMm =
            g_apFollowMm > 2500.0f ? 2500.0f : (g_apFollowMm < 0.0f ? 0.0f : g_apFollowMm);
        g_apHandoverMm = -1.0f;
        bridgeHandoverReset(g_apBridge);
        g_lineLimitMm = 0.0f;
        g_scanRestoreV = g_tune.vMax;
        g_scanRestoreTimeoutMs = g_tune.plan.moveTimeoutMs;
        g_tune.plan.moveTimeoutMs = SCAN_TIMEOUT_MS;
        g_tune.vMax = v < 40.0f ? 40.0f : (v > 120.0f ? 120.0f : v);
        g_apCount = 0;
        g_apMarker = false;
        g_apGap = false;
        g_apAfterGapMm = -1.0f;
        g_apLast = FloorClass::Mixed;
        floorPatternReset(g_pat);
        beginMove(Mode::Straight);
        g_approach = true;
        g_apRun = true;
        straightBegin(g_straight, g_per.motion, 700.0f, g_tune.vMax, 0.0f, g_per.motion.headingDeg);
        Serial.print(F("# approach at "));
        Serial.print(g_tune.vMax, 0);
        Serial.println(F(" mm/s"));
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
    g_pp.irPitchMm = IR_SENSOR_PITCH_MM;
    g_pp.irD1OnRight = IR_D1_ON_RIGHT;
    g_pp.irAirCounts = IR_AIR_COUNTS;
    g_pp.irAirChannels = IR_AIR_CHANNELS;
    lineCalibrationReset(g_cal);
    perceiverReset(g_perceiver);
    if (lineCalLoad(g_cal) && calibrationLooksReal(g_cal)) {
        perceiverSetCalibration(g_perceiver, g_cal);
        Serial.println(F("# line calibration loaded from EEPROM"));
    } else {
        lineCalibrationReset(g_cal);
        Serial.println(F("# no usable line calibration stored: k, slide, f"));
    }
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
    lineSensorsRead(g_ir);
    for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
        g_state.irRaw[k] =
            g_ir[k]; // what perceive() reads: the line view, the mask, the floor class
    }
    if (g_staticWhich != 0) {
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            g_staticSum[k] += g_ir[k];
            g_staticSumSq[k] += static_cast<uint32_t>(g_ir[k]) * g_ir[k];
        }
        if (++g_staticN >= STATIC_TICKS) {
            finishStatic();
        }
    }
    if (g_capturing) {
        // A sweep where every channel reads (nearly) the same low value is not a surface: the real
        // channels differ from each other by tens of counts. It is the array without its emitter
        // supply, seeing only the room light, and one such sweep would ruin the white level.
        uint16_t lo = 0xFFFF;
        uint16_t hi = 0;
        uint32_t sum = 0;
        for (uint8_t k = 0; k < IR_CHANNEL_COUNT; ++k) {
            lo = g_ir[k] < lo ? g_ir[k] : lo;
            hi = g_ir[k] > hi ? g_ir[k] : hi;
            sum += g_ir[k];
        }
        if ((hi - lo) < 8 && (sum / IR_CHANNEL_COUNT) < 600) {
            ++g_flatSkipped;
        } else {
            lineCalibrationAccumulate(g_cal, g_ir);
        }
    }
    readSensors();
    perceive(g_perceiver, g_state, g_pp, g_per);
    pollSerial();
    if (g_mode != Mode::Idle) {
        stepMove();
    }
}
