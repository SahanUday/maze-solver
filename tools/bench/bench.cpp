// ============================================================================
//  tools/bench/bench.cpp  --  bench firmware: checks every sensor and motor on the robot and takes
//  the measurements the spec/ constants need. Not the mission. Flash it with
//  `pio run -e bench -t upload`, open the serial monitor at 250000 baud and type `h`.
//
//  The motors NEVER move on their own: only `o`, `s` and `t` start them, and any byte received
//  while a command runs stops it. Limits that hold whatever the controller does: a duty cap, a time
//  cap, a distance cap, a front cut-off and a pushed-but-not-moving cut-off.
//
//  Everything above the drivers is the mission's own code: perceive() (sense/Perception.h), the
//  motion controller and moves (motion/MotionControl.h, motion/MotionPrimitives.h) with
//  missionParams(), and the IR calibration (sense/LineSense.h, line_cal_store.h). A number
//  measured here is the number the mission sees.
//
//  Output: lines starting with '#' are for people. Every other line is CSV whose first field names
//  its kind (status, stat, open, move, result, cal), so a saved log can be split with grep.
// ============================================================================

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "app/heading_estimator.h"
#include "core/RobotState.h"
#include "core/Scheduler.h"
#include "hal/encoders.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"
#include "line_cal_store.h"
#include "mission/MissionParams.h"
#include "motion/MotionControl.h"
#include "motion/MotionPrimitives.h"
#include "sense/LineSense.h"
#include "sense/Perception.h"
#include "spec/fixed/Hardware.h"
#include "spec/measurements/Robot.h"
#include "spec/tuning/Motion.h"

namespace
{

// ---- Safety limits ---------------------------------------------------------
constexpr int16_t DUTY_CAP = 600;           // any wheel, any move (MOTOR_PWM_TOP is 799)
constexpr uint16_t OPEN_MAX_MS = 3000;      // an open-loop run
constexpr uint16_t MOVE_MAX_MS = 20000;     // a closed-loop move: 2100 mm at ~110 mm/s
constexpr float DIST_CAP_MM = 2100.0f;      // 8 tiles and a margin, from where the move began
constexpr float SPEED_CAP_MM_S = 600.0f;    // `s` speed
constexpr float TURN_CAP_DEG = 360.0f;      // `t` angle
constexpr uint16_t FRONT_CUT_MM = 130;      // front reading (axle to wall) at or below: stop
constexpr float FRONT_CUT_MIN_MM_S = 20.0f; // ...while driving forward at least this fast
// The front sensor hears nothing below ~114 mm from the axle (tuning/WallFollow.h), so the cut-off
// needs a reading above that to act on.
static_assert(FRONT_CUT_MM > 114, "the front cut-off sits inside the front sensor's dead zone");
constexpr int16_t PUSH_DUTY = 250;      // open loop: pushing at least this hard...
constexpr uint16_t PUSH_STILL_MS = 600; // ...with neither encoder moving this long: stop
constexpr uint16_t PUSH_GRACE_MS = 300; // a wheel may take this long to break away from rest

// ---- Defaults --------------------------------------------------------------
constexpr uint16_t STREAM_DEFAULT_MS = 100;
constexpr uint16_t MEASURE_DEFAULT_MS = 2000;
constexpr uint16_t MEASURE_MAX_MS = 60000;
constexpr uint16_t MOVE_LOG_MS = 20; // a move or open-loop line this often
constexpr uint16_t CAPTURE_LOG_MS = 500;
constexpr uint16_t ABORT_QUIET_MS = 100; // after a stop, the rest of the typed line is dropped

constexpr uint8_t LINE_MAX = 40;
constexpr uint8_t LABEL_MAX = 16;

// ---- State -----------------------------------------------------------------
const MissionParams g_params = missionParams(); // what the mission's moves run with

RobotState g_state;
Perceiver g_pc;
PerceiveParams g_pp;
Perception g_per;
bool g_imuPresent = false;

int32_t g_encZeroL = 0; // the counts `z` called zero
int32_t g_encZeroR = 0;
uint32_t g_tickMaxUs = 0; // slowest tick since the last report

enum class Activity : uint8_t { Idle, Stream, Measure, Capture, Open, Straight, Turn };
Activity g_act = Activity::Idle;
uint32_t g_actStartMs = 0;
uint32_t g_lastLogMs = 0;

// Serial input.
char g_line[LINE_MAX + 1];
uint8_t g_lineLen = 0;
bool g_lineOverflow = false;
uint32_t g_dropUntilMs = 0;

// `v`
uint16_t g_streamMs = STREAM_DEFAULT_MS;

// `m`: Welford running mean and spread per quantity.
struct Stat {
    uint16_t n;
    float mean;
    float m2;
    int32_t lo;
    int32_t hi;
};
enum StatIndex : uint8_t {
    STAT_IR0 = 0, // ..STAT_IR0 + 7
    STAT_US_F = IR_CHANNEL_COUNT,
    STAT_US_L,
    STAT_US_R,
    STAT_ACC_FWD,
    STAT_INCLINE,
    STAT_GYRO_YAW,
    STAT_COUNT
};
Stat g_stat[STAT_COUNT];
uint16_t g_measureMs = MEASURE_DEFAULT_MS;
uint16_t g_measureTicks = 0;
float g_measureHeading0 = 0.0f;
char g_label[LABEL_MAX + 1];

// `k` / `f` / `save`
LineCalibration g_capture;
bool g_captureDone = false; // g_capture holds a finished capture
bool g_calInUse = false;    // the perceiver has a calibration
bool g_calFromCapture = false;

// `o`
int16_t g_openL = 0;
int16_t g_openR = 0;
uint16_t g_openMs = 0;
int32_t g_openStartL = 0;
int32_t g_openStartR = 0;
int32_t g_openHalfL = 0;
int32_t g_openHalfR = 0;
bool g_openHalfTaken = false;
int32_t g_openLogL = 0;
int32_t g_openLogR = 0;
uint16_t g_stillMs = 0;
int32_t g_lastEncL = 0;
int32_t g_lastEncR = 0;

// `s` / `t`
MotionControl g_mc;
Straight g_straight;
Turn g_turn;
float g_moveArg = 0.0f;   // mm or deg, as asked
float g_moveSpeed = 0.0f; // `s` only
float g_moveDist0 = 0.0f;
float g_moveHeading0 = 0.0f;
int16_t g_dutyL = 0;
int16_t g_dutyR = 0;

// ---- Small helpers -----------------------------------------------------------

int16_t clampDuty(long d)
{
    return static_cast<int16_t>(d > DUTY_CAP ? DUTY_CAP : (d < -DUTY_CAP ? -DUTY_CAP : d));
}

int16_t clampDuty16(int16_t d)
{
    return clampDuty(static_cast<long>(d));
}

int32_t encL()
{
    return g_state.encoderCountL - g_encZeroL;
}

int32_t encR()
{
    return g_state.encoderCountR - g_encZeroR;
}

bool usValid(uint8_t bit)
{
    return (g_state.usValid & bit) != 0;
}

bool parseLong(const char *tok, long &out)
{
    if (tok == nullptr) {
        return false;
    }
    char *end = nullptr;
    out = strtol(tok, &end, 10);
    return end != tok && *end == '\0';
}

bool parseFloat(const char *tok, float &out)
{
    if (tok == nullptr) {
        return false;
    }
    char *end = nullptr;
    out = static_cast<float>(strtod(tok, &end));
    return end != tok && *end == '\0';
}

void printCsv(int32_t v)
{
    Serial.print(',');
    Serial.print(v);
}

void printCsvF(float v, uint8_t digits)
{
    Serial.print(',');
    Serial.print(v, digits);
}

// ---- Motors ------------------------------------------------------------------

void drive(int16_t left, int16_t right)
{
    g_dutyL = clampDuty16(left);
    g_dutyR = clampDuty16(right);
    motorSet(Motor::Left, g_dutyL);
    motorSet(Motor::Right, g_dutyR);
}

void motorsStart()
{
    motionControlReset(g_mc);
    drive(0, 0);
    motorsEnable(true);
}

void motorsStop()
{
    drive(0, 0);
    motorsEnable(false);
}

// ---- Sensors -----------------------------------------------------------------

// The same driver calls as main.cpp's readSensors().
void readSensors()
{
    encodersRead(g_state.encoderCountL, g_state.encoderCountR);
    lineSensorsRead(g_state.irRaw);
    headingEstimatorRead(g_state);
    ultrasonicUpdate();
    g_state.distFrontMm = ultrasonicDistanceMm(Ultrasonic::Front);
    g_state.distLeftMm = ultrasonicDistanceMm(Ultrasonic::Left);
    g_state.distRightMm = ultrasonicDistanceMm(Ultrasonic::Right);
    g_state.usValid = ultrasonicValidMask();
}

void printStatusHeader()
{
    Serial.println(F("# status,t_ms,encL,encR,dist_mm,speed_mm_s,heading_deg,yaw_dps,ax,ay,az,gz,"
                     "incline,usF,usL,usR,ir0,ir1,ir2,ir3,ir4,ir5,ir6,ir7,floor,lineErr_mm"));
    Serial.println(F("# us 0 = no echo. floor 0 black 1 white 2 air 3 mixed, empty without a "
                     "calibration. encoders since z"));
}

void printStatus()
{
    Serial.print(F("status,"));
    Serial.print(g_state.timestampMs);
    printCsv(encL());
    printCsv(encR());
    printCsvF(g_per.motion.distMm, 1);
    printCsvF(g_per.motion.speedMmS, 1);
    printCsvF(g_per.motion.headingDeg, 2);
    printCsvF(g_per.motion.yawRateDps, 2);
    for (uint8_t i = 0; i < 3; ++i) {
        printCsv(g_state.imu.accel[i]);
    }
    printCsv(g_state.imu.gyro[IMU_GYRO_AXIS_YAW]);
    printCsv(g_per.inclineCounts);
    printCsv(usValid(1) ? g_state.distFrontMm : 0);
    printCsv(usValid(2) ? g_state.distLeftMm : 0);
    printCsv(usValid(4) ? g_state.distRightMm : 0);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        printCsv(g_state.irRaw[ch]);
    }
    Serial.print(',');
    if (g_calInUse) {
        Serial.print(static_cast<uint8_t>(g_per.floor));
    }
    Serial.print(',');
    if (g_calInUse) {
        Serial.print(g_per.line.errMm, 1);
    }
    Serial.println();
}

// ---- Help and settings -------------------------------------------------------

void printHelp()
{
    Serial.println(F("# Bench firmware. The motors move only on o, s or t; any key stops them."));
    Serial.println(F("# Sensors (motors off):"));
    Serial.println(
        F("#   w               every sensor now, and the slowest tick since the last w"));
    Serial.println(F("#   v [ms]          stream that line every ms (default 100, min 10)"));
    Serial.println(F("#   z               zero the odometer and the encoder counts"));
    Serial.println(F("#   b               re-zero the gyro: hold the robot still for ~2 s"));
    Serial.println(F("#   m [ms] [label]  hold still, measure for ms (default 2000): mean, sd, "
                     "min, max"));
    Serial.println(F("# IR calibration (motors off):"));
    Serial.println(F("#   k               start a capture: slide the array over white AND black"));
    Serial.println(F("#   f               finish it; it is used at once, but not saved"));
    Serial.println(F("#   save            write the finished capture to EEPROM for the mission"));
    Serial.println(F("#   cal             the calibration in use"));
    Serial.println(F("# Motors (wheels off the floor for o, until you trust it):"));
    Serial.println(F("#   o <L> <R> <ms>  open loop: signed duty per wheel, |duty| <= 600, "
                     "ms <= 3000"));
    Serial.println(F("#   s <mm> [mm/s]   closed-loop straight, signed, |mm| <= 2100"));
    Serial.println(F("#   t <deg>         closed-loop turn in place, CCW positive, |deg| <= 360"));
    Serial.println(F("# Other: p settings, h help, x stop"));
}

void printCalibration(const LineCalibration &cal)
{
    Serial.println(F("# cal,ch,white,black,range,usable"));
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        Serial.print(F("cal,"));
        Serial.print(ch);
        printCsv(cal.white[ch]);
        printCsv(cal.black[ch]);
        printCsv(cal.black[ch] > cal.white[ch] ? cal.black[ch] - cal.white[ch] : 0);
        printCsv(cal.scale[ch] != 0 ? 1 : 0);
        Serial.println();
    }
    Serial.print(F("# looks real (whites >= "));
    Serial.print(IR_CAL_WHITE_MIN_COUNTS);
    Serial.print(F(", spread >= "));
    Serial.print(IR_CAL_WHITE_SPREAD_COUNTS);
    Serial.print(F(", a channel usable): "));
    Serial.println(lineCalibrationLooksReal(cal) ? F("yes") : F("NO"));
}

void printSettings()
{
    const MotionControlParams &c = g_params.control;
    const PlanParams &pl = g_params.step.plan;
    Serial.print(F("# imu "));
    Serial.println(g_imuPresent ? F("present") : F("MISSING - no heading, s and t refused"));
    Serial.print(F("# line calibration: "));
    if (!g_calInUse) {
        Serial.println(F("none"));
    } else {
        Serial.println(g_calFromCapture ? F("last capture (not saved unless you typed save)")
                                        : F("loaded from EEPROM"));
    }
    Serial.print(F("# limits: duty "));
    Serial.print(DUTY_CAP);
    Serial.print(F(", open "));
    Serial.print(OPEN_MAX_MS);
    Serial.print(F(" ms, move "));
    Serial.print(MOVE_MAX_MS);
    Serial.print(F(" ms, distance "));
    Serial.print(DIST_CAP_MM, 0);
    Serial.print(F(" mm, front cut "));
    Serial.print(FRONT_CUT_MM);
    Serial.print(F(" mm, push-still "));
    Serial.print(PUSH_DUTY);
    Serial.print(F(" duty for "));
    Serial.print(PUSH_STILL_MS);
    Serial.println(F(" ms"));
    Serial.print(F("# moves use spec/tuning/Motion.h: speed "));
    Serial.print(SEARCH_SPEED_MM_S, 0);
    Serial.print(F(" mm/s, accel "));
    Serial.print(pl.accelMmS2, 0);
    Serial.print(F(", turn "));
    Serial.print(pl.turnRateMaxDps, 0);
    Serial.print(F(" deg/s at "));
    Serial.print(pl.turnAccelDps2, 0);
    Serial.print(F(", kp/ki/kd dist "));
    Serial.print(c.kpDist, 2);
    Serial.print('/');
    Serial.print(c.kiDist, 2);
    Serial.print('/');
    Serial.print(c.kdDist, 2);
    Serial.print(F(", turn "));
    Serial.print(c.kpTurn, 2);
    Serial.print('/');
    Serial.print(c.kiTurn, 2);
    Serial.print('/');
    Serial.print(c.kdTurn, 2);
    Serial.println();
    Serial.print(F("# mm per count "));
    Serial.print(MM_PER_ENCODER_COUNT, 4);
    Serial.print(F(", track "));
    Serial.print(TRACK_WIDTH_MM);
    Serial.print(F(" mm, us offsets F/L/R "));
    Serial.print(US_OFFSET_FRONT_MM);
    Serial.print('/');
    Serial.print(US_OFFSET_LEFT_MM);
    Serial.print('/');
    Serial.println(US_OFFSET_RIGHT_MM);
}

// ---- `m`: measure standing still ---------------------------------------------

void statReset(Stat &s)
{
    s.n = 0;
    s.mean = 0.0f;
    s.m2 = 0.0f;
    s.lo = INT32_MAX;
    s.hi = INT32_MIN;
}

void statAdd(Stat &s, int32_t v)
{
    if (s.n == UINT16_MAX) {
        return;
    }
    ++s.n;
    const float x = static_cast<float>(v);
    const float delta = x - s.mean;
    s.mean += delta / static_cast<float>(s.n);
    s.m2 += delta * (x - s.mean);
    s.lo = v < s.lo ? v : s.lo;
    s.hi = v > s.hi ? v : s.hi;
}

void measureBegin(uint16_t ms, const char *label)
{
    for (uint8_t i = 0; i < STAT_COUNT; ++i) {
        statReset(g_stat[i]);
    }
    g_measureMs = ms;
    g_measureTicks = 0;
    g_measureHeading0 = g_per.motion.headingDeg;
    strncpy(g_label, label != nullptr ? label : "-", LABEL_MAX);
    g_label[LABEL_MAX] = '\0';
    g_act = Activity::Measure;
    g_actStartMs = g_state.timestampMs;
    Serial.print(F("# measuring "));
    Serial.print(g_label);
    Serial.print(F(" for "));
    Serial.print(ms);
    Serial.println(F(" ms - keep still"));
}

void measureTick()
{
    ++g_measureTicks;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        statAdd(g_stat[STAT_IR0 + ch], g_state.irRaw[ch]);
    }
    // Every tick that holds an echo. A sensor refreshes every 60 ms, so each reading is counted
    // about six times; that leaves the mean and sd as they are. (Counting only changed values
    // would drop a reading that is steady to the millimetre: a real target, not a dead sensor.)
    const uint16_t us[3] = {g_state.distFrontMm, g_state.distLeftMm, g_state.distRightMm};
    for (uint8_t i = 0; i < 3; ++i) {
        if (usValid(static_cast<uint8_t>(1u << i))) {
            statAdd(g_stat[STAT_US_F + i], us[i]);
        }
    }
    statAdd(g_stat[STAT_ACC_FWD], g_state.imu.accel[IMU_ACCEL_AXIS_FORWARD]);
    statAdd(g_stat[STAT_INCLINE], g_per.inclineCounts);
    statAdd(g_stat[STAT_GYRO_YAW], g_state.imu.gyro[IMU_GYRO_AXIS_YAW]);
}

void printStat(const __FlashStringHelper *name, const Stat &s)
{
    Serial.print(F("stat,"));
    Serial.print(g_label);
    Serial.print(',');
    Serial.print(name);
    printCsv(s.n);
    if (s.n == 0) {
        Serial.println(F(",,,,"));
        return;
    }
    printCsvF(s.mean, 1);
    printCsvF(sqrtf(s.m2 / static_cast<float>(s.n)), 2);
    printCsv(s.lo);
    printCsv(s.hi);
    Serial.println();
}

void measureFinish()
{
    static const char irNames[IR_CHANNEL_COUNT][4] PROGMEM = {"ir0", "ir1", "ir2", "ir3",
                                                              "ir4", "ir5", "ir6", "ir7"};
    Serial.println(F("# stat,label,name,n,mean,sd,min,max"));
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        printStat(reinterpret_cast<const __FlashStringHelper *>(irNames[ch]),
                  g_stat[STAT_IR0 + ch]);
    }
    printStat(F("usF"), g_stat[STAT_US_F]);
    printStat(F("usL"), g_stat[STAT_US_L]);
    printStat(F("usR"), g_stat[STAT_US_R]);
    printStat(F("accFwd"), g_stat[STAT_ACC_FWD]);
    printStat(F("incline"), g_stat[STAT_INCLINE]);
    printStat(F("gyroYaw"), g_stat[STAT_GYRO_YAW]);
    const float drift = headingWrapDeg(g_per.motion.headingDeg - g_measureHeading0);
    Serial.print(F("# "));
    Serial.print(g_measureTicks);
    Serial.print(F(" ticks; us n = ticks with an echo, 0 = none heard; heading moved "));
    Serial.print(drift, 2);
    Serial.print(F(" deg = "));
    Serial.print(drift * 60000.0f / static_cast<float>(g_measureMs), 2);
    Serial.println(F(" deg/min"));
}

// ---- `k` / `f` / `save`: IR calibration -----------------------------------------

void captureLog()
{
    Serial.print(F("# range so far:"));
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        Serial.print(' ');
        Serial.print(g_capture.black[ch] > g_capture.white[ch]
                         ? g_capture.black[ch] - g_capture.white[ch]
                         : 0);
    }
    Serial.println();
}

void captureFinish()
{
    const uint8_t usable = lineCalibrationFinish(g_capture);
    g_captureDone = true;
    g_act = Activity::Idle;
    printCalibration(g_capture);
    if (usable == 0) {
        Serial.println(F("# no channel usable - nothing changed. k to try again"));
        return;
    }
    perceiverSetCalibration(g_pc, g_capture);
    g_calInUse = true;
    g_calFromCapture = true;
    Serial.println(F("# in use now (v shows the line). save to keep it for the mission"));
}

void saveCapture()
{
    if (!g_captureDone) {
        Serial.println(F("# nothing to save: k, slide over white and black, then f"));
        return;
    }
    if (!lineCalibrationLooksReal(g_capture)) {
        Serial.println(F("# refused: this capture does not look real (cal shows why). Not saved"));
        return;
    }
    const uint32_t t0 = micros();
    lineCalSave(g_capture);
    const uint32_t us = micros() - t0;
    LineCalibration back;
    const bool ok = lineCalLoad(back) && memcmp(&back, &g_capture, sizeof back) == 0;
    Serial.print(ok ? F("# saved and read back OK in ") : F("# SAVE FAILED: read back differs, "));
    Serial.print(us);
    Serial.println(F(" us"));
}

// ---- `o`: open loop ----------------------------------------------------------------

void openBegin(int16_t left, int16_t right, uint16_t ms)
{
    g_openL = clampDuty16(left);
    g_openR = clampDuty16(right);
    g_openMs = ms > OPEN_MAX_MS ? OPEN_MAX_MS : ms;
    g_openStartL = g_openLogL = g_lastEncL = encL();
    g_openStartR = g_openLogR = g_lastEncR = encR();
    g_openHalfTaken = false;
    g_stillMs = 0;
    g_moveDist0 = g_per.motion.distMm;
    g_act = Activity::Open;
    g_actStartMs = g_lastLogMs = g_state.timestampMs;
    Serial.print(F("# open loop L "));
    Serial.print(g_openL);
    Serial.print(F(" R "));
    Serial.print(g_openR);
    Serial.print(F(" for "));
    Serial.print(g_openMs);
    Serial.println(F(" ms"));
    Serial.println(F("# open,t_ms,dutyL,dutyR,encL,encR,speedL_mm_s,speedR_mm_s,heading_deg,usF"));
    motorsStart();
    drive(g_openL, g_openR);
}

void openEnd(const __FlashStringHelper *why)
{
    motorsStop();
    g_act = Activity::Idle;
    const uint32_t ms = g_state.timestampMs - g_actStartMs;
    const int32_t dL = encL() - g_openStartL;
    const int32_t dR = encR() - g_openStartR;
    Serial.print(F("# open ended ("));
    Serial.print(why);
    Serial.print(F(") after "));
    Serial.print(ms);
    Serial.print(F(" ms: L "));
    Serial.print(dL);
    Serial.print(F(" counts "));
    Serial.print(static_cast<float>(dL) * MM_PER_ENCODER_COUNT, 1);
    Serial.print(F(" mm, R "));
    Serial.print(dR);
    Serial.print(F(" counts "));
    Serial.print(static_cast<float>(dR) * MM_PER_ENCODER_COUNT, 1);
    Serial.println(F(" mm"));
    if (g_openHalfTaken && ms > g_openMs / 2) {
        // The second half, where a wheel that started has reached its speed.
        const float s = static_cast<float>(ms - g_openMs / 2) / 1000.0f;
        Serial.print(F("# second-half speed L "));
        Serial.print(static_cast<float>(encL() - g_openHalfL) * MM_PER_ENCODER_COUNT / s, 1);
        Serial.print(F(" R "));
        Serial.print(static_cast<float>(encR() - g_openHalfR) * MM_PER_ENCODER_COUNT / s, 1);
        Serial.println(F(" mm/s"));
    }
}

void openTick()
{
    const uint32_t ms = g_state.timestampMs - g_actStartMs;
    if (!g_openHalfTaken && ms >= g_openMs / 2) {
        g_openHalfL = encL();
        g_openHalfR = encR();
        g_openHalfTaken = true;
    }
    if (g_state.timestampMs - g_lastLogMs >= MOVE_LOG_MS) {
        const float s = static_cast<float>(g_state.timestampMs - g_lastLogMs) / 1000.0f;
        Serial.print(F("open,"));
        Serial.print(ms);
        printCsv(g_openL);
        printCsv(g_openR);
        printCsv(encL() - g_openStartL);
        printCsv(encR() - g_openStartR);
        printCsvF(static_cast<float>(encL() - g_openLogL) * MM_PER_ENCODER_COUNT / s, 0);
        printCsvF(static_cast<float>(encR() - g_openLogR) * MM_PER_ENCODER_COUNT / s, 0);
        printCsvF(g_per.motion.headingDeg, 1);
        printCsv(usValid(1) ? g_state.distFrontMm : 0);
        Serial.println();
        g_openLogL = encL();
        g_openLogR = encR();
        g_lastLogMs = g_state.timestampMs;
    }

    // Pushing hard with neither wheel turning: a wall, a jam or a cable.
    const bool moved = encL() != g_lastEncL || encR() != g_lastEncR;
    g_lastEncL = encL();
    g_lastEncR = encR();
    const int16_t peak = max(abs(g_openL), abs(g_openR));
    g_stillMs = (!moved && peak >= PUSH_DUTY && ms >= PUSH_GRACE_MS)
                    ? static_cast<uint16_t>(g_stillMs + CONTROL_LOOP_PERIOD_MS)
                    : 0;

    const float travelled = g_per.motion.distMm - g_moveDist0;
    if (ms >= g_openMs) {
        openEnd(F("time"));
    } else if (g_stillMs >= PUSH_STILL_MS) {
        openEnd(F("STALLED: pushing, not moving"));
    } else if (travelled > DIST_CAP_MM || travelled < -DIST_CAP_MM) {
        openEnd(F("DISTANCE CAP"));
    } else if (g_openL > 0 && g_openR > 0 && usValid(1) && g_state.distFrontMm <= FRONT_CUT_MM &&
               g_per.motion.speedMmS > FRONT_CUT_MIN_MM_S) {
        openEnd(F("FRONT CUT: wall ahead"));
    }
}

// ---- `s` / `t`: closed-loop moves ------------------------------------------------

void moveBegin(Activity kind, float arg, float speed)
{
    g_moveArg = arg;
    g_moveSpeed = speed;
    g_moveDist0 = g_per.motion.distMm;
    g_moveHeading0 = g_per.motion.headingDeg;
    if (kind == Activity::Straight) {
        straightBegin(g_straight, g_per.motion, arg, speed, 0.0f, g_per.motion.headingDeg);
    } else {
        turnBegin(g_turn, g_per.motion, headingWrapDeg(g_per.motion.headingDeg + arg), arg);
    }
    g_act = kind;
    g_actStartMs = g_lastLogMs = g_state.timestampMs;
    Serial.println(F("# move,t_ms,ref,dist_mm,speed_mm_s,heading_deg,yaw_dps,dutyL,dutyR,usF"));
    Serial.println(F("# ref = the profile's mm (s) or deg turned (t)"));
    motorsStart();
}

void moveEnd(const __FlashStringHelper *why)
{
    motorsStop();
    const bool straight = g_act == Activity::Straight;
    g_act = Activity::Idle;
    const uint32_t ms = g_state.timestampMs - g_actStartMs;
    const float travelled = g_per.motion.distMm - g_moveDist0;
    const float turned = headingWrapDeg(g_per.motion.headingDeg - g_moveHeading0);
    // result,kind,asked,speed,ms,travelled_mm,turned_deg,error,why
    Serial.println(F("# result,kind,asked,speed_mm_s,ms,travelled_mm,turned_deg,error,why"));
    Serial.print(F("result,"));
    Serial.print(straight ? F("straight") : F("turn"));
    printCsvF(g_moveArg, 1);
    printCsvF(straight ? g_moveSpeed : 0.0f, 0);
    printCsv(static_cast<int32_t>(ms));
    printCsvF(travelled, 1);
    printCsvF(turned, 2);
    // A straight's error is in mm along the move; a turn's in deg against the target heading.
    printCsvF(straight ? g_moveArg - travelled
                       : headingWrapDeg(g_turn.targetHeadingDeg - g_per.motion.headingDeg),
              2);
    Serial.print(',');
    Serial.println(why);
}

void moveLog(float ref)
{
    if (g_state.timestampMs - g_lastLogMs < MOVE_LOG_MS) {
        return;
    }
    g_lastLogMs = g_state.timestampMs;
    Serial.print(F("move,"));
    Serial.print(g_state.timestampMs - g_actStartMs);
    printCsvF(ref, 1);
    printCsvF(g_per.motion.distMm - g_moveDist0, 1);
    printCsvF(g_per.motion.speedMmS, 0);
    printCsvF(g_per.motion.headingDeg, 2);
    printCsvF(g_per.motion.yawRateDps, 1);
    printCsv(g_dutyL);
    printCsv(g_dutyR);
    printCsv(usValid(1) ? g_state.distFrontMm : 0);
    Serial.println();
}

void moveTick()
{
    const bool straight = g_act == Activity::Straight;
    MotionTarget target;
    MoveStatus st;
    if (straight) {
        st = straightStep(g_straight, g_params.step.plan, g_per.motion, 0.0f,
                          CONTROL_LOOP_PERIOD_MS, target);
    } else {
        st = turnStep(g_turn, g_params.step.plan, g_per.motion, CONTROL_LOOP_PERIOD_MS, target);
    }
    const MotionOutput out =
        motionControlStep(g_mc, g_params.control, target, g_per.motion, CONTROL_LOOP_PERIOD_MS);
    drive(out.left, out.right);
    moveLog(straight ? g_straight.refMm - g_moveDist0 : g_turn.refAngleDeg);

    const uint32_t ms = g_state.timestampMs - g_actStartMs;
    const float travelled = g_per.motion.distMm - g_moveDist0;
    if (st == MoveStatus::Done) {
        moveEnd(F("done"));
    } else if (st == MoveStatus::Stuck) {
        moveEnd(F("STUCK: no progress"));
    } else if (g_mc.stalled) {
        moveEnd(F("STALLED: pushing, not moving"));
    } else if (ms >= MOVE_MAX_MS) {
        moveEnd(F("TIME CAP"));
    } else if (travelled > DIST_CAP_MM + 50.0f || travelled < -(DIST_CAP_MM + 50.0f)) {
        moveEnd(F("DISTANCE CAP"));
    } else if (straight && g_moveArg > 0.0f && usValid(1) && g_state.distFrontMm <= FRONT_CUT_MM &&
               g_per.motion.speedMmS > FRONT_CUT_MIN_MM_S) {
        moveEnd(F("FRONT CUT: wall ahead"));
    }
}

// ---- Commands ------------------------------------------------------------------

// Stop whatever runs. The motors are off afterwards whatever was running.
void stopActivity(const __FlashStringHelper *why)
{
    switch (g_act) {
        case Activity::Open:
            openEnd(why);
            break;
        case Activity::Straight:
        case Activity::Turn:
            moveEnd(why);
            break;
        case Activity::Measure:
            Serial.println(F("# measure stopped early"));
            measureFinish();
            break;
        case Activity::Capture:
            Serial.println(F("# capture abandoned - the calibration in use is unchanged"));
            break;
        case Activity::Stream:
        case Activity::Idle:
            break;
    }
    motorsStop();
    g_act = Activity::Idle;
}

void cmdOpen(char *a1, char *a2, char *a3)
{
    long l = 0;
    long r = 0;
    long ms = 0;
    if (!parseLong(a1, l) || !parseLong(a2, r) || !parseLong(a3, ms) || ms <= 0) {
        Serial.println(F("# usage: o <dutyL> <dutyR> <ms>, e.g. o 300 300 800"));
        return;
    }
    if (l > DUTY_CAP || l < -DUTY_CAP || r > DUTY_CAP || r < -DUTY_CAP) {
        Serial.print(F("# duty capped at +-"));
        Serial.println(DUTY_CAP);
    }
    openBegin(clampDuty(l), clampDuty(r),
              static_cast<uint16_t>(ms > OPEN_MAX_MS ? OPEN_MAX_MS : ms));
}

void cmdStraight(char *a1, char *a2)
{
    float mm = 0.0f;
    float v = SEARCH_SPEED_MM_S;
    if (!parseFloat(a1, mm) || mm == 0.0f || (a2 != nullptr && !parseFloat(a2, v)) || v <= 0.0f) {
        Serial.println(F("# usage: s <mm> [mm/s], e.g. s 500 or s -250 100"));
        return;
    }
    if (mm > DIST_CAP_MM || mm < -DIST_CAP_MM || v > SPEED_CAP_MM_S) {
        Serial.println(F("# refused: |mm| <= 2100 and mm/s <= 600"));
        return;
    }
    moveBegin(Activity::Straight, mm, v);
}

void cmdTurn(char *a1)
{
    float deg = 0.0f;
    if (!parseFloat(a1, deg) || deg == 0.0f) {
        Serial.println(F("# usage: t <deg>, CCW positive, e.g. t 90 or t -180"));
        return;
    }
    if (deg > TURN_CAP_DEG || deg < -TURN_CAP_DEG) {
        Serial.println(F("# refused: |deg| <= 360"));
        return;
    }
    moveBegin(Activity::Turn, deg, 0.0f);
}

void cmdMeasure(char *a1, char *a2)
{
    long ms = MEASURE_DEFAULT_MS;
    const char *label = a1;
    if (parseLong(a1, ms)) {
        label = a2;
    } else {
        ms = MEASURE_DEFAULT_MS;
    }
    if (ms < 100 || ms > MEASURE_MAX_MS) {
        Serial.println(F("# usage: m [ms 100..60000] [label], e.g. m 3000 white_board"));
        return;
    }
    measureBegin(static_cast<uint16_t>(ms), label);
}

void cmdStream(char *a1)
{
    long ms = STREAM_DEFAULT_MS;
    if (a1 != nullptr && !parseLong(a1, ms)) {
        Serial.println(F("# usage: v [ms], e.g. v 10"));
        return;
    }
    g_streamMs = static_cast<uint16_t>(ms < CONTROL_LOOP_PERIOD_MS ? CONTROL_LOOP_PERIOD_MS
                                                                   : (ms > 60000 ? 60000 : ms));
    printStatusHeader();
    g_act = Activity::Stream;
    g_lastLogMs = 0;
}

void runCommand(char *line)
{
    char *cmd = strtok(line, " \t");
    if (cmd == nullptr) {
        return;
    }
    char *a1 = strtok(nullptr, " \t");
    char *a2 = strtok(nullptr, " \t");
    char *a3 = strtok(nullptr, " \t");
    const bool moving = strcmp(cmd, "o") == 0 || strcmp(cmd, "s") == 0 || strcmp(cmd, "t") == 0;

    // During a capture only f and x mean something; the motors stay off.
    if (g_act == Activity::Capture) {
        if (strcmp(cmd, "f") == 0) {
            captureFinish();
        } else if (strcmp(cmd, "x") == 0) {
            stopActivity(F("stopped"));
        } else {
            Serial.println(F("# capturing: f to finish, x to abandon"));
        }
        return;
    }

    if (moving && (cmd[0] == 's' || cmd[0] == 't') && !g_imuPresent) {
        Serial.println(F("# refused: no gyro, so no heading to hold"));
    } else if (strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
    } else if (strcmp(cmd, "p") == 0) {
        printSettings();
    } else if (strcmp(cmd, "w") == 0) {
        printStatusHeader();
        printStatus();
        Serial.print(F("# slowest tick since the last w: "));
        Serial.print(g_tickMaxUs);
        Serial.print(F(" us of "));
        Serial.print(CONTROL_LOOP_PERIOD_MS * 1000u);
        Serial.println(F(" us"));
        g_tickMaxUs = 0;
    } else if (strcmp(cmd, "v") == 0) {
        cmdStream(a1);
    } else if (strcmp(cmd, "z") == 0) {
        g_encZeroL = g_state.encoderCountL;
        g_encZeroR = g_state.encoderCountR;
        odometryReset(g_pc.odo);
        Serial.println(F("# odometer and encoders zeroed"));
    } else if (strcmp(cmd, "b") == 0) {
        Serial.println(F("# hold the robot still..."));
        if (headingEstimatorRezero()) {
            Serial.println(F("# heading is 0 here"));
        }
        g_tickMaxUs = 0; // the capture blocks; it is not a slow tick
    } else if (strcmp(cmd, "m") == 0) {
        cmdMeasure(a1, a2);
    } else if (strcmp(cmd, "k") == 0) {
        lineCalibrationReset(g_capture);
        g_captureDone = false;
        g_act = Activity::Capture;
        g_lastLogMs = g_state.timestampMs;
        Serial.println(F("# capturing: slide every channel over white AND black, then f"));
    } else if (strcmp(cmd, "f") == 0) {
        Serial.println(F("# no capture running: k first"));
    } else if (strcmp(cmd, "save") == 0) {
        saveCapture();
    } else if (strcmp(cmd, "cal") == 0) {
        if (g_calInUse) {
            printCalibration(g_pc.cal);
        } else {
            Serial.println(F("# no calibration in use: k, slide, f"));
        }
    } else if (strcmp(cmd, "o") == 0) {
        cmdOpen(a1, a2, a3);
    } else if (strcmp(cmd, "s") == 0) {
        cmdStraight(a1, a2);
    } else if (strcmp(cmd, "t") == 0) {
        cmdTurn(a1);
    } else if (strcmp(cmd, "x") == 0) {
        Serial.println(F("# nothing running"));
    } else {
        Serial.println(F("# unknown command (h for help)"));
    }
}

// Collect a line. While something runs (except a capture, which takes f and x), any byte
// stops it, and the rest of what was typed is dropped.
void pollSerial(uint32_t nowMs)
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (g_act != Activity::Idle && g_act != Activity::Capture) {
            stopActivity(F("stopped by a key"));
            g_dropUntilMs = nowMs + ABORT_QUIET_MS;
            g_lineLen = 0;
            continue;
        }
        if (static_cast<int32_t>(nowMs - g_dropUntilMs) < 0) {
            g_dropUntilMs = nowMs + ABORT_QUIET_MS;
            continue;
        }
        if (c == '\n' || c == '\r') {
            if (g_lineOverflow) {
                Serial.println(F("# line too long"));
            } else if (g_lineLen > 0) {
                g_line[g_lineLen] = '\0';
                runCommand(g_line);
            }
            g_lineLen = 0;
            g_lineOverflow = false;
        } else if (g_lineLen < LINE_MAX) {
            g_line[g_lineLen++] = static_cast<char>(c);
        } else {
            g_lineOverflow = true;
        }
    }
}

void activityTick()
{
    switch (g_act) {
        case Activity::Stream:
            if (g_state.timestampMs - g_lastLogMs >= g_streamMs) {
                g_lastLogMs = g_state.timestampMs;
                printStatus();
            }
            break;
        case Activity::Measure:
            measureTick();
            if (g_state.timestampMs - g_actStartMs >= g_measureMs) {
                g_act = Activity::Idle;
                measureFinish();
            }
            break;
        case Activity::Capture:
            lineCalibrationAccumulate(g_capture, g_state.irRaw);
            if (g_state.timestampMs - g_lastLogMs >= CAPTURE_LOG_MS) {
                g_lastLogMs = g_state.timestampMs;
                captureLog();
            }
            break;
        case Activity::Open:
            openTick();
            break;
        case Activity::Straight:
        case Activity::Turn:
            moveTick();
            break;
        case Activity::Idle:
            break;
    }
}

} // namespace

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    motorsInit(); // EN stays low (coast) until a move command
    encodersInit();
    lineSensorsInit();
    Serial.println(F("# bench: zeroing the gyro - keep the robot still"));
    g_imuPresent = headingEstimatorInit();
    ultrasonicInit();

    perceiverReset(g_pc);
    LineCalibration cal;
    if (lineCalLoad(cal)) {
        perceiverSetCalibration(g_pc, cal);
        g_calInUse = true;
        Serial.println(lineCalibrationLooksReal(cal)
                           ? F("# line calibration loaded from EEPROM")
                           : F("# line calibration loaded, but it does NOT look real (cal)"));
    } else {
        Serial.println(F("# no line calibration in EEPROM: k, slide, f, save"));
    }
    Serial.println(F("# bench ready - motors off. h for help"));
}

void loop()
{
    static uint32_t nextTickMs = 0;
    const uint32_t nowMs = millis();
    pollSerial(nowMs);
    if (!tickDue(nowMs, nextTickMs, CONTROL_LOOP_PERIOD_MS)) {
        return;
    }
    const uint32_t t0 = micros();
    g_state.timestampMs = nowMs;
    readSensors();
    perceive(g_pc, g_state, g_pp, g_per);
    activityTick();
    const uint32_t us = micros() - t0;
    g_tickMaxUs = us > g_tickMaxUs ? us : g_tickMaxUs;
}
