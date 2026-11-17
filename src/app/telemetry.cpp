// ============================================================================
//  telemetry.cpp  --  see telemetry.h.
// ============================================================================

#include "app/telemetry.h"

#include <Arduino.h>

#include "spec/LineSensor.h"
#include "spec/Mission.h"

namespace
{

// One status line. Columns are named by the `# status` header printed at the start:
//   t_ms phase fault moves turns flags(1 marker, 2 gap, 4 exiting) dist_mm speed_mm_s heading_deg
//   us_front us_left us_right(mm, 0 = no echo) floor(0 black 1 white 2 air 3 mixed) line_err_mm
//   ir0..ir7 duty_left duty_right
void logStatus(const RunLoop &run, const RobotState &state, const DriveCommand &cmd)
{
    if (RUN_LOG_EVERY_MS == 0) {
        return;
    }
    static uint32_t lastMs = 0;
    static bool headerPrinted = false;
    const uint32_t nowMs = millis();
    if (!headerPrinted) {
        headerPrinted = true;
        Serial.println(F("# status t_ms,phase,fault,moves,turns,flags,dist,speed,heading,"
                         "usF,usL,usR,floor,lineErr,ir0..ir7,dutyL,dutyR"));
    }
    if (nowMs - lastMs < RUN_LOG_EVERY_MS) {
        return;
    }
    lastMs = nowMs;
    const SectionA &m = run.mission;
    const Perception &p = run.perception;
    const uint8_t flags = static_cast<uint8_t>(
        (m.marker.atMarker ? 1 : 0) | (m.marker.gapSeen ? 2 : 0) | (m.marker.sideExit ? 4 : 0));
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(static_cast<uint8_t>(m.phase));
    Serial.print(',');
    Serial.print(static_cast<uint8_t>(m.fault));
    Serial.print(',');
    Serial.print(m.explore.moves);
    Serial.print(',');
    Serial.print(m.explore.turns);
    Serial.print(',');
    Serial.print(flags);
    Serial.print(',');
    Serial.print(static_cast<int>(p.motion.distMm));
    Serial.print(',');
    Serial.print(static_cast<int>(p.motion.speedMmS));
    Serial.print(',');
    Serial.print(static_cast<int>(p.motion.headingDeg));
    Serial.print(',');
    Serial.print((state.usValid & 1u) != 0 ? state.distFrontMm : 0);
    Serial.print(',');
    Serial.print((state.usValid & 2u) != 0 ? state.distLeftMm : 0);
    Serial.print(',');
    Serial.print((state.usValid & 4u) != 0 ? state.distRightMm : 0);
    Serial.print(',');
    Serial.print(static_cast<uint8_t>(p.floor));
    Serial.print(',');
    Serial.print(static_cast<int>(p.line.errMm));
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        Serial.print(',');
        Serial.print(state.irRaw[ch]);
    }
    Serial.print(',');
    Serial.print(cmd.left);
    Serial.print(',');
    Serial.println(cmd.right);
}

void logPhaseChange(const RunLoop &run)
{
    static SectionAPhase loggedPhase = SectionAPhase::WaitStart;
    const SectionAPhase phase = run.mission.phase;
    if (phase == loggedPhase) {
        return;
    }
    loggedPhase = phase;
    Serial.print(F("run: phase "));
    Serial.print(static_cast<uint8_t>(phase));
    Serial.print(F(" fault "));
    Serial.print(static_cast<uint8_t>(run.mission.fault));
    Serial.print(F(" moves "));
    Serial.print(run.mission.explore.moves);
    Serial.print(F(" turns "));
    Serial.println(run.mission.explore.turns);
}

} // namespace

void telemetryTick(const RunLoop &run, const RobotState &state, const DriveCommand &cmd)
{
    if (run.started) {
        logStatus(run, state, cmd);
    }
    logPhaseChange(run);
}
