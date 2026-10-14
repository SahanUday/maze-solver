// ============================================================================
//  line_cal_store.h  --  the IR line calibration, kept in EEPROM between power-ups.
//
//  Lighting on the day is not the lighting in the lab, and the robot cannot be
//  reprogrammed at the arena, so the calibration is taken there (DIP_LINE_CAL) and
//  read back on every start. EEPROM is non-critical glue: the Arduino library is fine.
// ============================================================================

#pragma once

#include "LineSense.h"

// False if nothing valid is stored: never written, damaged, or no channel usable.
bool lineCalLoad(LineCalibration &cal);

void lineCalSave(const LineCalibration &cal);
