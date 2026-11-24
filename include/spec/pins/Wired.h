// ============================================================================
//  spec/pins/Wired.h  --  which operator controls are wired to the robot.
//
//  TEMPORARY (#64): this file exists so the panel code can merge before the button is wired.
//  When the button is wired, set the flag true, check it on the bench, then delete this file and
//  the no-button start it guards.
//
//  Only the button needs a flag. An unwired DIP bank reads all OFF (explore, the same as "not
//  wired") and writing to an unconnected LED pin does nothing, but an unwired button cannot be
//  told from one that is not pressed.
// ============================================================================

#pragma once

// false: the run begins by itself, RUN_START_DELAY_MS after the loop starts (spec/tuning/Run.h).
// true:  the run waits for a debounced press of the start button (PIN_BTN_START).
// Danger: a button that is wired while this is still false is ignored, and the robot starts by
// itself while a person may still be holding it.
constexpr bool BUTTON_WIRED = false;
