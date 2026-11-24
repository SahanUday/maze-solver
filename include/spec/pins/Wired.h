// ============================================================================
//  spec/pins/Wired.h  --  which operator controls are wired. TEMPORARY (#64): once the button is
//  wired, set the flag, check it on the bench, then delete this file and the no-button start.
// ============================================================================

#pragma once

// false: the run begins by itself, RUN_START_DELAY_MS after the loop starts.
// true:  the run waits for a debounced press of PIN_BTN_START.
// A button wired while this is false is ignored, and the robot starts by itself.
constexpr bool BUTTON_WIRED = false;
