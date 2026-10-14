// ============================================================================
//  MotionTypes.h  --  the hand-off between a motion primitive and the controller.
//  Plain data, no logic.
// ============================================================================

#pragma once

// What a primitive wants this tick: where the robot should be and how fast it should be going.
// Distance is on the odometer's scale; heading is the gyro's (deg, CCW positive, [-180, 180)).
struct MotionTarget {
    float distRefMm = 0.0f;
    float speedRefMmS = 0.0f;
    float headingRefDeg = 0.0f;
    float yawRateRefDps = 0.0f;
    bool pivot = false; // turning in place: the heading integral is kiTurn, else kiHold
};

// What the robot is actually doing, from Perception.
struct MotionFeedback {
    float distMm = 0.0f;
    float speedMmS = 0.0f;
    float headingDeg = 0.0f;
    float yawRateDps = 0.0f;
};
