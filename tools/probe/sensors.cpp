// Bench probe: motors stay off. Streams every sensor as one CSV line at 10 Hz so the
// numbers can be read while the robot is moved by hand. Not part of the mission build.
#include <Arduino.h>

#include "RobotSpec.h"
#include "RobotState.h"
#include <math.h>
#include "hal/encoders.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    motorsInit(); // EN low: coast
    encodersInit();
    lineSensorsInit();
    const bool imuOk = imuInit();
    ultrasonicInit();
    Serial.println(F("# t_ms,encL,encR,ir0..ir7,usF,usL,usR,usValid,ax,ay,az,gz,tiltDeg"));
    Serial.println(imuOk ? F("# imu ok") : F("# imu MISSING"));
}

void loop()
{
    // The ultrasonic driver counts calls: one per CONTROL_LOOP_PERIOD_MS, like the real loop.
    static uint32_t nextUsMs = 0;
    static uint32_t nextMs = 0;
    const uint32_t now = millis();
    if (now >= nextUsMs) {
        nextUsMs += CONTROL_LOOP_PERIOD_MS;
        ultrasonicUpdate();
    }
    if (now < nextMs) {
        return;
    }
    nextMs = now + 100;

    int32_t l = 0, r = 0;
    encodersRead(l, r);
    uint16_t ir[IR_CHANNEL_COUNT];
    lineSensorsRead(ir);
    ImuRaw imu;
    const bool imuOk = imuRead(imu);

    Serial.print(now);
    Serial.print(',');
    Serial.print(l);
    Serial.print(',');
    Serial.print(r);
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        Serial.print(',');
        Serial.print(ir[i]);
    }
    Serial.print(',');
    Serial.print(ultrasonicDistanceMm(Ultrasonic::Front));
    Serial.print(',');
    Serial.print(ultrasonicDistanceMm(Ultrasonic::Left));
    Serial.print(',');
    Serial.print(ultrasonicDistanceMm(Ultrasonic::Right));
    Serial.print(',');
    Serial.print(ultrasonicValidMask());
    Serial.print(',');
    Serial.print(imuOk ? imu.accel[0] : 0);
    Serial.print(',');
    Serial.print(imuOk ? imu.accel[1] : 0);
    Serial.print(',');
    Serial.print(imuOk ? imu.accel[2] : 0);
    Serial.print(',');
    Serial.print(imuOk ? imu.gyro[IMU_GYRO_AXIS_YAW] : 0);
    Serial.print(',');
    Serial.println(imuOk ? degrees(atan2f(sqrtf((float)imu.accel[0] * imu.accel[0] +
                                                (float)imu.accel[1] * imu.accel[1]),
                                          (float)imu.accel[2]))
                         : 0.0f);
}
