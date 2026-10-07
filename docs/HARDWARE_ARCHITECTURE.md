# Micromouse Hardware Architecture

## 🎯 Overall Objective
Building a high-performance micromouse using an Arduino Mega. The primary focus of this hardware design is extreme electrical stability, noise immunity, and reliable sensor integration. This document outlines the physical hardware design required before any code is written.

## ⚡ Power Distribution (Isolated Topology)
To prevent the motors from crashing the logic board, power is split into two isolated buses:
*   **Power Source:** 3S (12.6V) LiPo Battery.
*   **The "Dirty" Motor Bus:** 12V routed strictly from the battery directly to the two IBT-2 (BTS7960) motor drivers.
*   **The "Clean" Logic Bus:** 12V is stepped down to a perfectly flat 5.0V using an LM2596 Buck Converter. This clean 5V powers the Arduino Mega, the 8-channel IR array, the MPU-6050 gyroscope, and all ultrasonic sensors.

## 🌍 Grounding Strategy (Star Ground)
A strict **Star Ground** is implemented at the physical battery connector. 
*   The heavy motor ground wire and the delicate logic ground wire never touch each other until they reach the battery splice. 
*   This prevents "ground bounce" where heavy motor current artificially raises the logic voltage, which would otherwise corrupt sensor readings and crash the Arduino.

## 🛡️ Noise Suppression & Safety
To ensure the code runs flawlessly, the physical wiring includes multiple layers of noise filtering:
1.  **Back-EMF Protection:** 35V / 1000µF electrolytic capacitors are mounted directly on the IBT-2 power inputs to absorb 20V+ voltage spikes when the motors suddenly brake.
2.  **Sensor Decoupling:** Local 10µF to 100µF capacitors are placed directly across the VCC/GND pins of the IR array and ultrasonic sensors to supply instant current during ping/flash bursts.
3.  **Logic Pull-Downs:** 10kΩ resistors are installed on the IBT-2 PWM logic lines to lock the motors at 0 speed while the Arduino Mega boots up.
4.  **Signal Debouncing:** 4.7kΩ pull-up resistors and 0.1µF ceramic capacitors are used on the motor encoder lines to ensure clean hardware interrupts.
5.  **Battery Monitoring:** A 20kΩ/10kΩ voltage divider steps the 12.6V battery down to a safe 4.2V for the Arduino's analog pin, allowing the code to trigger an emergency stop if the battery drops below 10.0V.