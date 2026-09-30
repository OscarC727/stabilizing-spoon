# Self-Stabilizing Spoon

A spoon that cancels hand tremors. An IMU measures how the hand is moving, and a PID loop running on an ESP32 drives two servos in a 2-axis gimbal to keep the spoon level.

**Status:** working breadboard prototype. A friend and I are designing a custom PCB for it.

![Control loop and power diagram](docs/system-diagram.svg)

## How it works

1. The **MPU6050** reads acceleration and rotation over I²C.
2. The **ESP32** combines those readings into a pitch and roll estimate with a filter, so gyro drift and accelerometer noise don't throw it off.
3. A **PID loop** on each axis compares the spoon's tilt to level and computes a correction.
4. Two **SG90 servos**, mounted in series as a gimbal, move the spoon opposite to the hand's motion.

## Hardware

| Part | What it does |
|---|---|
| ESP32 (second build: Seeed XIAO ESP32) | Runs the filter and both PID loops |
| MPU6050 IMU | Accelerometer + gyroscope, I²C |
| 2x SG90 servos | Pitch and roll axes of the gimbal |
| Buck converter | Steps a 9 V supply down to 5 V |
| 9 V supply | Powers everything, with a common ground |

## What I learned

This is the project where I learned PID and sensor filtering from scratch: how IMUs work, why a raw accelerometer is too noisy to use alone, and the difference between complementary and Kalman filters. It's also where I learned to separate a slow signal from a fast one by frequency, which is the core of telling a hand tremor apart from intentional movement.

## What's next

- [ ] Finish the custom PCB and move off the breadboard
- [ ] Upload the firmware and a wiring diagram with pin numbers
- [ ] Add photos and a demo video
- [ ] Compare against an Arduino Nano build for more predictable loop timing

## Repo layout

```
firmware/   ESP32 code (coming soon)
hardware/   wiring and PCB files (coming soon)
docs/       diagrams
media/      photos and video (coming soon)
```
