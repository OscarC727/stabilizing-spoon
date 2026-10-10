# Firmware

ESP32 code for the filter and PID loops.

| Folder | What it is |
|---|---|
| `gimbal_pid/` | Current version. 200 Hz fixed loop, gyro bias calibration, derivative on measurement, anti-windup, microsecond servo writes. |
| `archive/pid/` | Second version. Switched timing to `micros()` and added plotter output. |
| `archive/PIDSPOOn/` | First working version. Basic PID, `millis()` timing, and a complementary filter that trusts the gyro more when the accelerometer reading is off from 1 g. |

Each folder is its own Arduino sketch, so open the `.ino` inside it directly.

Libraries: **ESP32Servo** and **MPU6050** (Electronic Cats / i2cdevlib).
