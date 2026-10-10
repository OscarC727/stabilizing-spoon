#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>

// ---- Servo objects ----
Servo servoRoll;
Servo servoPitch;

// servo pins
const int ROLL_PIN = 19;
const int PITCH_PIN = 18;

// servo pulse range (ESP32Servo defaults, same as servo.write() uses)
const int SERVO_MIN_US = 544;
const int SERVO_MAX_US = 2400;

//IMU Data starting point
float rollangle = 0;
float pitchangle = 0;

//roll PID
float rollkp = 1.0;
float rollki = 0.0;
float rollkd = 0.1;

//pitch PID
float pitchkp = 1.0;
float pitchki = 0.0;
float pitchkd = 0.1;

// max correction either way (degrees of servo travel)
const float OUT_LIMIT = 80;

//integrals
float rollintegral = 0;
float pitchintegral = 0;

// gyro bias, measured at startup
float gxBias = 0;
float gyBias = 0;

MPU6050 mpu;

// fixed control rate
const unsigned long LOOP_US = 5000;   // 5 ms = 200 Hz

unsigned long lastTime = 0;    // microseconds
unsigned long lastPrint = 0;   // for throttling serial output

// PID with:
//  - derivative on measurement (uses the gyro rate instead of differentiating a noisy error)
//  - anti-windup (integral stops growing while the output is maxed out, and is clamped)
float CalcPID(float kp, float ki, float kd, float &integral, float error, float rate, float dt) {
  float pd = (kp * error) - (kd * rate);   // target is constant, so d(error)/dt = -rate

  float newIntegral = integral + (error * dt);
  float output = pd + (ki * newIntegral);

  // only accept the new integral if it doesn't push the output past its limit
  if (output <= OUT_LIMIT && output >= -OUT_LIMIT) {
    integral = newIntegral;
  }

  // hard clamp so the I term alone can never exceed the output limit
  if (ki > 0) {
    float iMax = OUT_LIMIT / ki;
    integral = constrain(integral, -iMax, iMax);
  }

  output = pd + (ki * integral);
  return constrain(output, -OUT_LIMIT, OUT_LIMIT);
}

// write a float angle to a servo in microseconds (finer than servo.write()'s 1-degree steps)
void writeServoAngle(Servo &s, float angle) {
  angle = constrain(angle, 90 - OUT_LIMIT, 90 + OUT_LIMIT);
  int us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US);
  s.writeMicroseconds(us);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro - keep the board still...");
  const int N = 500;
  long sumX = 0, sumY = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    delay(2);
  }
  gxBias = sumX / (float)N;
  gyBias = sumY / (float)N;
  Serial.print("Gyro bias x: "); Serial.print(gxBias);
  Serial.print("  y: ");         Serial.println(gyBias);
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  servoRoll.attach(ROLL_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoPitch.attach(PITCH_PIN, SERVO_MIN_US, SERVO_MAX_US);

  servoRoll.write(90);
  servoPitch.write(90);

  Wire.begin();
  Wire.setClock(400000);          // faster I2C reads
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU Connection Successful");
  } else {
    Serial.println("MPU Connection Failed");
  }

  // built-in low-pass filter on the MPU6050 (~42 Hz) to cut vibration/servo noise
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  delay(500);          // let servos settle at 90 before calibrating
  calibrateGyro();

  // start the filter from the accelerometer so it doesn't have to "slide" in from 0
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  rollangle  = atan2(ay, az) * 180.0 / PI;
  pitchangle = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;

  // Start the timer here so the first dt isn't huge
  lastTime = micros();
}

void loop() {

  // run the control loop at a fixed rate
  unsigned long now = micros();
  if (now - lastTime < LOOP_US) return;
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelRoll = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;

  float gyroRollRate  = (gx - gxBias) / 131.0;
  float gyroPitchRate = (gy - gyBias) / 131.0;

  const float ALPHA = 0.98;
  rollangle = ALPHA * (rollangle + gyroRollRate * dt) + (1 - ALPHA) * accelRoll;
  pitchangle = ALPHA * (pitchangle + gyroPitchRate * dt) + (1 - ALPHA) * accelPitch;

  float rollError = (0 - rollangle);
  float pitchError = (0 - pitchangle);

  float rollCorrection  = CalcPID(rollkp,  rollki,  rollkd,  rollintegral,  rollError,  gyroRollRate,  dt);
  float pitchCorrection = CalcPID(pitchkp, pitchki, pitchkd, pitchintegral, pitchError, gyroPitchRate, dt);

  // calculate angle correction off starting angle (roll servo is mounted reversed)
  float rollservoangle = 90 - rollCorrection;
  float pitchservoangle = 90 + pitchCorrection;

  //send to servos
  writeServoAngle(servoRoll, rollservoangle);
  writeServoAngle(servoPitch, pitchservoangle);

  // ---- Plotter output (~50 Hz so printing doesn't slow the control loop) ----
  // Format works with both the Arduino Serial Plotter and pid_plotter.py
  if (now - lastPrint >= 20000) {   // 20000 us = 20 ms
    lastPrint = now;
    Serial.print("Target:");      Serial.print(0);
    Serial.print(",Roll:");       Serial.print(rollangle);
    Serial.print(",Pitch:");      Serial.print(pitchangle);
    Serial.print(",RollErr:");    Serial.print(rollError);
    Serial.print(",PitchErr:");   Serial.println(pitchError);
  }
}
