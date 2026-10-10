#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>

// ---- Servo objects ----
Servo servoRoll;
Servo servoPitch;

// servo pins
const int ROLL_PIN = 19;
const int PITCH_PIN = 18;

//IMU Data starting point
float rollangle = 0;
float pitchangle = 0;

//roll PID
float rollkp = 4;
float rollki = 1;
float rollkd = 2;

//pitch PID
float pitchkp = 4;
float pitchki = 1;
float pitchkd = 2;

//timings for integrals and derivatives
float rollintegral = 0;
float rolllasterror = 0;

float pitchintegral = 0;
float pitchlasterror = 0;

MPU6050 mpu;

unsigned long lastTime = 0;    // now in microseconds
unsigned long lastPrint = 0;   // for throttling serial output

float CalcPID ( float kp, float ki, float kd, float &integral, float &lasterror, float error, float dt) {
  integral = integral + (error * dt);
  float derivative = (error - lasterror) / dt;
  lasterror = error;

  float output = (kp * error) + (ki * integral) + (kd * derivative);
  return output; 
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  servoRoll.attach(ROLL_PIN);
  servoPitch.attach(PITCH_PIN);

  servoRoll.write(90);
  servoPitch.write(90);

  Wire.begin ();
  mpu.initialize ();

  if (mpu.testConnection ()) {
    Serial.println ("MPU Connection Successful");
  } else {
    Serial.println ("MPU Connection Failed");
  }

  // Start the timer here so the first dt isn't ~1 second
  lastTime = micros();
  
}

void loop() {

//Record Derivative (micros() for a smoother dt than millis())
  unsigned long now = micros();
  float dt = ( now - lastTime ) / 1000000.0 ;
  lastTime = now;
  if (dt <= 0) dt = 0.0001;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelRoll = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;

  float gyroRollRate = gx / 131.0;
  float gyroPitchRate = gy / 131.0;

  const float ALPHA = 0.98;
  rollangle = ALPHA * (rollangle + gyroRollRate * dt) + (1 - ALPHA) * accelRoll;
  pitchangle = ALPHA * (pitchangle + gyroPitchRate * dt) + (1 - ALPHA) * accelPitch;

  float rollError = (0 - rollangle);
  float pitchError = (0 - pitchangle);

  float rollCorrection = CalcPID(rollkp, rollki, rollkd, rollintegral, rolllasterror, rollError, dt);
  float pitchCorrection = CalcPID(pitchkp, pitchki, pitchkd, pitchintegral, pitchlasterror, pitchError, dt);

  // calculate angle correction off starting angle
  float rollservoangle = 90 - rollCorrection;
  float pitchservoangle = 90 + pitchCorrection;

  //constraint incase
  int rollLimited = constrain(rollservoangle, 90 - 40, 90 + 40);
  int pitchLimited = constrain(pitchservoangle, 90 - 40, 90 + 40);

  //send to servos
  servoRoll.write(rollLimited);
  servoPitch.write(pitchLimited);

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
