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
float rollkp = 3.0;
float rollki = 0.5;
float rollkd = 1.0;

//pitch PID
float pitchkp = 3.0;
float pitchki = 0.5;
float pitchkd = 1.0;

//timings for integrals and derivatives
float rollintegral = 0;
float rolllasterror = 0;

float pitchintegral = 0;
float pitchlasterror = 0;

MPU6050 mpu;

unsigned long lastTime = 0;

float CalcPID ( float kp, float ki, float kd, float &integral, float &lasterror, float error, float dt) {
  integral = integral + (error * dt);
  integral = constrain(integral, -10, 10);   
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
}

void loop() {

//Record Derivative
  unsigned long now = millis();
  float dt = ( now - lastTime ) / 1000.0 ;
  lastTime = now;
  if (dt <= 0) dt = 0.0001;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("ax: "); Serial.print(ax);
  Serial.print(" ay: "); Serial.print(ay);
  Serial.print(" az: "); Serial.print(az);
  Serial.print(" gx: "); Serial.print(gx);
  Serial.print(" gy: "); Serial.print(gy);
  Serial.print(" gz: "); Serial.println(gz);

  float accelRoll = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;

  float gyroRollRate = gx / 131.0;
  float gyroPitchRate = gy / 131.0;

  float totalAccel = sqrt((float)ax*ax + (float)ay*ay + (float)az*az) / 16384.0;

  const float ALPHA = 0.98;
  float dynamicAlpha = ALPHA;
  if (totalAccel < 0.7 || totalAccel > 1.3) {
  dynamicAlpha = 0.995; 
}

rollangle = dynamicAlpha * (rollangle + gyroRollRate * dt) + (1 - dynamicAlpha) * accelRoll;
pitchangle = dynamicAlpha * (pitchangle + gyroPitchRate * dt) + (1 - dynamicAlpha) * accelPitch;

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

}
