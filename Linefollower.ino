/*  High-Speed Line Follower using RLS-08
    TB6612FNG Motor Driver
    Optimized PID Controller for Faster Response
*/

#include <Wire.h>
#include <SparkFun_TB6612.h>

#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

const int SensorCount = 8;
int sensorValues[SensorCount];
int base_speed = 200;
int max_speed = 255;
int L = 0;
int R = 0;
int error = 0;
int adj = 0;

float Kp = 0.1;
float Ki = 0.006;
float Kd = 2.0;

int P, I, D, lastError = 0;
uint16_t position;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C mode for quicker sensor readings
  brake(motor1, motor2);
  Serial.println("High-Speed RLS-08 Initialized");
}

void loop() {
  readRLS08();
  if (isFinishLine()) {
    stopMotors();
    while (true); // Stop the loop indefinitely
  }
  PID_control();
}

void readRLS08() {
  Wire.requestFrom(0x20, SensorCount);
  for (int i = 0; i < SensorCount; i++) {
    if (Wire.available()) {
      sensorValues[i] = Wire.read();
    }
  }
  position = calculatePosition();
}

uint16_t calculatePosition() {
  long weightedSum = 0;
  long sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    weightedSum += (i * 1000) * sensorValues[i];
    sum += sensorValues[i];
  }
  return (sum > 0) ? weightedSum / sum : 3500;
}

bool isFinishLine() {
  int whiteCount = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 200) { // Adjust threshold as needed
      whiteCount++;
    }
  }
  return whiteCount >= SensorCount; // All sensors detect white
}

void PID_control() {
  error = 3500 - position;
  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  adj = P * Kp + I * Ki + D * Kd;
  
  int speed_factor = abs(error) > 2000 ? 180 : max_speed; // Slow down on sharp turns
  L = constrain(base_speed + adj, 0, speed_factor);
  R = constrain(base_speed - adj, 0, speed_factor);
  
  L = L * 0.9 + (base_speed + adj) * 0.1; // Smooth acceleration
  R = R * 0.9 + (base_speed - adj) * 0.1;
  
  forward(L, R);
}

void forward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}

void sharp_right() {
  motor1.drive(-255);
  motor2.drive(255);
}

void sharp_left() {
  motor1.drive(255);
  motor2.drive(-255);
}

void stopMotors() {
  motor1.drive(0);
  motor2.drive(0);
}
