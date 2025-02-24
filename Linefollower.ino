/*  High-Speed Line Follower using RLS-08
    TB6612FNG Motor Driver
    Optimized PID Controller for Faster Response & Meander Handling
    Added IR sensor to detect obstacles and perform 180-degree turn
*/

#include <Wire.h>
#include <SparkFun_TB6612.h>

#define AIN1 7
#define BIN1 5
#define AIN2 6
#define BIN2 4
#define PWMA 9
#define PWMB 10
#define STBY 8
#define IR_SENSOR 12  // IR sensor connected to digital pin 12

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

const int SensorCount = 8;
int sensorPins[SensorCount] = {A0, A1, A2, A3, A4, A5, 2, 3};
int sensorValues[SensorCount];
int base_speed = 250;
int max_speed = 255;
int L = 0;
int R = 0;
int error = 0;
int adj = 0;

float Kp = 0.2;
float Ki = 0.005;
float Kd = 3.0;

int P, I, D, lastError = 0;
uint16_t position;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C mode for quicker sensor readings
  pinMode(IR_SENSOR, INPUT);
  Serial.println("High-Speed RLS-08 Initialized");
}

void loop() {
  if (detectObstacle()) {
    perform180Turn();
    return;
  }
  
  readRLS08();
  position = calculatePosition();

  if (isFinishLine()) {
    stopMotors();
    while (true); // Stop the loop indefinitely
  }
  PID_control();
}

bool detectObstacle() {
  return digitalRead(IR_SENSOR) == LOW; // Assuming LOW means obstacle detected
}

void perform180Turn() {
  Serial.println("Obstacle detected! Performing 180-degree turn");
  motor1.drive(-200);
  motor2.drive(-200);
  delay(500);
  motor1.drive(200);
  motor2.drive(-200);
  delay(800);
}

void readRLS08() {
  for (int i = 0; i < SensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
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

bool isIntersection() {
  int blackCount = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 50) { // Adjust threshold for black detection
      blackCount++;
    }
  }
  return blackCount >= SensorCount; // All sensors detect black
}

void PID_control() {
  if (isIntersection()) {
    forward(base_speed + 20, base_speed + 20); // Increase speed slightly at intersections
    return;
  }
  
  error = 3500 - position;
  P = error;
  I = constrain(I + error, -1000, 1000); // Prevent integral windup
  D = error - lastError;
  lastError = error;

  adj = P * Kp + I * Ki + D * Kd;
  
  int speed_factor = (abs(error) > 1800) ? 200 : max_speed; // Allow faster turns
  L = constrain(base_speed + adj, 0, speed_factor);
  R = constrain(base_speed - adj, 0, speed_factor);
  
  L = L * 0.9 + (base_speed + adj) * 0.1; // More aggressive acceleration
  R = R * 0.9 + (base_speed - adj) * 0.1;
  
  if (error > 2000) {
    sharp_right();
  } else if (error < -2000) {
    sharp_left();
  } else {
    forward(L, R);
  }
}

void forward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}

void sharp_right() {
  motor1.drive(-200);
  motor2.drive(200);
}

void sharp_left() {
  motor1.drive(200);
  motor2.drive(-200);
}

void stopMotors() {
  motor1.drive(0);
  motor2.drive(0);
}
