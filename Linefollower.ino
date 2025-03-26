// Motor driver pins
#define AIN1 5
#define AIN2 4
#define PWMA 3
#define BIN1 7
#define BIN2 8
#define PWMB 9
#define STBY 6

// RLS-08 sensor pins
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Optimized PID constants (Fast response, no overshoot)
float Kp = 16;   // More aggressive to take fast turns
float Ki = 0.004;  // Small integral component for slight correction
float Kd = 12.5;    // Strong damping to prevent overshoot

// PID variables
float error = 0, previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Motor speed (Constant speed)
const int baseSpeed = 155;
const int maxSpeed = 255;

void setup() {
    Serial.begin(115200);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH); // Enable motor driver
}

// Read sensor and calculate position error
int getError() {
    int sensorValues[8];
    int weight[] = {-4, -3, -2, -1, 1, 2, 3, 4};  // Increased weights for sharper response
    int sum = 0, total = 0;
    bool lineDetected = false;

    for (int i = 0; i < 8; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);

        if (sensorValues[i] > 500) {  // Detect black line
            sum += sensorValues[i] * weight[i];
            total += sensorValues[i];
            lineDetected = true;
        }
    }

    if (!lineDetected) {
        return (previousError < 0) ? -6 : 6;  // Stronger lost-line correction
    }
    
    return (total > 0) ? sum / total : 0;
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);
    
    if (leftSpeed > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, leftSpeed);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, 0);
    }
    
    if (rightSpeed > 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, rightSpeed);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, 0);
    }
}

void loop() {
    error = getError();
    integral = constrain(integral + error, -40, 40);  // Reduced integral windup
    derivative = error - previousError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    setMotorSpeed(leftSpeed, rightSpeed);
}
