/*
 * Line Follower Robot with ESP32
 * PID Control with Adaptive Speed
 * Optimized for tight corners and high performance
 * 
 * Hardware:
 * - ESP32
 * - 2x 6V DC motors
 * - 8-channel reflectance sensor array
 * - L298N motor driver
 * 
 * Author: Felipe Mattia (translated to EN)
 * Year: 2025
 */

// Pin configuration
#define LEFT_MOTOR_IN1 26
#define LEFT_MOTOR_IN2 27
#define LEFT_MOTOR_ENA 14
#define RIGHT_MOTOR_IN1 32
#define RIGHT_MOTOR_IN2 33
#define RIGHT_MOTOR_ENB 25

// Reflectance sensor pins
const int sensorPins[] = {13, 12, 14, 27, 26, 25, 33, 32};
const int NUM_SENSORS = 8;

// PID configuration
#define KP 2.5        // Proportional
#define KI 0.1        // Integral
#define KD 0.8        // Derivative

// Speed configuration
#define BASE_SPEED 200      // Base speed (0-255)
#define MIN_SPEED 80        // Minimum speed in corners
#define MAX_SPEED 255       // Maximum speed
#define CURVE_THRESHOLD 0.6 // Threshold to detect curves

// PID variables
float errorValue = 0;
float previousError = 0;
float integralError = 0;
float derivativeError = 0;
float pidOutput = 0;

// Control variables
int leftSpeed = BASE_SPEED;
int rightSpeed = BASE_SPEED;
float averageError = 0;
bool curveDetected = false;

// PWM configuration
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Calibration variables
int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
bool sensorsCalibrated = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Line Follower Robot - Initializing...");
  
  // Motor pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  
  // PWM setup for speed control
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_MOTOR_ENA, 0);
  ledcAttachPin(RIGHT_MOTOR_ENB, 1);
  
  // Sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Wait for sensor stabilization
  delay(1000);
  Serial.println("System initialized successfully!");
  
  // Ask for calibration
  Serial.println("Do you want to calibrate the sensors? (y/n)");
  delay(2000);
  
  if (Serial.available()) {
    char answer = Serial.read();
    if (answer == 'y' || answer == 'Y') {
      calibrateSensors();
    }
  }
  
  Serial.println("Robot is ready!");
}

void loop() {
  // Check calibration
  if (!sensorsCalibrated) {
    Serial.println("Sensors not calibrated! Press 'c' to calibrate");
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'c' || cmd == 'C') {
        calibrateSensors();
      }
    }
    delay(100);
    return;
  }
  
  // Read sensors with noise filtering
  int sensorValues[NUM_SENSORS];
  readSensors(sensorValues);
  
  // Compute error using weighted average
  computeError(sensorValues);
  
  // Apply PID
  applyPID();
  
  // Adaptive speed control
  applyAdaptiveSpeed();
  
  // Apply motor commands
  applyMotorCommands();
  
  // Debug (optional)
  if (millis() % 100 == 0) {
    debugInfo();
  }
  
  // Small delay for stability
  delay(5);
}

void readSensors(int values[]) {
  // Read with noise filter and averaging
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sum = 0;
    for (int j = 0; j < 3; j++) { // Average of 3 readings
      sum += analogRead(sensorPins[i]);
      delayMicroseconds(50);
    }
    int avg = sum / 3;
    
    // Normalize using calibration
    if (sensorsCalibrated) {
      // Map analog to 0-1000 (0 = white, 1000 = black)
      int normalized = map(avg, minValues[i], maxValues[i], 0, 1000);
      values[i] = constrain(normalized, 0, 1000);
    } else {
      // If not calibrated, use raw value
      values[i] = avg;
    }
  }
}

void computeError(int values[]) {
  // Weighted average error calculation
  float weightedSum = 0;
  float weightSum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    float weight = (i - (NUM_SENSORS - 1) / 2.0) * 2.0; // Weight -7 to +7
    
    // Detect black line (high values = line)
    if (values[i] > 500) { // Threshold 500/1000
      weightedSum += weight;
      weightSum += abs(weight);
    }
  }
  
  if (weightSum > 0) {
    errorValue = errorValue * 0.7 + (weightedSum / weightSum) * 0.3;
  } else {
    // Line not detected - keep previous direction or search
    if (abs(errorValue) > 0.5) {
      errorValue = (errorValue > 0) ? 1.0 : -1.0; // Search towards last error
    }
  }
}

void applyPID() {
  // PID calculation
  integralError += errorValue;
  derivativeError = errorValue - previousError;
  
  // Clamp integral to avoid windup
  if (abs(integralError) > 50) {
    integralError = (integralError > 0) ? 50 : -50;
  }
  
  // PID output
  pidOutput = KP * errorValue + KI * integralError + KD * derivativeError;
  
  // Clamp output
  if (abs(pidOutput) > 255) {
    pidOutput = (pidOutput > 0) ? 255 : -255;
  }
  
  previousError = errorValue;
}

void applyAdaptiveSpeed() {
  // Curve detection based on error
  float absError = abs(errorValue);
  
  if (absError > CURVE_THRESHOLD) {
    curveDetected = true;
    // Progressive speed reduction based on error
    float reductionFactor = map(absError * 100, CURVE_THRESHOLD * 100, 100, 100, MIN_SPEED);
    reductionFactor = constrain(reductionFactor, MIN_SPEED, BASE_SPEED);
    
    leftSpeed = BASE_SPEED - (pidOutput * reductionFactor / 255);
    rightSpeed = BASE_SPEED + (pidOutput * reductionFactor / 255);
  } else {
    if (curveDetected) {
      // Gradual return to normal speed
      leftSpeed = BASE_SPEED - pidOutput;
      rightSpeed = BASE_SPEED + pidOutput;
      
      // Check if stabilized
      if (abs(errorValue) < 0.1) {
        curveDetected = false;
      }
    } else {
      // Normal straight-line operation
      leftSpeed = BASE_SPEED - pidOutput;
      rightSpeed = BASE_SPEED + pidOutput;
    }
  }
  
  // Clamp speeds
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
}

void applyMotorCommands() {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    ledcWrite(0, leftSpeed);
  } else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    ledcWrite(0, abs(leftSpeed));
  }
  
  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    ledcWrite(1, rightSpeed);
  } else {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    ledcWrite(1, abs(rightSpeed));
  }
}

void debugInfo() {
  Serial.print("Err: ");
  Serial.print(errorValue, 3);
  Serial.print(" | PID: ");
  Serial.print(pidOutput, 1);
  Serial.print(" | L_Spd: ");
  Serial.print(leftSpeed);
  Serial.print(" | R_Spd: ");
  Serial.print(rightSpeed);
  Serial.print(" | Curve: ");
  Serial.println(curveDetected ? "YES" : "NO");
}

// Emergency stop
void emergencyStop() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

// Automatic sensor calibration
void calibrateSensors() {
  Serial.println("=== SENSOR AUTO-CALIBRATION ===");
  Serial.println("1. Place the robot on a WHITE surface");
  Serial.println("2. Press 'w' to calibrate WHITE");
  Serial.println("3. Place the robot on a BLACK surface");
  Serial.println("4. Press 'b' to calibrate BLACK");
  Serial.println("5. Press 's' to start the robot");
  
  // Init arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023; // Start high to find min
    maxValues[i] = 0;    // Start low to find max
  }
  
  while (!sensorsCalibrated) {
    if (Serial.available()) {
      char cmd = Serial.read();
      switch (cmd) {
        case 'w':
        case 'W':
          calibrateWhite();
          break;
        case 'b':
        case 'B':
          calibrateBlack();
          break;
        case 's':
        case 'S':
          finishCalibration();
          break;
      }
    }
    
    // Show live sensor values
    showSensorValues();
    delay(100);
  }
}

void calibrateWhite() {
  Serial.println("Calibrating WHITE, move the robot over white for 3 seconds");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < minValues[i]) {
        minValues[i] = val;
      }
    }
    delay(50);
  }
  
  Serial.println("WHITE calibrated!");
  showCalibration();
}

void calibrateBlack() {
  Serial.println("Calibrating BLACK, move the robot over black for 3 seconds");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val > maxValues[i]) {
        maxValues[i] = val;
      }
    }
    delay(50);
  }
  
  Serial.println("BLACK calibrated!");
  showCalibration();
}

void finishCalibration() {
  Serial.println("=== CALIBRATION FINISHED ===");
  showCalibration();
  
  // Validate calibration
  bool valid = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (maxValues[i] - minValues[i] < 100) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" may have insufficient calibration!");
      valid = false;
    }
  }
  
  if (valid) {
    sensorsCalibrated = true;
    Serial.println("Calibration valid! Starting robot");
  } else {
    Serial.println("It is recommended to recalibrate the sensors!");
  }
}

void showCalibration() {
  Serial.println("Calibration values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min=");
    Serial.print(minValues[i]);
    Serial.print(" Max=");
    Serial.print(maxValues[i]);
    Serial.print(" Range=");
    Serial.println(maxValues[i] - minValues[i]);
  }
}

void showSensorValues() {
  Serial.print("Current values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(analogRead(sensorPins[i]));
    Serial.print(" ");
  }
  Serial.println();
}


