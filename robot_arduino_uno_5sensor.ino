/*
 * ============================================================
 *  LINE FOLLOWING ROBOT - ARDUINO UNO (TESTING VERSION)
 * ============================================================
 *  Hardware:
 *    - Arduino Uno
 *    - 5x IR Sensors (Analog Output)
 *    - TCS230 Color Sensor
 *    - HC-SR04 Ultrasonic Sensor
 *    - L298N Motor Driver
 *    - SG90 Servo Gripper (SNM200 Claw - 1 servo used)
 *
 *  Arena Tasks:
 *    1. Follow yellow line from Start to End
 *    2. Detect & avoid 2 red obstacles
 *    3. Detect green cube → pick it up
 *    4. Reach End zone → drop/place green cube
 *
 *  Author  : EC6090 Mini Project 2026
 *  Board   : Arduino Uno (migrate to ESP32 later)
 * ============================================================
 */

#include <Servo.h>
#include <Wire.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// --- IR Sensors (Analog) ---
// Layout: S1(Far Left) ... S5(Far Right)
#define IR_S1   A0   // Far Left
#define IR_S2   A1   // Mid Left
#define IR_S3   A2   // Center
#define IR_S4   A3   // Mid Right
#define IR_S5   A6   // Far Right  (A6/A7 = analog only on Uno)

// --- Ultrasonic Sensor (HC-SR04) ---
#define TRIG_PIN  7
#define ECHO_PIN  8

// --- L298N Motor Driver ---
#define ENA     9    // PWM - Left motor speed
#define IN1     2    // Left motor direction
#define IN2     3    // Left motor direction
#define IN3     4    // Right motor direction
#define IN4     5    // Right motor direction
#define ENB     6    // PWM - Right motor speed

// --- Servo Gripper (SG90) ---
#define SERVO_PIN  10

// --- TCS230 Color Sensor ---
#define TCS_S0   A7   // Note: A7 analog only - we use as output workaround
// Better: use digital pins for TCS230
// Remapped below using actual digital pins:
#define TCS_S0_PIN  11
#define TCS_S1_PIN  12
#define TCS_S2_PIN  13
#define TCS_S3_PIN  A4   // used as digital
#define TCS_OUT_PIN A5   // used as digital input (frequency out)

// ============================================================
//  CONSTANTS & THRESHOLDS
// ============================================================

// IR sensor threshold - tune based on your surface
// Analog value: higher = darker surface (on yellow line)
// You MUST calibrate these on your actual arena floor!
#define IR_THRESHOLD     500   // values above = ON LINE (yellow)

// Ultrasonic obstacle distance threshold (cm)
#define OBSTACLE_DIST    15    // stop if object closer than 15cm

// Motor speeds (0-255)
#define BASE_SPEED       160   // normal line following speed
#define TURN_SPEED       130   // speed during corrections
#define SHARP_TURN_SPEED 100   // speed during sharp turns
#define AVOID_SPEED      140   // speed during obstacle avoidance

// PD Controller gains - TUNE THESE on actual arena
float Kp = 25.0;
float Kd = 12.0;

// Servo angles
#define SERVO_OPEN    90    // Claw open (release cube)
#define SERVO_CLOSED  10    // Claw closed (grip cube)

// Color sensor timing
#define COLOR_SAMPLE_TIME  100  // ms to sample color

// ============================================================
//  GLOBAL VARIABLES
// ============================================================

Servo gripperServo;

// IR sensor readings
int irRaw[5];
int irBinary[5];   // 1 = on line, 0 = off line

// PD control
int lastError = 0;

// Robot state machine
enum RobotState {
  STATE_LINE_FOLLOW,
  STATE_OBSTACLE_DETECTED,
  STATE_AVOID_LEFT,
  STATE_AVOID_RIGHT,
  STATE_REJOIN_LINE,
  STATE_PICK_CUBE,
  STATE_CARRYING,
  STATE_DROP_CUBE,
  STATE_DONE
};

RobotState currentState = STATE_LINE_FOLLOW;

// Cube tracking
bool cubePickedUp = false;
int obstaclesAvoided = 0;

// Color sensor variables
int redFreq   = 0;
int greenFreq = 0;
int blueFreq  = 0;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Robot Initializing...");

  // IR sensor pins (analog - no pinMode needed for INPUT)

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Servo
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(SERVO_CLOSED);  // Start with claw closed
  delay(500);

  // TCS230 Color Sensor
  pinMode(TCS_S0_PIN, OUTPUT);
  pinMode(TCS_S1_PIN, OUTPUT);
  pinMode(TCS_S2_PIN, OUTPUT);
  pinMode(TCS_S3_PIN, OUTPUT);
  pinMode(TCS_OUT_PIN, INPUT);

  // Set TCS230 frequency scaling to 20%
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // Stop motors initially
  stopMotors();

  Serial.println("Robot Ready! Starting in 3 seconds...");
  delay(3000);
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readIRSensors();
  float distance = readUltrasonic();

  switch (currentState) {

    case STATE_LINE_FOLLOW:
      // Check for obstacle first
      if (distance < OBSTACLE_DIST && distance > 0) {
        Serial.println(">> Obstacle detected!");
        stopMotors();
        delay(300);
        currentState = STATE_OBSTACLE_DETECTED;
        break;
      }
      // Check for green cube (only if not yet picked)
      if (!cubePickedUp) {
        if (detectColor() == 'G') {
          Serial.println(">> Green cube detected! Picking up...");
          stopMotors();
          delay(200);
          currentState = STATE_PICK_CUBE;
          break;
        }
      }
      // Normal line following with PD control
      followLine();
      break;

    case STATE_OBSTACLE_DETECTED:
      // Check color - if RED, it's an obstacle to avoid
      if (detectColor() == 'R' || obstaclesAvoided < 2) {
        Serial.println(">> Red obstacle - avoiding LEFT");
        currentState = STATE_AVOID_LEFT;
      } else {
        // Unknown object, try to go around
        currentState = STATE_AVOID_LEFT;
      }
      break;

    case STATE_AVOID_LEFT:
      avoidObstacleLeft();
      obstaclesAvoided++;
      currentState = STATE_REJOIN_LINE;
      break;

    case STATE_REJOIN_LINE:
      rejoinLine();
      currentState = STATE_LINE_FOLLOW;
      break;

    case STATE_PICK_CUBE:
      pickUpCube();
      cubePickedUp = true;
      currentState = STATE_CARRYING;
      break;

    case STATE_CARRYING:
      // Continue line following while carrying cube
      if (distance < OBSTACLE_DIST && distance > 0) {
        // Reached end zone marker or obstacle
        // Check if all sensors lose line = end zone
        int lineSum = 0;
        for (int i = 0; i < 5; i++) lineSum += irBinary[i];
        if (lineSum == 0) {
          // All sensors off line = END ZONE
          currentState = STATE_DROP_CUBE;
        }
      }
      // Check end zone - if line ends (all sensors off)
      int lineSum2 = 0;
      for (int i = 0; i < 5; i++) lineSum2 += irBinary[i];
      if (lineSum2 == 0 && cubePickedUp) {
        Serial.println(">> End zone reached! Dropping cube...");
        stopMotors();
        delay(300);
        currentState = STATE_DROP_CUBE;
      } else {
        followLine();
      }
      break;

    case STATE_DROP_CUBE:
      dropCube();
      currentState = STATE_DONE;
      break;

    case STATE_DONE:
      stopMotors();
      Serial.println(">> TASK COMPLETE!");
      // Blink or signal completion
      delay(1000);
      break;
  }
}

// ============================================================
//  IR SENSOR READING
// ============================================================
void readIRSensors() {
  irRaw[0] = analogRead(IR_S1);
  irRaw[1] = analogRead(IR_S2);
  irRaw[2] = analogRead(IR_S3);
  irRaw[3] = analogRead(IR_S4);
  irRaw[4] = analogRead(A6);   // IR_S5

  for (int i = 0; i < 5; i++) {
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // Debug - uncomment to tune threshold
  /*
  Serial.print("IR: ");
  for(int i=0;i<5;i++){
    Serial.print(irBinary[i]);
    Serial.print(" ");
  }
  Serial.println();
  */
}

// ============================================================
//  LINE FOLLOWING (PD Controller)
// ============================================================
void followLine() {
  // Weighted error calculation
  int weights[5] = {-2, -1, 0, 1, 2};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  int error = 0;
  if (sensorSum != 0) {
    error = weightedSum; // range: -2 to +2 roughly
  } else {
    // All sensors off line - use last known error direction
    error = (lastError > 0) ? 3 : -3;
  }

  // PD correction
  float correction = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  // Clamp to valid range
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Reduce speed on sharp turns
  if (abs(error) >= 2) {
    leftSpeed  = (leftSpeed  > 0) ? min(leftSpeed,  SHARP_TURN_SPEED) : max(leftSpeed,  -SHARP_TURN_SPEED);
    rightSpeed = (rightSpeed > 0) ? min(rightSpeed, SHARP_TURN_SPEED) : max(rightSpeed, -SHARP_TURN_SPEED);
  }

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  OBSTACLE AVOIDANCE
// ============================================================
void avoidObstacleLeft() {
  Serial.println("Avoiding: Turn Left");

  // Step 1: Turn left away from obstacle
  driveMotors(-AVOID_SPEED, AVOID_SPEED);
  delay(400);

  // Step 2: Move forward to pass obstacle
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(600);

  // Step 3: Turn right to realign
  driveMotors(AVOID_SPEED, -AVOID_SPEED);
  delay(400);

  // Step 4: Move forward a bit
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(400);

  stopMotors();
  delay(200);
}

// ============================================================
//  REJOIN LINE AFTER AVOIDANCE
// ============================================================
void rejoinLine() {
  Serial.println("Rejoining line...");
  int timeout = 3000; // max 3 seconds to find line
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    readIRSensors();
    int lineSum = 0;
    for (int i = 0; i < 5; i++) lineSum += irBinary[i];

    if (lineSum > 0) {
      Serial.println("Line found!");
      return;
    }
    // Slowly move forward looking for line
    driveMotors(120, 120);
    delay(50);
  }
  // If line not found, stop
  stopMotors();
  Serial.println("WARNING: Line not found after avoidance!");
}

// ============================================================
//  COLOR SENSOR - TCS230
// ============================================================
char detectColor() {
  // Read Red
  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, LOW);
  redFreq = pulseIn(TCS_OUT_PIN, LOW, 50000);

  delay(20);

  // Read Green
  digitalWrite(TCS_S2_PIN, HIGH);
  digitalWrite(TCS_S3_PIN, HIGH);
  greenFreq = pulseIn(TCS_OUT_PIN, LOW, 50000);

  delay(20);

  // Read Blue
  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, HIGH);
  blueFreq = pulseIn(TCS_OUT_PIN, LOW, 50000);

  delay(20);

  // Debug
  Serial.print("R:"); Serial.print(redFreq);
  Serial.print(" G:"); Serial.print(greenFreq);
  Serial.print(" B:"); Serial.println(blueFreq);

  // ---- Color Detection Logic ----
  // NOTE: You MUST calibrate these thresholds on your actual
  // colored cubes under your arena lighting conditions!
  // Lower frequency = stronger that color
  // These are example starting values - TUNE THEM!

  if (redFreq < greenFreq && redFreq < blueFreq && redFreq < 80) {
    Serial.println("Detected: RED");
    return 'R';
  }
  if (greenFreq < redFreq && greenFreq < blueFreq && greenFreq < 80) {
    Serial.println("Detected: GREEN");
    return 'G';
  }
  if (blueFreq < redFreq && blueFreq < greenFreq && blueFreq < 80) {
    Serial.println("Detected: BLUE");
    return 'B';
  }

  return 'U'; // Unknown
}

// ============================================================
//  PICK UP CUBE (SG90 Gripper)
// ============================================================
void pickUpCube() {
  Serial.println("Opening claw...");
  gripperServo.write(SERVO_OPEN);    // Open claw
  delay(600);

  // Move forward slightly to position over cube
  driveMotors(100, 100);
  delay(300);
  stopMotors();
  delay(200);

  Serial.println("Closing claw (gripping)...");
  gripperServo.write(SERVO_CLOSED);  // Close claw to grip
  delay(800);

  Serial.println("Cube picked up!");
}

// ============================================================
//  DROP / PLACE CUBE
// ============================================================
void dropCube() {
  Serial.println("Dropping cube at end zone...");

  // Move forward a little into drop zone
  driveMotors(100, 100);
  delay(300);
  stopMotors();
  delay(200);

  gripperServo.write(SERVO_OPEN);    // Open claw = release cube
  delay(800);

  Serial.println("Cube placed successfully!");
  cubePickedUp = false;
}

// ============================================================
//  ULTRASONIC SENSOR
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  float distance = duration * 0.034 / 2.0;

  return distance; // in cm
}

// ============================================================
//  MOTOR CONTROL
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // LEFT MOTOR
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }

  // RIGHT MOTOR
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ============================================================
//  END OF CODE
// ============================================================
