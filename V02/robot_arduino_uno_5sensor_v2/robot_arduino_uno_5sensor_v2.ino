/*
 * ============================================================
 *  LINE FOLLOWING ROBOT - ARDUINO UNO (TESTING VERSION v2)
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
 *    2. Ultrasonic detects object ahead → STOP → Read color
 *       → RED cube  : AVOID (go around, rejoin line)
 *       → GREEN cube: PICK UP (grip, carry to end zone)
 *    3. Carry green cube to End zone → DROP
 *
 *  FIX LOG (v2):
 *    - Color is ONLY read when ultrasonic detects object
 *    - RED  → avoid correctly
 *    - GREEN → pick correctly (never avoid green)
 *    - End zone detected by distance marker + timer, not IR loss
 *    - State machine cleaned up and made robust
 *
 *  Author  : EC6090 Mini Project 2026
 *  Board   : Arduino Uno (migrate to ESP32 later)
 * ============================================================
 */

#include <Servo.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// --- IR Sensors (Analog Output) ---
// Layout (front of robot): S1=Far Left ... S5=Far Right
#define IR_S1_PIN   A0
#define IR_S2_PIN   A1
#define IR_S3_PIN   A2
#define IR_S4_PIN   A3
#define IR_S5_PIN   A6   // A6 = analog only on Uno, fine here

// --- Ultrasonic Sensor (HC-SR04) ---
#define TRIG_PIN    7
#define ECHO_PIN    8

// --- L298N Motor Driver ---
#define ENA_PIN     9    // PWM Left motor speed  (must be PWM pin)
#define IN1_PIN     2    // Left motor direction A
#define IN2_PIN     3    // Left motor direction B
#define IN3_PIN     4    // Right motor direction A
#define IN4_PIN     5    // Right motor direction B
#define ENB_PIN     6    // PWM Right motor speed (must be PWM pin)

// --- Servo Gripper ---
#define SERVO_PIN   10

// --- TCS230 Color Sensor ---
// S0, S1 = frequency scaling
// S2, S3 = photodiode filter select
// OUT     = frequency output
#define TCS_S0_PIN  11
#define TCS_S1_PIN  12
#define TCS_S2_PIN  13
#define TCS_S3_PIN  A4   // used as digital output
#define TCS_OUT_PIN A5   // used as digital input (pulse frequency)

// ============================================================
//  TUNABLE CONSTANTS
// ============================================================

// IR Threshold: analog value ABOVE this = sensor ON the yellow line
// CALIBRATE THIS: put sensor on yellow line, read Serial, set value
// between off-line reading and on-line reading
#define IR_THRESHOLD      500

// Ultrasonic: stop and check color if object closer than this (cm)
#define DETECT_DIST       18
// Ultrasonic: considered "no object" if reading above this (cm)
#define CLEAR_DIST        25

// Motor base speeds (0–255 PWM)
#define BASE_SPEED        155   // normal line following
#define SHARP_SPEED       100   // sharp turn inner speed cap
#define AVOID_SPEED       140   // obstacle bypass speed
#define CREEP_SPEED       100   // slow approach for pick up

// PD controller gains — TUNE on actual arena
// Start here, increase Kp if robot is sluggish on curves,
// decrease if it oscillates/wobbles
float Kp = 25.0;
float Kd = 12.0;

// Servo positions
#define SERVO_OPEN        85    // Claw fully open  (release)
#define SERVO_CLOSED      15    // Claw fully closed (grip)

// End zone: robot travels this long (ms) after picking cube
// before we consider it arrived at the end zone
// TUNE based on actual arena track length after green cube
#define END_ZONE_TRAVEL_MS  8000   // 8 seconds - tune this!

// ============================================================
//  GLOBAL STATE
// ============================================================

Servo gripperServo;

// IR readings
int irRaw[5];
int irBinary[5];   // 1 = on yellow line, 0 = off line

// PD
int lastError = 0;

// Tracking
bool  cubePickedUp    = false;
int   obstaclesAvoided = 0;
unsigned long pickUpTime = 0;   // millis() when green cube was picked

// ============================================================
//  ROBOT STATE MACHINE
// ============================================================
// Every state has ONE clear responsibility.
// Color is ONLY read inside STATE_IDENTIFY_OBJECT.

enum RobotState {
  STATE_LINE_FOLLOW,       // Follow yellow line normally
  STATE_IDENTIFY_OBJECT,   // Object detected ahead → read color → decide
  STATE_AVOID_RED,         // Bypass red obstacle, rejoin line
  STATE_REJOIN_LINE,       // Sweep to find yellow line after bypass
  STATE_APPROACH_GREEN,    // Creep forward to position gripper on green cube
  STATE_PICK_GREEN,        // Close gripper, lift
  STATE_CARRY_TO_END,      // Line follow while carrying, watch for end zone
  STATE_DROP_CUBE,         // Open gripper at end zone
  STATE_DONE               // Task complete, stop
};

RobotState currentState = STATE_LINE_FOLLOW;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("=== Robot Init v2 ===");

  // Motor driver
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // TCS230
  pinMode(TCS_S0_PIN, OUTPUT);
  pinMode(TCS_S1_PIN, OUTPUT);
  pinMode(TCS_S2_PIN, OUTPUT);
  pinMode(TCS_S3_PIN, OUTPUT);
  pinMode(TCS_OUT_PIN, INPUT);

  // Frequency scaling: S0=HIGH, S1=LOW → 20% scaling (good for indoor)
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // Servo — start closed (gripping nothing but ready)
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(SERVO_OPEN);   // open at start, cube not loaded yet
  delay(500);

  Serial.println("Starting in 3s...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readIRSensors();

  switch (currentState) {

    // ----------------------------------------------------------
    case STATE_LINE_FOLLOW: {
      float dist = readUltrasonic();

      // Object detected ahead → stop → identify it
      if (dist > 0 && dist < DETECT_DIST) {
        stopMotors();
        delay(300);
        Serial.print("Object at ");
        Serial.print(dist);
        Serial.println("cm -> Identifying color...");
        currentState = STATE_IDENTIFY_OBJECT;
        break;
      }

      // Nothing ahead → follow line normally
      followLine();
      break;
    }

    // ----------------------------------------------------------
    // KEY STATE: Read color → decide RED or GREEN
    // This is the ONLY place color is read during normal operation
    // ----------------------------------------------------------
    case STATE_IDENTIFY_OBJECT: {
      char color = readColorMultiple(5); // average 5 readings for accuracy

      Serial.print("Color identified: ");
      Serial.println(color);

      if (color == 'R') {
        // RED CUBE → obstacle → avoid it
        Serial.println(">> RED obstacle -> AVOID");
        currentState = STATE_AVOID_RED;

      } else if (color == 'G' && !cubePickedUp) {
        // GREEN CUBE → target → pick it up
        Serial.println(">> GREEN cube -> APPROACH & PICK");
        currentState = STATE_APPROACH_GREEN;

      } else if (color == 'G' && cubePickedUp) {
        // Already carrying green, this shouldn't happen
        // Treat as obstacle just in case
        Serial.println(">> Already carrying cube, avoiding unknown");
        currentState = STATE_AVOID_RED;

      } else {
        // Unknown color → treat as obstacle to be safe
        Serial.println(">> Unknown color -> treating as obstacle");
        currentState = STATE_AVOID_RED;
      }
      break;
    }

    // ----------------------------------------------------------
    case STATE_AVOID_RED: {
      avoidRedObstacle();
      obstaclesAvoided++;
      Serial.print("Obstacles avoided so far: ");
      Serial.println(obstaclesAvoided);
      currentState = STATE_REJOIN_LINE;
      break;
    }

    // ----------------------------------------------------------
    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) {
        Serial.println("Line rejoined -> resuming follow");
        currentState = STATE_LINE_FOLLOW;
      } else {
        // Line not found after timeout → try again slowly
        Serial.println("WARNING: Line not found, retrying...");
        // Stay in rejoin state and try again
      }
      break;
    }

    // ----------------------------------------------------------
    case STATE_APPROACH_GREEN: {
      // Creep forward slowly to get gripper right onto the cube
      Serial.println("Approaching green cube slowly...");
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(400);   // TUNE: how far to creep before gripping
      stopMotors();
      delay(200);
      currentState = STATE_PICK_GREEN;
      break;
    }

    // ----------------------------------------------------------
    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      currentState = STATE_CARRY_TO_END;
      break;
    }

    // ----------------------------------------------------------
    case STATE_CARRY_TO_END: {
      // Follow line while carrying cube
      // Detect end zone by:
      //   Method 1 (primary)   : timer since pickup exceeded END_ZONE_TRAVEL_MS
      //   Method 2 (secondary) : ultrasonic sees end marker/wall close up

      unsigned long elapsed = millis() - pickUpTime;

      // Method 1: timer based end zone
      if (elapsed > END_ZONE_TRAVEL_MS) {
        Serial.println(">> End zone reached (timer)! Dropping...");
        stopMotors();
        delay(300);
        currentState = STATE_DROP_CUBE;
        break;
      }

      // Method 2: wall/marker at end zone
      float dist = readUltrasonic();
      if (dist > 0 && dist < 10) {   // very close = end wall
        Serial.println(">> End zone reached (wall detected)! Dropping...");
        stopMotors();
        delay(300);
        currentState = STATE_DROP_CUBE;
        break;
      }

      // Otherwise keep following line
      followLine();
      break;
    }

    // ----------------------------------------------------------
    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      currentState = STATE_DONE;
      break;
    }

    // ----------------------------------------------------------
    case STATE_DONE: {
      stopMotors();
      // Flash via Serial to indicate success
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 1000) {
        Serial.println("*** TASK COMPLETE - ALL DONE ***");
        lastPrint = millis();
      }
      break;
    }
  }
}

// ============================================================
//  READ IR SENSORS
// ============================================================
void readIRSensors() {
  irRaw[0] = analogRead(IR_S1_PIN);
  irRaw[1] = analogRead(IR_S2_PIN);
  irRaw[2] = analogRead(IR_S3_PIN);
  irRaw[3] = analogRead(IR_S4_PIN);
  irRaw[4] = analogRead(IR_S5_PIN);  // A6

  for (int i = 0; i < 5; i++) {
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // Uncomment below to calibrate IR threshold on Serial Monitor:
  /*
  Serial.print("IR RAW: ");
  for(int i=0;i<5;i++){
    Serial.print(irRaw[i]);
    Serial.print("\t");
  }
  Serial.println();
  */
}

// ============================================================
//  LINE FOLLOWING — PD Controller
// ============================================================
void followLine() {
  // Weighted position error
  // Weights: S1=-2(far left) ... S5=+2(far right)
  // Negative error = line is to the LEFT  → turn left
  // Positive error = line is to the RIGHT → turn right
  int weights[5]  = {-2, -1, 0, 1, 2};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  int error;
  if (sensorSum > 0) {
    error = weightedSum;  // normal: range -4 to +4
  } else {
    // All sensors off line → last resort: hard turn in last direction
    error = (lastError > 0) ? 4 : -4;
  }

  // PD correction value
  float correction = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  // Clamp speeds to [-255, 255]
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Cap sharp turns so robot doesn't spin too hard
  if (abs(error) >= 2) {
    if (leftSpeed  > 0) leftSpeed  = min(leftSpeed,  SHARP_SPEED);
    if (rightSpeed > 0) rightSpeed = min(rightSpeed, SHARP_SPEED);
  }

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  AVOID RED OBSTACLE
//  Strategy: Left bypass (U-shape around cube)
//  Timing values MUST be tuned on actual arena!
// ============================================================
void avoidRedObstacle() {
  Serial.println("Avoiding RED cube...");

  // Step 1: Turn LEFT (pivot)
  driveMotors(-AVOID_SPEED, AVOID_SPEED);
  delay(380);    // ~90 degrees turn — TUNE THIS
  stopMotors();
  delay(150);

  // Step 2: Move forward past the obstacle
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(650);    // forward distance past cube — TUNE THIS
  stopMotors();
  delay(150);

  // Step 3: Turn RIGHT to face back toward line
  driveMotors(AVOID_SPEED, -AVOID_SPEED);
  delay(380);    // ~90 degrees — TUNE THIS
  stopMotors();
  delay(150);

  // Step 4: Move forward to get over line
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(400);    // TUNE THIS
  stopMotors();
  delay(200);

  Serial.println("Avoidance maneuver done.");
}

// ============================================================
//  REJOIN LINE AFTER AVOIDANCE
//  Sweeps forward slowly until IR sensors detect yellow line
// ============================================================
bool rejoinLine() {
  Serial.println("Searching for line...");
  unsigned long startTime = millis();
  unsigned long timeout   = 4000;  // 4 seconds max search

  while (millis() - startTime < timeout) {
    readIRSensors();

    int lineCount = 0;
    for (int i = 0; i < 5; i++) lineCount += irBinary[i];

    if (lineCount >= 1) {
      Serial.println("Line found!");
      return true;
    }

    // Slowly creep forward looking for line
    driveMotors(110, 110);
    delay(40);
  }

  stopMotors();
  Serial.println("ERROR: Could not find line!");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
// ============================================================
void pickGreenCube() {
  Serial.println("Opening gripper...");
  gripperServo.write(SERVO_OPEN);
  delay(500);

  // Ensure robot is stopped and stable
  stopMotors();
  delay(300);

  Serial.println("Closing gripper to grip cube...");
  gripperServo.write(SERVO_CLOSED);
  delay(800);   // give servo time to close fully

  Serial.println("Green cube PICKED UP!");
}

// ============================================================
//  DROP CUBE AT END ZONE
// ============================================================
void dropCubeAtEnd() {
  Serial.println("Opening gripper to release cube...");

  stopMotors();
  delay(200);

  gripperServo.write(SERVO_OPEN);
  delay(800);

  // Back up slightly so robot doesn't sit on cube
  driveMotors(-100, -100);
  delay(300);
  stopMotors();

  cubePickedUp = false;
  Serial.println("Cube PLACED at end zone!");
}

// ============================================================
//  COLOR SENSOR — TCS230
//  Read color multiple times and return most common result
//  for noise resistance
// ============================================================

// Single color reading - returns 'R', 'G', 'B', or 'U'
char readColorOnce() {
  long r, g, b;

  // Read RED channel: S2=LOW, S3=LOW
  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, LOW);
  delay(10);
  r = pulseIn(TCS_OUT_PIN, LOW, 60000);

  // Read GREEN channel: S2=HIGH, S3=HIGH
  digitalWrite(TCS_S2_PIN, HIGH);
  digitalWrite(TCS_S3_PIN, HIGH);
  delay(10);
  g = pulseIn(TCS_OUT_PIN, LOW, 60000);

  // Read BLUE channel: S2=LOW, S3=HIGH
  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, HIGH);
  delay(10);
  b = pulseIn(TCS_OUT_PIN, LOW, 60000);

  Serial.print("  R="); Serial.print(r);
  Serial.print("  G="); Serial.print(g);
  Serial.print("  B="); Serial.println(b);

  // TCS230: LOWER frequency = STRONGER that color
  // If pulseIn returns 0 = timeout = no object, return Unknown

  if (r == 0 && g == 0 && b == 0) return 'U';

  // ---- CALIBRATE THESE THRESHOLDS ----
  // Step 1: Point sensor at RED cube   → note R,G,B values
  // Step 2: Point sensor at GREEN cube → note R,G,B values
  // Step 3: Set thresholds so RED and GREEN are clearly separated
  //
  // Example calibration results (yours WILL differ):
  //   RED cube:   R=45  G=110  B=95
  //   GREEN cube: R=110 G=40   B=90
  //
  // Decision logic: smallest value = dominant color
  // Additional absolute threshold avoids false triggers far away

  // Absolute threshold: ignore if all values too high (no cube close)
  long minVal = min(r, min(g, b));
  if (minVal > 200) return 'U';  // too far, no reliable reading

  if (r < g && r < b) {
    return 'R';  // Red dominant
  }
  if (g < r && g < b) {
    return 'G';  // Green dominant
  }
  if (b < r && b < g) {
    return 'B';  // Blue dominant
  }

  return 'U';  // Cannot determine
}

// Read color N times, return the most frequent result (voting)
char readColorMultiple(int times) {
  int countR = 0, countG = 0, countB = 0, countU = 0;

  for (int i = 0; i < times; i++) {
    char c = readColorOnce();
    if      (c == 'R') countR++;
    else if (c == 'G') countG++;
    else if (c == 'B') countB++;
    else               countU++;
    delay(30);
  }

  Serial.print("Vote -> R:"); Serial.print(countR);
  Serial.print(" G:"); Serial.print(countG);
  Serial.print(" B:"); Serial.print(countB);
  Serial.print(" U:"); Serial.println(countU);

  // Return whichever color got the most votes
  if (countR >= countG && countR >= countB && countR > countU) return 'R';
  if (countG >= countR && countG >= countB && countG > countU) return 'G';
  if (countB >= countR && countB >= countG && countB > countU) return 'B';

  return 'U';
}

// ============================================================
//  ULTRASONIC SENSOR — HC-SR04
// ============================================================
float readUltrasonic() {
  // Send 10us pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Receive echo (timeout 25ms = max ~425cm range)
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  if (duration == 0) return 999.0;  // timeout = no object

  float distance = (duration * 0.0343) / 2.0;
  return distance;  // in cm
}

// ============================================================
//  MOTOR CONTROL — L298N
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // --- LEFT MOTOR ---
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, leftSpeed);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, -leftSpeed);
  }

  // --- RIGHT MOTOR ---
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, rightSpeed);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

// ============================================================
//  END OF CODE v2
// ============================================================
