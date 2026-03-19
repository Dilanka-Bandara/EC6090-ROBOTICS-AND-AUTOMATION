/*
 * ============================================================
 *  LINE FOLLOWING ROBOT — ARDUINO UNO (TESTING VERSION v3)
 * ============================================================
 *  Hardware:
 *    - Arduino Uno
 *    - 5x IR Sensors (Analog Output)
 *    - TCS230 Color Sensor
 *    - HC-SR04 Ultrasonic Sensor
 *    - L298N Motor Driver (1x, rear wheel drive)
 *    - SG90 Servo Gripper (SNM200 Claw)
 *
 *  FIXED in v3:
 *    - Removed A6 (doesn't exist on Uno)
 *    - IR S5 moved to A4
 *    - TCS230 S3 moved to A5 (used as digital output)
 *    - TCS230 S2 + OUT share D13 (pinMode switched in code)
 *    - Full pin audit — all 20 I/O pins accounted for
 *
 *  FINAL PIN MAP:
 *    D0       → Reserved (Serial RX) — DO NOT USE
 *    D1       → Reserved (Serial TX) — DO NOT USE
 *    D2       → L298N IN1
 *    D3       → L298N IN2
 *    D4       → L298N IN3
 *    D5       → L298N IN4
 *    D6~      → L298N ENB (right motor PWM speed)
 *    D7       → HC-SR04 TRIG
 *    D8       → HC-SR04 ECHO
 *    D9~      → L298N ENA (left motor PWM speed)
 *    D10~     → SG90 Servo signal
 *    D11      → TCS230 S0
 *    D12      → TCS230 S1
 *    D13      → TCS230 S2 (OUTPUT) / TCS230 OUT (INPUT) — shared
 *    A0       → IR Sensor S1 (Far Left)
 *    A1       → IR Sensor S2 (Mid Left)
 *    A2       → IR Sensor S3 (Center)
 *    A3       → IR Sensor S4 (Mid Right)
 *    A4       → IR Sensor S5 (Far Right)
 *    A5       → TCS230 S3 (used as digital output)
 *
 *  Author  : EC6090 Mini Project 2026
 *  Board   : Arduino Uno (migrate to ESP32 + 7 sensors later)
 * ============================================================
 */

#include <Servo.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// --- IR Sensors (Analog Output) ---
#define IR_S1   A0    // Far Left
#define IR_S2   A1    // Mid Left
#define IR_S3   A2    // Center
#define IR_S4   A3    // Mid Right
#define IR_S5   A4    // Far Right

// --- HC-SR04 Ultrasonic ---
#define TRIG_PIN  7
#define ECHO_PIN  8

// --- L298N Motor Driver ---
#define IN1_PIN   2     // Left motor direction A
#define IN2_PIN   3     // Left motor direction B
#define IN3_PIN   4     // Right motor direction A
#define IN4_PIN   5     // Right motor direction B
#define ENA_PIN   9     // Left motor PWM speed  (PWM~)
#define ENB_PIN   6     // Right motor PWM speed (PWM~)

// --- SG90 Servo ---
#define SERVO_PIN 10    // PWM~

// --- TCS230 Color Sensor ---
// S0, S1 = frequency output scaling
// S2, S3 = photodiode filter select
// OUT     = frequency output (pulse)
// NOTE:  D13 is SHARED between S2 (output) and OUT (input)
//        pinMode is switched in code before each use
#define TCS_S0_PIN    11
#define TCS_S1_PIN    12
#define TCS_D13_PIN   13    // shared: S2 output + OUT input
#define TCS_S3_PIN    A5    // A5 used as digital output

// ============================================================
//  TUNABLE CONSTANTS — CALIBRATE ON YOUR ACTUAL ARENA
// ============================================================

// IR threshold: analog value ABOVE this = sensor on yellow line
// How to calibrate:
//   1. Open Serial Monitor
//   2. Place robot on white/bare floor → note values (e.g. 200)
//   3. Place robot on yellow line      → note values (e.g. 750)
//   4. Set threshold halfway:  (200+750)/2 = ~475
#define IR_THRESHOLD      500

// Ultrasonic: stop and identify object if closer than this (cm)
#define DETECT_DIST       18

// Motor speeds (0–255 PWM)
#define BASE_SPEED        155   // normal line following speed
#define SHARP_SPEED       100   // max speed on sharp turns
#define AVOID_SPEED       140   // obstacle bypass speed
#define CREEP_SPEED       100   // slow approach for pickup

// PD Controller gains — start here, tune on arena
// Increase Kp if robot is slow to correct
// Decrease Kp if robot wobbles/oscillates
// Increase Kd to reduce overshoot on curves
float Kp = 25.0;
float Kd = 12.0;

// Servo claw angles — adjust to match your physical claw
#define SERVO_OPEN    85    // claw fully open  (release)
#define SERVO_CLOSED  15    // claw fully closed (grip)

// Time (ms) robot travels after picking green cube before
// assuming it reached the end zone — TUNE for your arena length
#define END_ZONE_TRAVEL_MS  8000

// ============================================================
//  GLOBAL VARIABLES
// ============================================================

Servo gripperServo;

int irRaw[5];         // raw analog readings
int irBinary[5];      // 1 = on yellow line, 0 = off line

int lastError     = 0;
bool cubePickedUp = false;
int  obstaclesAvoided = 0;
unsigned long pickUpTime = 0;

// ============================================================
//  STATE MACHINE
// ============================================================

enum RobotState {
  STATE_LINE_FOLLOW,
  STATE_IDENTIFY_OBJECT,
  STATE_AVOID_RED,
  STATE_REJOIN_LINE,
  STATE_APPROACH_GREEN,
  STATE_PICK_GREEN,
  STATE_CARRY_TO_END,
  STATE_DROP_CUBE,
  STATE_DONE
};

RobotState currentState = STATE_LINE_FOLLOW;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("=== Robot v3 — Arduino Uno ===");
  Serial.println("Correct pin map: no A6, D13 shared");

  // Motor driver
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // TCS230 fixed pins
  pinMode(TCS_S0_PIN,  OUTPUT);
  pinMode(TCS_S1_PIN,  OUTPUT);
  pinMode(TCS_S3_PIN,  OUTPUT);
  // D13 starts as OUTPUT for S2 control
  pinMode(TCS_D13_PIN, OUTPUT);

  // TCS230 frequency scaling: S0=HIGH, S1=LOW = 20% scaling
  // Good balance between speed and accuracy for indoor use
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // Servo — open claw at start
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(SERVO_OPEN);
  delay(500);

  Serial.println("Starting in 3 seconds...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readIRSensors();

  switch (currentState) {

    // --------------------------------------------------------
    case STATE_LINE_FOLLOW: {
      float dist = readUltrasonic();
      if (dist > 0 && dist < DETECT_DIST) {
        stopMotors();
        delay(300);
        Serial.print("Object at ");
        Serial.print(dist);
        Serial.println("cm — reading color...");
        currentState = STATE_IDENTIFY_OBJECT;
        break;
      }
      followLine();
      break;
    }

    // --------------------------------------------------------
    // ONLY place color is read — robot is always stopped here
    // --------------------------------------------------------
    case STATE_IDENTIFY_OBJECT: {
      char color = readColorVoted(5);
      Serial.print("Color result: ");
      Serial.println(color);

      if (color == 'R') {
        Serial.println(">> RED — avoiding obstacle");
        currentState = STATE_AVOID_RED;
      } else if (color == 'G' && !cubePickedUp) {
        Serial.println(">> GREEN — picking up cube");
        currentState = STATE_APPROACH_GREEN;
      } else if (color == 'G' && cubePickedUp) {
        Serial.println(">> GREEN seen but already carrying — avoiding");
        currentState = STATE_AVOID_RED;
      } else {
        Serial.println(">> Unknown color — treating as obstacle");
        currentState = STATE_AVOID_RED;
      }
      break;
    }

    // --------------------------------------------------------
    case STATE_AVOID_RED: {
      avoidRedObstacle();
      obstaclesAvoided++;
      Serial.print("Total avoided: ");
      Serial.println(obstaclesAvoided);
      currentState = STATE_REJOIN_LINE;
      break;
    }

    // --------------------------------------------------------
    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) {
        currentState = STATE_LINE_FOLLOW;
      }
      // if not found, stays here and tries again next loop
      break;
    }

    // --------------------------------------------------------
    case STATE_APPROACH_GREEN: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(400);   // creep forward onto cube — tune this!
      stopMotors();
      delay(200);
      currentState = STATE_PICK_GREEN;
      break;
    }

    // --------------------------------------------------------
    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      currentState = STATE_CARRY_TO_END;
      break;
    }

    // --------------------------------------------------------
    case STATE_CARRY_TO_END: {
      // End zone detection: timer OR wall close ahead
      unsigned long elapsed = millis() - pickUpTime;

      if (elapsed > END_ZONE_TRAVEL_MS) {
        Serial.println(">> End zone (timer) — dropping cube");
        stopMotors();
        delay(200);
        currentState = STATE_DROP_CUBE;
        break;
      }

      float dist = readUltrasonic();
      if (dist > 0 && dist < 8) {
        Serial.println(">> End zone (wall) — dropping cube");
        stopMotors();
        delay(200);
        currentState = STATE_DROP_CUBE;
        break;
      }

      followLine();
      break;
    }

    // --------------------------------------------------------
    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      currentState = STATE_DONE;
      break;
    }

    // --------------------------------------------------------
    case STATE_DONE: {
      stopMotors();
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 2000) {
        Serial.println("*** TASK COMPLETE ***");
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
  irRaw[0] = analogRead(IR_S1);
  irRaw[1] = analogRead(IR_S2);
  irRaw[2] = analogRead(IR_S3);
  irRaw[3] = analogRead(IR_S4);
  irRaw[4] = analogRead(IR_S5);   // A4 — no longer A6!

  for (int i = 0; i < 5; i++) {
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // Uncomment to calibrate IR_THRESHOLD:
  /*
  Serial.print("RAW: ");
  for (int i = 0; i < 5; i++) {
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
  int weights[5]  = {-2, -1, 0, 1, 2};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  int error;
  if (sensorSum > 0) {
    error = weightedSum;
  } else {
    // All sensors off line — hard turn in last known direction
    error = (lastError > 0) ? 4 : -4;
  }

  float correction = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Cap speed on sharp turns
  if (abs(error) >= 2) {
    if (leftSpeed  > 0) leftSpeed  = min(leftSpeed,  SHARP_SPEED);
    if (rightSpeed > 0) rightSpeed = min(rightSpeed, SHARP_SPEED);
  }

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  AVOID RED OBSTACLE — Left bypass
//  All delay() values must be tuned on your actual arena!
// ============================================================
void avoidRedObstacle() {
  Serial.println("Bypassing red obstacle...");

  // Turn left ~90 degrees
  driveMotors(-AVOID_SPEED, AVOID_SPEED);
  delay(380);
  stopMotors();
  delay(150);

  // Move forward past obstacle width
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(650);
  stopMotors();
  delay(150);

  // Turn right ~90 degrees back toward line
  driveMotors(AVOID_SPEED, -AVOID_SPEED);
  delay(380);
  stopMotors();
  delay(150);

  // Move forward to cross back over line
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(400);
  stopMotors();
  delay(200);
}

// ============================================================
//  REJOIN LINE
// ============================================================
bool rejoinLine() {
  Serial.println("Searching for line...");
  unsigned long start = millis();

  while (millis() - start < 4000) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];

    if (count >= 1) {
      Serial.println("Line found!");
      return true;
    }

    driveMotors(110, 110);
    delay(40);
  }

  stopMotors();
  Serial.println("WARNING: Line not found!");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
// ============================================================
void pickGreenCube() {
  gripperServo.write(SERVO_OPEN);
  delay(600);
  stopMotors();
  delay(300);
  gripperServo.write(SERVO_CLOSED);
  delay(800);
  Serial.println("Cube gripped!");
}

// ============================================================
//  DROP CUBE AT END ZONE
// ============================================================
void dropCubeAtEnd() {
  stopMotors();
  delay(200);
  gripperServo.write(SERVO_OPEN);
  delay(800);
  // Back away from cube
  driveMotors(-100, -100);
  delay(300);
  stopMotors();
  cubePickedUp = false;
  Serial.println("Cube released at end zone!");
}

// ============================================================
//  TCS230 COLOR SENSOR
//
//  IMPORTANT: D13 is SHARED between S2 (output) and OUT (input)
//  We switch pinMode before each use:
//    - Before writing S2  → pinMode(D13, OUTPUT)
//    - Before reading OUT → pinMode(D13, INPUT)
//  This works because S2 is set BEFORE taking the reading,
//  then D13 switches to INPUT to receive the pulse frequency.
// ============================================================

char readColorOnce() {
  long r = 0, g = 0, b = 0;

  // --- Read RED channel ---
  // S2=LOW, S3=LOW selects red photodiodes
  pinMode(TCS_D13_PIN, OUTPUT);       // D13 = S2 output mode
  digitalWrite(TCS_D13_PIN, LOW);     // S2 = LOW
  digitalWrite(TCS_S3_PIN,  LOW);     // S3 = LOW  (A5 as digital)
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);        // D13 = OUT input mode
  r = pulseIn(TCS_D13_PIN, LOW, 60000);

  // --- Read GREEN channel ---
  // S2=HIGH, S3=HIGH selects green photodiodes
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, HIGH);    // S2 = HIGH
  digitalWrite(TCS_S3_PIN,  HIGH);    // S3 = HIGH
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  g = pulseIn(TCS_D13_PIN, LOW, 60000);

  // --- Read BLUE channel ---
  // S2=LOW, S3=HIGH selects blue photodiodes
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);     // S2 = LOW
  digitalWrite(TCS_S3_PIN,  HIGH);    // S3 = HIGH
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  b = pulseIn(TCS_D13_PIN, LOW, 60000);

  // Debug — uncomment to calibrate color thresholds:
  /*
  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);
  */

  // Timeout = no object close enough
  if (r == 0 && g == 0 && b == 0) return 'U';

  // Too far away = ignore (all values too large)
  long minVal = min(r, min(g, b));
  if (minVal > 250) return 'U';

  // TCS230: LOWER frequency value = STRONGER that color
  // The dominant color has the SMALLEST pulse width value
  if (r < g && r < b) return 'R';
  if (g < r && g < b) return 'G';
  if (b < r && b < g) return 'B';

  return 'U';
}

// Read color N times and return most voted result
// Eliminates noise from single bad readings
char readColorVoted(int times) {
  int cR = 0, cG = 0, cB = 0, cU = 0;

  for (int i = 0; i < times; i++) {
    char c = readColorOnce();
    if      (c == 'R') cR++;
    else if (c == 'G') cG++;
    else if (c == 'B') cB++;
    else               cU++;
    delay(25);
  }

  Serial.print("Votes -> R:"); Serial.print(cR);
  Serial.print(" G:"); Serial.print(cG);
  Serial.print(" B:"); Serial.print(cB);
  Serial.print(" U:"); Serial.println(cU);

  // Return winner only if it beats unknowns
  if (cR >= cG && cR >= cB && cR > cU) return 'R';
  if (cG >= cR && cG >= cB && cG > cU) return 'G';
  if (cB >= cR && cB >= cG && cB > cU) return 'B';
  return 'U';
}

// ============================================================
//  ULTRASONIC — HC-SR04
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return 999.0;

  return (duration * 0.0343) / 2.0;
}

// ============================================================
//  MOTOR CONTROL — L298N
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, leftSpeed);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, -leftSpeed);
  }
  // Right motor
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
//  END OF CODE v3
// ============================================================
