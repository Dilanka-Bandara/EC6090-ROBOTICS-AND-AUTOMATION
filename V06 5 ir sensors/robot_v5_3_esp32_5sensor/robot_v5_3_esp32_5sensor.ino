/*
 * ============================================================
 *  LINE FOLLOWING ROBOT — ESP32 30-PIN DEVKIT V1 (v5.3)
 * ============================================================
 *  Board: ESP32 DevKit V1 — 30 pin (D2, D3, D4... labels)
 *
 *  FIXED in v5.2:
 *    ledcSetup()    → ledcAttach(pin, freq, res)  ✅
 *    ledcAttachPin()→ removed (built into ledcAttach) ✅
 *    ledcWrite(ch)  → ledcWrite(pin, val)          ✅
 *    Reason: ESP32 Arduino core v3.x changed LEDC API
 *
 *  SAFE PIN REMAPPING from v5:
 *    Removed GPIO 2  (strapping pin — boot issues)
 *    Removed GPIO 5  (strapping pin — boot issues)
 *    Removed GPIO 12 (strapping pin — boot issues)
 *    Removed GPIO 15 (strapping pin — boot issues)
 *    Removed GPIO 36,39 (may not exist on all 30-pin boards)
 *
 *  FINAL SAFE PIN MAP (30-pin DevKit V1):
 *
 *    YOUR BOARD     GPIO    Component
 *    ─────────────────────────────────────────
 *    D16         =  16   →  L298N IN1  (Left  motor dir A)
 *    D17         =  17   →  L298N IN2  (Left  motor dir B)
 *    D18         =  18   →  L298N IN3  (Right motor dir A)
 *    D19         =  19   →  L298N IN4  (Right motor dir B)
 *    D25         =  25   →  L298N ENA  (Left  motor PWM)
 *    D26         =  26   →  L298N ENB  (Right motor PWM)
 *    D27         =  27   →  SG90 Servo signal
 *    D22         =  22   →  HC-SR04 TRIG
 *    D23         =  23   →  HC-SR04 ECHO (+ voltage divider!)
 *    D13         =  13   →  TCS230 S0
 *    D14         =  14   →  TCS230 S1
 *    D21         =  21   →  TCS230 S2
 *    D4          =  4    →  TCS230 S3
 *    D33         =  33   →  TCS230 OUT
 *    D32         =  32   →  IR Sensor S1 (Far  Left)
 *    D35         =  35   →  IR Sensor S2 (Mid  Left)   input only
 *    D34         =  34   →  IR Sensor S3 (Left Center) input only
 *    D3 (RX)     =  3    →  IR Sensor S4 (Center)
 *    D1 (TX)     =  1    →  IR Sensor S5 (Right Center)
 *    D2 (boot)   AVOID
 *    D5 (boot)   AVOID
 *    D12 (boot)  AVOID
 *    D15 (boot)  AVOID
 *
 *  ⚠️ NOTE on D3 and D1 for IR sensors S4 and S5:
 *    These are Serial RX/TX pins — they work fine as inputs
 *    when Serial is only used for debug (not receiving data)
 *    If Serial debug causes issues, swap to D0 or D10
 *
 *  ⚠️ VOLTAGE:
 *    All GPIO = 3.3V
 *    IR sensors → 3.3V VCC
 *    TCS230     → 3.3V VCC
 *    HC-SR04    → 5V VCC, ECHO needs voltage divider!
 *    Servo      → 5V VCC (Vin pin)
 *    L298N      → 5V VCC (Vin pin)
 *
 *  HC-SR04 ECHO voltage divider:
 *    ECHO → 1kΩ → D23(GPIO23)
 *                     |
 *                    2kΩ
 *                     |
 *                    GND
 *
 *  Arduino IDE board selection:
 *    Tools → Board → ESP32 Arduino → ESP32 Dev Module
 *
 *  Required library:
 *    ESP32Servo (install via Library Manager)
 *
 *  Author : EC6090 Mini Project 2026
 * ============================================================
 */

#include <ESP32Servo.h>

// ============================================================
//  PIN DEFINITIONS — ESP32 30-pin DevKit V1
//  Use D-number from your board = GPIO number in code
// ============================================================

// --- 5x IR Sensors (Analog, 5V VCC + voltage divider on OUT) ---
// Currently using 5 sensors — upgrade to 7 later
// Layout front of robot: S1=Far Left ... S5=Far Right
//
//   Board label   GPIO   Sensor position
//   ──────────────────────────────────────
//   D35           35     S1 Far  Left  (input only)
//   D34           34     S2 Mid  Left  (input only)
//   VN            39     S3 Center     (input only)
//   VP            36     S4 Mid  Right (input only)
//   D33           33     S5 Far  Right
//
// TO UPGRADE TO 7 SENSORS LATER — add these two:
//   D32           32     S0 Far Left extra  (input only)
//   D32           32     S6 Far Right extra
//   Change NUM_SENSORS 5 → 7, weights -2..+2 → -3..+3
#define IR_S1   35    // D35 — Far  Left  (input only)
#define IR_S2   34    // D34 — Mid  Left  (input only)
#define IR_S3   39    // VN  — Center     (input only)
#define IR_S4   36    // VP  — Mid  Right (input only)
#define IR_S5   33    // D33 — Far  Right

// --- HC-SR04 Ultrasonic ---
#define TRIG_PIN  22    // D22
#define ECHO_PIN  23    // D23 — needs voltage divider 5V→3.3V!

// --- L298N Motor Driver ---
#define IN1_PIN   16    // D16 — Left  motor dir A
#define IN2_PIN   17    // D17 — Left  motor dir B
#define IN3_PIN   18    // D18 — Right motor dir A
#define IN4_PIN   19    // D19 — Right motor dir B
#define ENA_PIN   25    // D25 — Left  motor PWM (LEDC ch0)
#define ENB_PIN   26    // D26 — Right motor PWM (LEDC ch1)

// --- SG90 Servo ---
#define SERVO_PIN 27    // D27 — LEDC ch2

// --- TCS230 Color Sensor ---
#define TCS_S0_PIN   13   // D13
#define TCS_S1_PIN   14   // D14
#define TCS_S2_PIN   21   // D21
#define TCS_S3_PIN   4    // D4
#define TCS_OUT_PIN  33   // D33 — dedicated input ✅

// ============================================================
//  ESP32 LEDC PWM CONFIGURATION — core v3.x API
//  New API: ledcAttach(pin, freq, resolution)
//           ledcWrite(pin, value)   ← use PIN not channel!
//  No more ledcSetup() or ledcAttachPin() in core v3.x
// ============================================================
#define PWM_FREQ        5000    // 5kHz for motors
#define PWM_RESOLUTION  8       // 8-bit = 0-255 (same as Uno)
#define SERVO_FREQ      50      // 50Hz for servo

// ============================================================
//  MOTOR TEST MODE
// ============================================================
#define MOTOR_TEST_MODE  false

// ============================================================
//  TUNABLE CONSTANTS
// ============================================================

// ESP32 ADC = 12-bit (0–4095)
// Start at 2000, calibrate on your arena
#define IR_THRESHOLD    2600

#define DETECT_DIST     18     // cm — ultrasonic stop distance

#define BASE_SPEED      155
#define SHARP_SPEED     100
#define AVOID_SPEED     140
#define CREEP_SPEED     100

float Kp = 25.0;
float Kd = 12.0;

#define SERVO_OPEN      85
#define SERVO_CLOSED    15

#define END_ZONE_TRAVEL_MS  8000

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
Servo gripperServo;

int  irRaw[5];
int  irBinary[5];

int  lastError        = 0;
bool cubePickedUp     = false;
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
  Serial.begin(115200);
  Serial.println("=== Robot v5.3 — ESP32 5 sensors, upgrade to 7 later ===");
  Serial.println("Safe pins only — boot strapping pins avoided");

  // Motor direction pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // LEDC PWM setup — core v3.x API
  // Single call: ledcAttach(pin, frequency, resolution)
  ledcAttach(ENA_PIN,   PWM_FREQ,   PWM_RESOLUTION);  // left motor
  ledcAttach(ENB_PIN,   PWM_FREQ,   PWM_RESOLUTION);  // right motor
  ledcAttach(SERVO_PIN, SERVO_FREQ, PWM_RESOLUTION);  // servo

  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // TCS230
  pinMode(TCS_S0_PIN,  OUTPUT);
  pinMode(TCS_S1_PIN,  OUTPUT);
  pinMode(TCS_S2_PIN,  OUTPUT);
  pinMode(TCS_S3_PIN,  OUTPUT);
  pinMode(TCS_OUT_PIN, INPUT);

  // TCS230 frequency scaling 20%
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // IR input-only pins (35,34,39,36) need no pinMode call
  // Only S5 (D33) needs pinMode since it is not input-only
  pinMode(IR_S5, INPUT);

  // Servo
  gripperServo.attach(SERVO_PIN, 500, 2400);
  gripperServo.write(SERVO_OPEN);
  delay(500);

  // Motor test
  if (MOTOR_TEST_MODE) {
    Serial.println("=== MOTOR TEST ===");
    driveMotors(150, 150);  delay(2000);
    driveMotors(150, 0);    delay(2000);
    driveMotors(0, 150);    delay(2000);
    driveMotors(-150,-150); delay(2000);
    stopMotors();
    Serial.println("Done. Set MOTOR_TEST_MODE false.");
    while(true);
  }

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

    case STATE_LINE_FOLLOW: {
      float dist = readUltrasonic();
      if (dist > 0 && dist < DETECT_DIST) {
        stopMotors();
        delay(300);
        Serial.print("Object at ");
        Serial.print(dist);
        Serial.println("cm");
        currentState = STATE_IDENTIFY_OBJECT;
        break;
      }
      followLine();
      break;
    }

    case STATE_IDENTIFY_OBJECT: {
      char color = readColorVoted(5);
      Serial.print("Color: "); Serial.println(color);
      if (color == 'R') {
        currentState = STATE_AVOID_RED;
      } else if (color == 'G' && !cubePickedUp) {
        currentState = STATE_APPROACH_GREEN;
      } else {
        currentState = STATE_AVOID_RED;
      }
      break;
    }

    case STATE_AVOID_RED: {
      avoidRedObstacle();
      obstaclesAvoided++;
      currentState = STATE_REJOIN_LINE;
      break;
    }

    case STATE_REJOIN_LINE: {
      if (rejoinLine()) currentState = STATE_LINE_FOLLOW;
      break;
    }

    case STATE_APPROACH_GREEN: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(400);
      stopMotors(); delay(200);
      currentState = STATE_PICK_GREEN;
      break;
    }

    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      currentState = STATE_CARRY_TO_END;
      break;
    }

    case STATE_CARRY_TO_END: {
      if (millis() - pickUpTime > END_ZONE_TRAVEL_MS) {
        stopMotors();
        currentState = STATE_DROP_CUBE;
        break;
      }
      float dist = readUltrasonic();
      if (dist > 0 && dist < 8) {
        stopMotors();
        currentState = STATE_DROP_CUBE;
        break;
      }
      followLine();
      break;
    }

    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      currentState = STATE_DONE;
      break;
    }

    case STATE_DONE: {
      stopMotors();
      static unsigned long lp = 0;
      if (millis() - lp > 2000) {
        Serial.println("*** TASK COMPLETE ***");
        lp = millis();
      }
      break;
    }
  }
}

// ============================================================
//  READ 7 IR SENSORS
// ============================================================
void readIRSensors() {
  irRaw[0] = analogRead(IR_S1);
  irRaw[1] = analogRead(IR_S2);
  irRaw[2] = analogRead(IR_S3);
  irRaw[3] = analogRead(IR_S4);
  irRaw[4] = analogRead(IR_S5);

  for (int i = 0; i < 5; i++) {
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // Uncomment to calibrate:
  /*
  Serial.print("RAW: ");
  for(int i=0;i<5;i++){
    Serial.print(irRaw[i]); Serial.print("\t");
  }
  Serial.println();
  */
}

// ============================================================
//  LINE FOLLOWING — PD Controller (5 sensors)
// ============================================================
void followLine() {
  // 5 sensor weights: -2(far left) to +2(far right)
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
    error = (lastError > 0) ? 3 : -3;
  }

  float correction = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (abs(error) >= 2) {
    if (leftSpeed  > 0) leftSpeed  = min(leftSpeed,  SHARP_SPEED);
    if (rightSpeed > 0) rightSpeed = min(rightSpeed, SHARP_SPEED);
  }

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  AVOID RED OBSTACLE
// ============================================================
void avoidRedObstacle() {
  driveMotors(-AVOID_SPEED, AVOID_SPEED); delay(380);
  stopMotors(); delay(150);
  driveMotors(AVOID_SPEED, AVOID_SPEED);  delay(650);
  stopMotors(); delay(150);
  driveMotors(AVOID_SPEED, -AVOID_SPEED); delay(380);
  stopMotors(); delay(150);
  driveMotors(AVOID_SPEED, AVOID_SPEED);  delay(400);
  stopMotors(); delay(200);
}

// ============================================================
//  REJOIN LINE
// ============================================================
bool rejoinLine() {
  unsigned long start = millis();
  while (millis() - start < 4000) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count >= 1) { Serial.println("Line found!"); return true; }
    driveMotors(110, 110);
    delay(40);
  }
  stopMotors();
  return false;
}

// ============================================================
//  PICK UP / DROP CUBE
// ============================================================
void pickGreenCube() {
  gripperServo.write(SERVO_OPEN);  delay(600);
  stopMotors();                    delay(300);
  gripperServo.write(SERVO_CLOSED);delay(800);
  Serial.println("Cube gripped!");
}

void dropCubeAtEnd() {
  stopMotors();                    delay(200);
  gripperServo.write(SERVO_OPEN);  delay(800);
  driveMotors(-100,-100);          delay(300);
  stopMotors();
  cubePickedUp = false;
  Serial.println("Cube released!");
}

// ============================================================
//  TCS230 COLOR SENSOR — clean separate pins on ESP32
// ============================================================
char readColorOnce() {
  long r=0, g=0, b=0;

  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, LOW);
  delay(10);
  r = pulseIn(TCS_OUT_PIN, LOW, 100000);

  digitalWrite(TCS_S2_PIN, HIGH);
  digitalWrite(TCS_S3_PIN, HIGH);
  delay(10);
  g = pulseIn(TCS_OUT_PIN, LOW, 100000);

  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, HIGH);
  delay(10);
  b = pulseIn(TCS_OUT_PIN, LOW, 100000);

  // Uncomment to calibrate:
  /*
  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);
  */

  if (r==0 && g==0 && b==0) return 'U';
  if (min(r,min(g,b)) > 250) return 'U';
  if (r < g && r < b) return 'R';
  if (g < r && g < b) return 'G';
  if (b < r && b < g) return 'B';
  return 'U';
}

char readColorVoted(int times) {
  int cR=0,cG=0,cB=0,cU=0;
  for (int i=0; i<times; i++) {
    char c = readColorOnce();
    if      (c=='R') cR++;
    else if (c=='G') cG++;
    else if (c=='B') cB++;
    else             cU++;
    delay(25);
  }
  Serial.print("Votes R:"); Serial.print(cR);
  Serial.print(" G:"); Serial.print(cG);
  Serial.print(" B:"); Serial.print(cB);
  Serial.print(" U:"); Serial.println(cU);
  if (cR>=cG && cR>=cB && cR>cU) return 'R';
  if (cG>=cR && cG>=cB && cG>cU) return 'G';
  if (cB>=cR && cB>=cG && cB>cU) return 'B';
  return 'U';
}

// ============================================================
//  ULTRASONIC
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  if (dur==0) return 999.0;
  return (dur * 0.0343) / 2.0;
}

// ============================================================
//  MOTOR CONTROL — ledcWrite (not analogWrite!)
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // core v3.x: ledcWrite(pin, value) — use PIN not channel
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, leftSpeed);
  } else {
    digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, -leftSpeed);
  }
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENB_PIN, rightSpeed);
  } else {
    digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH);
    ledcWrite(ENB_PIN, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);
}
// ============================================================
//  END OF CODE v5.1 — ESP32 30-pin DevKit V1
// ============================================================
