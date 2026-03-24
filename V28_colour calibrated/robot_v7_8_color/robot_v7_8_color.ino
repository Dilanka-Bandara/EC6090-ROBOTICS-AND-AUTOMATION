/*
 * ============================================================
 *  EC6090 MINI PROJECT 2026 — LINE FOLLOWING ROBOT v7.8
 *  COLOR CALIBRATION — tuned from measured cube values
 * ============================================================
 *  Board: ESP32 DevKit V1 — 30 pin
 *
 *  Components:
 *    - 5x IR Sensors (TCRT5000 digital) for line following
 *    - 1x HC-SR04 Ultrasonic sensor for obstacle detection
 *    - 1x TCS230/TCS3200 Color sensor for red/green detection
 *    - 1x L298N Motor driver for 2 DC motors
 *    - 1x SG90 Servo for drop-gate pick & place
 *    - ESP32 DevKit V1 (30-pin) controller
 *
 *  Arena: Yellow tape line on grey concrete floor
 *         Closed-loop path with curves (~1.3m diameter sections)
 *         2x Red obstacle cubes ON the line
 *         1x Green cube ON the line (to pick up)
 *         End/Target zone to drop cube
 *
 * ============================================================
 *  CHANGES FROM v7.7 → v7.8
 * ============================================================
 *
 *  COLOR SENSOR CALIBRATED FROM REAL MEASUREMENTS:
 *
 *  TCS230 reminder: LOWER pulse = MORE of that color detected.
 *  The sensor outputs a frequency — shorter pulse = higher
 *  frequency = more reflection of that wavelength.
 *
 *  Measured values (RAW pulseIn readings):
 *
 *  RED CUBE:
 *    R: 273–342  ← LOW  = lots of red light detected
 *    G: 375–440  ← HIGH = little green detected
 *    B: 369–452  ← HIGH = little blue detected
 *    Key signature: R is always the LOWEST channel
 *
 *  GREEN CUBE:
 *    R: 351–455  ← HIGH = little red detected
 *    G: 379–453  ← MID
 *    B: 391–482  ← HIGH
 *    Key signature: R is always the HIGHEST channel
 *    Note: G and B are close — green cube reflects
 *    diffusely. The clearest marker is R being HIGH.
 *
 *  DETECTION LOGIC (direct RAW comparison, no map()):
 *    The old map()-based approach used calibration ranges
 *    that don't match these values. Replaced with direct
 *    RAW pulse comparison which is simpler and more reliable
 *    for these specific cube colors.
 *
 *    RED detection:
 *      R < 350          (R is distinctly low)
 *      AND R < G - 40   (R clearly lower than G)
 *      AND R < B - 40   (R clearly lower than B)
 *
 *    GREEN detection:
 *      R > 350          (R is high = little red)
 *      AND G < R        (G lower than R)
 *      AND B < R        (B lower than R)
 *      AND (G-B) < 60   (G and B are close = no dominant color)
 *
 *    Fallback: if neither threshold met → 'U' (unknown)
 *
 *  VOTES INCREASED: 9 → 15 for more reliability at detection
 *  DELAY BETWEEN READINGS: 20ms → 30ms for stable pulseIn (slower corner entry)
 *
 *  PIN MAP (30-pin DevKit V1) — unchanged from v7.0:
 *
 *    GPIO    Component
 *    ─────────────────────────────────────────
 *    16   →  L298N IN1  (Left  motor dir A)
 *    17   →  L298N IN2  (Left  motor dir B)
 *    18   →  L298N IN3  (Right motor dir A)
 *    19   →  L298N IN4  (Right motor dir B)
 *    25   →  L298N ENA  (Left  motor PWM)
 *    26   →  L298N ENB  (Right motor PWM)
 *    27   →  SG90 Servo signal
 *    22   →  HC-SR04 TRIG
 *    23   →  HC-SR04 ECHO (needs 5V→3.3V divider!)
 *    13   →  TCS230 S0
 *    14   →  TCS230 S1
 *    21   →  TCS230 S2
 *     4   →  TCS230 S3
 *    33   →  TCS230 OUT
 *    35   →  IR Sensor S1 (Far  Left)  — input only
 *    34   →  IR Sensor S2 (Mid  Left)  — input only
 *    39   →  IR Sensor S3 (Center)     — input only (VN)
 *    36   →  IR Sensor S4 (Mid  Right) — input only (VP)
 *    32   →  IR Sensor S5 (Far  Right)
 *
 *  ⚠️  GPIO 2, 5, 12, 15 avoided (strapping pins)
 *  ⚠️  HC-SR04 ECHO needs voltage divider: ECHO→1kΩ→GPIO23, 2kΩ→GND
 *  ⚠️  IR sensors → 3.3V VCC only
 *  ⚠️  Servo, L298N → 5V VCC (Vin pin or external)
 *
 *  Required library: ESP32Servo (install via Library Manager)
 *  Arduino IDE: Tools → Board → ESP32 Arduino → ESP32 Dev Module
 *
 *  Author: EC6090 Mini Project 2026 — v7.8 Color Calibration
 * ============================================================
 */

#include <ESP32Servo.h>

// ============================================================
//  DEBUG / TEST FLAGS
// ============================================================
#define TEST_MOTORS       false
#define TEST_IR_RAW       false
#define TEST_COLOR        true
#define TEST_ULTRASONIC   false
#define TEST_SERVO        false
#define CALIBRATE_COLOR   false
#define PRINT_DEBUG       true

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// IR Sensors
#define IR_S1   35    // Far  Left  (input-only)
#define IR_S2   34    // Mid  Left  (input-only)
#define IR_S3   39    // Center     (input-only VN)
#define IR_S4   36    // Mid  Right (input-only VP)
#define IR_S5   32    // Far  Right

// HC-SR04
#define TRIG_PIN  22
#define ECHO_PIN  23

// L298N
#define IN1_PIN   16
#define IN2_PIN   17
#define IN3_PIN   18
#define IN4_PIN   19
#define ENA_PIN   25
#define ENB_PIN   26

// Servo
#define SERVO_PIN 27

// TCS230
#define TCS_S0   13
#define TCS_S1   14
#define TCS_S2   21
#define TCS_S3    4
#define TCS_OUT  33

// ============================================================
//  LEDC PWM
// ============================================================
#define PWM_FREQ        5000
#define PWM_RESOLUTION  8

// ============================================================
//  TUNABLE CONSTANTS
// ============================================================

// Motor speeds
#define MAX_SPEED             200   // full speed on straights
#define BASE_SPEED            150   // normal line following
#define CURVE_SPEED            80   // moderate curves
#define SHARP_TURN_SPEED      200   // pivot speed on tight corners
#define CORNER_APPROACH_SPEED 100   // NEW: slow when early-warning pattern detected
#define REACQUIRE_SPEED        80   // NEW: gentle speed after pivot before full PID
#define BACKUP_SPEED           80   // NEW: reverse speed for missed-corner reposition
#define MIN_SPEED              80   // stall threshold
#define AVOID_SPEED           130   // obstacle avoidance
#define CREEP_SPEED            90   // approach green cube
#define SPIN_SPEED            120   // lost-line spin

// Corner timing
#define REACQUIRE_MS   150   // ms to drive slowly after pivot exit before full PID
#define BACKUP_MS       80   // ms to reverse before missed-corner pivot

// PID
float Kp = 3.2;
float Ki = 0.01;
float Kd = 4.5;
float integralSum = 0.0;
#define INTEGRAL_MAX  500.0

// Missed corner threshold (Fix 1)
// If |lastError| >= this AND all sensors dark → missed corner pivot
// Range is ±40 (5-sensor weights). 30 = only triggers on genuinely
// strong off-center reading, not a small wobble on a straight.
#define MISSED_CORNER_THRESHOLD  30

// Pivot hard timeout (Fix 2)
// If the center sensor never reacquires after this long, force-exit
// the pivot so the robot doesn't spin forever.
#define PIVOT_TIMEOUT_MS  3000

// Ultrasonic
#define DETECT_DIST_CM    20
#define END_WALL_DIST_CM   8

// Servo
#define SERVO_OPEN    85
#define SERVO_CLOSED  15

// Timing
#define END_ZONE_TIMEOUT_MS   15000
#define STUCK_TIMEOUT_MS      15000
#define LINE_LOSS_CONFIRM_MS    300

// IR polarity
#define INVERT_IR  true   // LOW from sensor = on line (most TCRT5000 modules)

// Color detection thresholds — tuned from measured cube RAW values
// TCS230: lower pulseIn value = MORE of that color detected
//
// RED cube measured:   R=273–342, G=375–440, B=369–452
// GREEN cube measured: R=351–455, G=379–453, B=391–482
//
#define COLOR_RED_R_MAX    350   // R below this = red cube (R channel is low/active)
#define COLOR_RED_MARGIN    40   // R must be this much lower than G and B
#define COLOR_GREEN_R_MIN  350   // R above this = green cube (R channel is high/inactive)
#define COLOR_GB_DIFF_MAX   70   // G and B must be within this range of each other

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
Servo gateServo;

int irRaw[5];
int irBinary[5];
int irPins[5] = {IR_S1, IR_S2, IR_S3, IR_S4, IR_S5};

int   lastError        = 0;
int   lastTightDir     = 0;
bool  cubePickedUp     = false;
int   obstaclesAvoided = 0;
unsigned long pickUpTime     = 0;
unsigned long stateEntryTime = 0;
unsigned long lastDebugPrint = 0;
unsigned long lineLostStart  = 0;
bool  lineLostTiming   = false;
float lastDistCm       = 999.0;  // cached ultrasonic — avoids blocking in dashboard

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
RobotState prevState    = STATE_LINE_FOLLOW;

const char* stateNames[] = {
  "LINE_FOLLOW", "IDENTIFY_OBJ", "AVOID_RED",
  "REJOIN_LINE", "APPROACH_GREEN", "PICK_GREEN",
  "CARRY_TO_END", "DROP_CUBE", "DONE"
};

// ============================================================
//  FUNCTION DECLARATIONS
// ============================================================
void  readIRSensors();
void  followLine();
void  followLineAdaptive();
void  avoidObstacle();
bool  rejoinLine();
void  pickGreenCube();
void  dropCubeAtEnd();
char  readColorOnce();
char  readColorVoted(int times);
float readUltrasonic();
void  driveMotors(int leftSpeed, int rightSpeed);
void  stopMotors();
void  changeState(RobotState newState);
void  printDashboard();
void  runTestMode();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("============================================");
  Serial.println("  EC6090 Robot v7.8 — Color Calibration");
  Serial.println("============================================");
  Serial.print("INVERT_IR=");
  Serial.println(INVERT_IR ? "YES (LOW=line)" : "NO (HIGH=line)");
  Serial.print("BASE_SPEED="); Serial.println(BASE_SPEED);
  Serial.print("MISSED_CORNER_THRESHOLD="); Serial.println(MISSED_CORNER_THRESHOLD);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RESOLUTION);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  for (int i = 0; i < 5; i++) {
    pinMode(irPins[i], INPUT);
  }

  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(SERVO_OPEN);
  delay(500);

  if (TEST_MOTORS || TEST_IR_RAW || TEST_COLOR ||
      TEST_ULTRASONIC || TEST_SERVO || CALIBRATE_COLOR) {
    runTestMode();
  }

  Serial.println("Place robot on LINE — starting in 3 seconds...");
  delay(3000);
  stateEntryTime = millis();
  Serial.println("GO!");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  readIRSensors();

  // Watchdog
  if (currentState != STATE_DONE) {
    if (millis() - stateEntryTime > STUCK_TIMEOUT_MS &&
        currentState == prevState) {
      Serial.println("⚠ WATCHDOG: Stuck — recovery spin");
      stopMotors();
      delay(200);
      if (lastError >= 0) driveMotors(SPIN_SPEED, -SPIN_SPEED);
      else                driveMotors(-SPIN_SPEED, SPIN_SPEED);
      delay(600);
      stopMotors();
      stateEntryTime = millis();
      if (currentState != STATE_LINE_FOLLOW &&
          currentState != STATE_CARRY_TO_END) {
        changeState(STATE_REJOIN_LINE);
      }
    }
  }
  prevState = currentState;

  if (PRINT_DEBUG && millis() - lastDebugPrint > 20) {
    printDashboard();
    lastDebugPrint = millis();
  }

  switch (currentState) {

    case STATE_LINE_FOLLOW: {
      float dist = readUltrasonic();
      lastDistCm = dist;   // cache for dashboard
      if (dist > 2 && dist < DETECT_DIST_CM) {
        stopMotors();
        delay(200);
        Serial.print("Object at "); Serial.print(dist, 1);
        Serial.println(" cm — checking color...");
        changeState(STATE_IDENTIFY_OBJECT);
        break;
      }
      followLineAdaptive();
      break;
    }

    case STATE_IDENTIFY_OBJECT: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(150);
      stopMotors();
      delay(200);

      char color = readColorVoted(15);
      Serial.print("Color result: "); Serial.println(color);

      if (color == 'R') {
        Serial.println("→ RED obstacle — avoiding");
        changeState(STATE_AVOID_RED);
      } else if (color == 'G' && !cubePickedUp) {
        Serial.println("→ GREEN cube — picking up");
        changeState(STATE_APPROACH_GREEN);
      } else {
        Serial.println("→ Unknown — treating as obstacle");
        changeState(STATE_AVOID_RED);
      }
      break;
    }

    case STATE_AVOID_RED: {
      avoidObstacle();
      obstaclesAvoided++;
      Serial.print("Obstacles avoided: "); Serial.println(obstaclesAvoided);
      changeState(STATE_REJOIN_LINE);
      break;
    }

    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) {
        changeState(cubePickedUp ? STATE_CARRY_TO_END : STATE_LINE_FOLLOW);
      } else {
        Serial.println("Rejoin timeout — spinning to recover...");
        if (lastError >= 0) driveMotors(SPIN_SPEED, -SPIN_SPEED);
        else                driveMotors(-SPIN_SPEED, SPIN_SPEED);
        delay(500);
        stopMotors();
      }
      break;
    }

    case STATE_APPROACH_GREEN: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(500);
      stopMotors();
      delay(200);
      changeState(STATE_PICK_GREEN);
      break;
    }

    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      changeState(STATE_CARRY_TO_END);
      break;
    }

    case STATE_CARRY_TO_END: {
      if (millis() - pickUpTime > END_ZONE_TIMEOUT_MS) {
        stopMotors();
        Serial.println("End zone — timeout");
        changeState(STATE_DROP_CUBE);
        break;
      }

      float dist = readUltrasonic();
      lastDistCm = dist;   // cache for dashboard
      if (dist > 0 && dist < END_WALL_DIST_CM) {
        stopMotors();
        Serial.println("End zone — wall detected");
        changeState(STATE_DROP_CUBE);
        break;
      }

      if (millis() - pickUpTime > 3000) {
        int sc = 0;
        for (int i = 0; i < 5; i++) sc += irBinary[i];
        if (sc == 0) {
          delay(LINE_LOSS_CONFIRM_MS);
          readIRSensors();
          sc = 0;
          for (int i = 0; i < 5; i++) sc += irBinary[i];
          if (sc == 0) {
            stopMotors();
            Serial.println("End zone — line terminated");
            changeState(STATE_DROP_CUBE);
            break;
          }
        }
      }

      if (dist > 2 && dist < DETECT_DIST_CM) {
        stopMotors();
        delay(200);
        char color = readColorVoted(11);
        if (color == 'R' || color == 'U') {
          Serial.println("Obstacle while carrying — avoiding");
          avoidObstacle();
          obstaclesAvoided++;
          changeState(STATE_REJOIN_LINE);
          break;
        }
      }

      followLineAdaptive();
      break;
    }

    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      changeState(STATE_DONE);
      break;
    }

    case STATE_DONE: {
      stopMotors();
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 3000) {
        Serial.println("★★★ TASK COMPLETE ★★★");
        Serial.print("Obstacles avoided: "); Serial.println(obstaclesAvoided);
        Serial.print("Total time: ");
        Serial.print((millis() - 3000) / 1000.0, 1);
        Serial.println(" seconds");
        lastPrint = millis();
      }
      break;
    }
  }
}

// ============================================================
//  STATE CHANGE HELPER
// ============================================================
void changeState(RobotState newState) {
  Serial.print("State: ");
  Serial.print(stateNames[currentState]);
  Serial.print(" → ");
  Serial.println(stateNames[newState]);
  currentState   = newState;
  stateEntryTime = millis();
  integralSum    = 0.0;
}

// ============================================================
//  READ IR SENSORS
// ============================================================
void readIRSensors() {
  for (int i = 0; i < 5; i++) {
    int raw = digitalRead(irPins[i]);
    irRaw[i] = raw;
    if (INVERT_IR) {
      irBinary[i] = (raw == LOW) ? 1 : 0;
    } else {
      irBinary[i] = (raw == HIGH) ? 1 : 0;
    }
  }
}

// ============================================================
//  LINE FOLLOWING — PID + Tight Turn + Missed Corner (v7.1)
//
//  Sensor layout (left to right):
//    Index:  0    1    2    3    4
//    Name:  S1   S2   S3   S4   S5
//    Weight:-40  -20   0   20   40
//
//  NORMAL TIGHT TURN TRIGGER (unchanged from v7.0):
//    LEFT  tight: S1=1, S2=0, S3=0  → pattern 10000
//    RIGHT tight: S5=1, S4=0, S3=0  → pattern 00001
//
//  NEW — MISSED CORNER TRIGGER (Fix 1):
//    Fires when ALL sensors go dark (00000) AND the last PID
//    error was strongly to one side (|lastError|>=30).
//    This catches the case where the robot was going fast and
//    the sensor array passed over the corner tip before the
//    10000/00001 pattern could register.
//
//  PIVOT EXIT (Fix 3 — widened from S2||S3 to S2||S3||S4):
//    Accepts any of the 3 inner sensors as reacquire confirmation.
//    Prevents over-rotation when reacquiring after a missed corner.
//
//  PIVOT HARD TIMEOUT (Fix 2):
//    If the pivot runs for more than PIVOT_TIMEOUT_MS (3000ms)
//    without reacquiring the line, force-exit to PID.
//    Prevents infinite spinning if something goes wrong.
// ============================================================

bool  turning           = false;
int   turnDirection     = 0;
unsigned long turnStartTime   = 0;

// Phase 3 reacquire state
bool  reacquiring       = false;
unsigned long reacquireStart  = 0;

// Corner approach state (Phase 1 early warning)
bool  cornerApproach    = false;

void followLineAdaptive() {

  bool s1 = irBinary[0];  // Far  Left
  bool s2 = irBinary[1];  // Mid  Left
  bool s3 = irBinary[2];  // Center
  bool s4 = irBinary[3];  // Mid  Right
  bool s5 = irBinary[4];  // Far  Right

  int sensorSum = irBinary[0]+irBinary[1]+irBinary[2]
                 +irBinary[3]+irBinary[4];

  // ══════════════════════════════════════════════════════════════
  //  PHASE 3 — CONTROLLED REACQUIRE
  //  After pivot exit, drive slowly for REACQUIRE_MS before
  //  handing back to full PID. Prevents momentum overshoot on
  //  the new line direction.
  // ══════════════════════════════════════════════════════════════
  if (reacquiring) {
    if (millis() - reacquireStart < REACQUIRE_MS) {
      driveMotors(REACQUIRE_SPEED, REACQUIRE_SPEED);
      return;
    }
    // Reacquire period done → hand back to PID
    reacquiring    = false;
    cornerApproach = false;
    integralSum    = 0.0;
    Serial.println("REACQUIRE DONE → full PID");
  }

  // ══════════════════════════════════════════════════════════════
  //  PHASE 2 — PIVOT
  //  Full counter-rotation pivot. Returns immediately — PID and
  //  lost-line blocks below are never reached while pivoting.
  // ══════════════════════════════════════════════════════════════
  if (turning) {

    if (turnDirection == -1) {
      driveMotors(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);  // pivot LEFT
    } else {
      driveMotors(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);  // pivot RIGHT
    }

    // Guard: 30ms only — just enough to prevent false exit from
    // the entry sensor flash. Pivot continues until S3 (center)
    // detects the line — no fixed time limit on the turn itself.
    if (millis() - turnStartTime > 30) {
      readIRSensors();
      // EXIT ONLY when S3 (center/middle sensor) sees the line.
      // S2 and S4 are intentionally excluded — they fire too early
      // on the edge of the line and would stop the pivot before the
      // robot is properly aligned. S3 center = robot is centred.
      if (irBinary[2] == 1) {
        stopMotors();
        turning        = false;
        reacquiring    = true;
        reacquireStart = millis();
        lastError      = 0;
        integralSum    = 0.0;
        Serial.println("CENTER DETECTED → motors stop → PID resume");
      }
    }

    // Hard timeout — force exit if line never reacquired
    if (millis() - turnStartTime > PIVOT_TIMEOUT_MS) {
      turning      = false;
      reacquiring  = false;
      cornerApproach = false;
      lastError    = 0;
      stopMotors();
      Serial.println("PIVOT TIMEOUT → forcing PID resume");
    }

    return;  // PID completely OFF during pivot
  }

  // ══════════════════════════════════════════════════════════════
  //  CORNER TRIGGER — check for Phase 1 early warning AND
  //  Phase 2 pivot trigger before entering PID
  // ══════════════════════════════════════════════════════════════
  if (!turning && !reacquiring) {

    // ── PHASE 1: EARLY WARNING ─────────────────────────────────
    // Pattern S1+S2 both on (11000) or S4+S5 both on (00011)
    // with center off → corner is approaching.
    // Slow down immediately so array has more time over corner tip.
    bool earlyLeft  = (s1 && s2 && !s3);
    bool earlyRight = (s4 && s5 && !s3);

    if (earlyLeft || earlyRight) {
      cornerApproach = true;
    } else if (!s1 && !s5) {
      // Back on straight — clear early warning flag
      cornerApproach = false;
    }

    // ── PHASE 2 TRIGGER: DECISIVE PIVOT ────────────────────────
    //
    // 45° corner patterns (far sensor alone):
    //   tightLeft  = 10000  (S1 only, middle+center off)
    //   tightRight = 00001  (S5 only, middle+center off)
    //
    // 90° corner patterns (three sensors on, center included):
    //   90Left  = 11100  (S1+S2+S3 all on, S4+S5 off)
    //   90Right = 00111  (S3+S4+S5 all on, S1+S2 off)
    //
    // Both cases trigger the same pivot behaviour:
    //   stop → pivot → wait for S3 center alone → stop → PID
    //
    bool tightLeft  = (s1 && !s2 && !s3);                    // 45° left:  10000
    bool tightRight = (s5 && !s4 && !s3);                    // 45° right: 00001
    bool sharp90Left  = (s1 && s2 && s3 && s4 && !s5);       // 90° left:  11110  ← measured
    bool sharp90Right = (!s1 && s2 && s3 && s4 && s5);       // 90° right: 01111  ← measured

    // MISSED CORNER: all sensors dark + last error was strong
    bool missedLeft  = (sensorSum == 0 &&
                        lastError <= -MISSED_CORNER_THRESHOLD);
    bool missedRight = (sensorSum == 0 &&
                        lastError >=  MISSED_CORNER_THRESHOLD);

    if (tightLeft || sharp90Left || missedLeft) {
      if (missedLeft) {
        Serial.println("MISSED CORNER LEFT-SENSOR → backup + pivot RIGHT");
        driveMotors(-BACKUP_SPEED, -BACKUP_SPEED);
        delay(BACKUP_MS);
        stopMotors();
      } else if (sharp90Left) {
        Serial.println("90-DEG LEFT-SENSOR (11110) → pivot RIGHT");
        stopMotors();
      } else {
        Serial.println("45-DEG LEFT-SENSOR (10000) → pivot RIGHT");
        stopMotors();
      }
      turning        = true;
      turnDirection  = 1;    // S1 physically on RIGHT side → pivot RIGHT
      lastTightDir   = 1;
      cornerApproach = false;
      integralSum    = 0.0;
      turnStartTime  = millis();
    }
    else if (tightRight || sharp90Right || missedRight) {
      if (missedRight) {
        Serial.println("MISSED CORNER RIGHT-SENSOR → backup + pivot LEFT");
        driveMotors(-BACKUP_SPEED, -BACKUP_SPEED);
        delay(BACKUP_MS);
        stopMotors();
      } else if (sharp90Right) {
        Serial.println("90-DEG RIGHT-SENSOR (01111) → pivot LEFT");
        stopMotors();
      } else {
        Serial.println("45-DEG RIGHT-SENSOR (00001) → pivot LEFT");
        stopMotors();
      }
      turning        = true;
      turnDirection  = -1;   // S5 physically on LEFT side → pivot LEFT
      lastTightDir   = -1;
      cornerApproach = false;
      integralSum    = 0.0;
      turnStartTime  = millis();
    }
  }

  // If pivot just triggered, return — motors already commanded
  if (turning) return;

  // ══════════════════════════════════════════════════════════════
  //  LOST-LINE RECOVERY
  //  Only reached when not turning and not reacquiring.
  //  sensorSum==0 here is genuine line loss, not a corner flyover
  //  (corner flyover is caught above by missedLeft/missedRight).
  // ══════════════════════════════════════════════════════════════
  if (sensorSum == 0) {
    if (!lineLostTiming) {
      lineLostStart  = millis();
      lineLostTiming = true;
    }

    if (millis() - lineLostStart < 200) {
      if (lastError > 0)       driveMotors(SPIN_SPEED, MIN_SPEED);
      else if (lastError < 0)  driveMotors(MIN_SPEED, SPIN_SPEED);
      else                     driveMotors(BASE_SPEED, BASE_SPEED);
      return;
    }

    int spinDir = (lastTightDir != 0) ? lastTightDir
                                      : (lastError >= 0 ? 1 : -1);
    if (spinDir > 0) driveMotors(SPIN_SPEED, -SPIN_SPEED);
    else             driveMotors(-SPIN_SPEED, SPIN_SPEED);
    return;
  }

  lineLostTiming = false;

  // ══════════════════════════════════════════════════════════════
  //  NORMAL PID LINE FOLLOWING
  //  Weights: S1=-40, S2=-20, S3=0, S4=20, S5=40
  // ══════════════════════════════════════════════════════════════
  int weights[5] = {-40, -20, 0, 20, 40};
  int weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
  }

  int error = weightedSum / sensorSum;

  integralSum += (float)error;
  integralSum  = constrain(integralSum, -INTEGRAL_MAX, INTEGRAL_MAX);
  float derivative = (float)(error - lastError);
  float correction = (Kp * (float)error) + (Ki * integralSum) + (Kd * derivative);
  lastError = error;

  // Adaptive speed:
  // cornerApproach flag active → force CORNER_APPROACH_SPEED
  // Otherwise use standard adaptive speed tiers
  int absError = abs(error);
  int baseSpd;

  if (cornerApproach) {
    // Phase 1 active — slow right down before corner
    baseSpd = CORNER_APPROACH_SPEED;
  } else if (absError <= 5) {
    baseSpd = MAX_SPEED;   // straight — full speed
  } else if (absError <= 20) {
    baseSpd = BASE_SPEED;  // gentle curve
  } else {
    baseSpd = CURVE_SPEED; // tighter curve
  }

  int leftSpeed  = constrain(baseSpd - (int)correction, -120, 255);
  int rightSpeed = constrain(baseSpd + (int)correction, -120, 255);

  driveMotors(leftSpeed, rightSpeed);
}

void followLine() {
  followLineAdaptive();
}

// ============================================================
//  AVOID OBSTACLE
// ============================================================
void avoidObstacle() {
  Serial.println("── Avoiding obstacle ──");

  readIRSensors();
  bool lineOnLeft  = (irBinary[0] == 1 || irBinary[1] == 1);
  bool lineOnRight = (irBinary[3] == 1 || irBinary[4] == 1);

  bool goRight = true;
  if (lineOnLeft && !lineOnRight) {
    goRight = false;
    Serial.println("  Direction: LEFT bypass");
  } else {
    Serial.println("  Direction: RIGHT bypass");
  }

  int turnSign = goRight ? 1 : -1;

  driveMotors(turnSign * AVOID_SPEED, -turnSign * AVOID_SPEED);
  delay(400);
  stopMotors(); delay(100);

  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(700);
  stopMotors(); delay(100);

  driveMotors(-turnSign * AVOID_SPEED, turnSign * AVOID_SPEED);
  delay(400);
  stopMotors(); delay(100);

  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(450);
  stopMotors(); delay(150);

  Serial.println("  Avoidance complete");
}

// ============================================================
//  REJOIN LINE
// ============================================================
bool rejoinLine() {
  Serial.println("Searching for line...");
  unsigned long start  = millis();
  int           sweepDir = (lastError >= 0) ? 1 : -1;

  while (millis() - start < 5000) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];

    if (count >= 1) {
      Serial.println("Line found!");
      return true;
    }

    unsigned long elapsed = millis() - start;
    if (elapsed < 1500) {
      driveMotors(90 + sweepDir * 30, 90 - sweepDir * 30);
    } else if (elapsed < 3000) {
      driveMotors(90 - sweepDir * 30, 90 + sweepDir * 30);
    } else {
      driveMotors(sweepDir * 100, -sweepDir * 100);
    }
    delay(30);
  }

  stopMotors();
  Serial.println("Line NOT found — timeout");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
// ============================================================
void pickGreenCube() {
  Serial.println("── Picking green cube ──");
  stopMotors(); delay(200);
  gateServo.write(SERVO_OPEN); delay(400);
  driveMotors(CREEP_SPEED, CREEP_SPEED); delay(300);
  stopMotors(); delay(200);
  gateServo.write(SERVO_CLOSED); delay(800);
  Serial.println("Cube gripped!");
}

// ============================================================
//  DROP CUBE AT END ZONE
// ============================================================
void dropCubeAtEnd() {
  Serial.println("── Dropping cube ──");
  stopMotors(); delay(300);
  gateServo.write(SERVO_OPEN); delay(1000);
  driveMotors(-100, -100); delay(500);
  stopMotors();
  cubePickedUp = false;
  Serial.println("Cube released!");
}

// ============================================================
//  COLOR SENSOR — single reading (v7.8 direct RAW comparison)
//
//  No map() used — direct pulse comparison against measured
//  thresholds. More reliable for these specific cube colors.
//
//  RED cube:   R is the lowest channel (strong red reflection)
//  GREEN cube: R is the highest channel (weak red reflection)
//              G and B are close to each other
// ============================================================
char readColorOnce() {
  long r = 0, g = 0, b = 0;

  // Read Red channel (S2=LOW, S3=LOW)
  digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, LOW);
  delay(15);
  r = pulseIn(TCS_OUT, LOW, 100000);

  // Read Green channel (S2=HIGH, S3=HIGH)
  digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH);
  delay(15);
  g = pulseIn(TCS_OUT, LOW, 100000);

  // Read Blue channel (S2=LOW, S3=HIGH)
  digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, HIGH);
  delay(15);
  b = pulseIn(TCS_OUT, LOW, 100000);

  // Timeout — any channel returned 0 means no pulse detected
  if (r == 0 || g == 0 || b == 0) return 'U';

  // Print raw values for debugging (comment out after tuning)
  Serial.print("  RAW R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);

  // ── RED DETECTION ───────────────────────────────────────────
  // R channel is lowest (most red light detected)
  // R must be below threshold AND clearly lower than G and B
  if (r < COLOR_RED_R_MAX &&
      r < (g - COLOR_RED_MARGIN) &&
      r < (b - COLOR_RED_MARGIN)) {
    return 'R';
  }

  // ── GREEN DETECTION ─────────────────────────────────────────
  // R channel is highest (least red light detected = green/other)
  // G and B must be close to each other (diffuse reflection)
  if (r > COLOR_GREEN_R_MIN &&
      r > g &&
      r > b &&
      abs(g - b) < COLOR_GB_DIFF_MAX) {
    return 'G';
  }

  return 'U';
}

// ============================================================
//  COLOR SENSOR — voted reading
// ============================================================
char readColorVoted(int times) {
  int cR = 0, cG = 0, cB = 0, cU = 0;

  for (int i = 0; i < times; i++) {
    char c = readColorOnce();
    switch (c) {
      case 'R': cR++; break;
      case 'G': cG++; break;
      case 'B': cB++; break;
      default:  cU++; break;
    }
    delay(30);
  }

  Serial.print("Color votes — R:"); Serial.print(cR);
  Serial.print(" G:"); Serial.print(cG);
  Serial.print(" B:"); Serial.print(cB);
  Serial.print(" U:"); Serial.println(cU);

  if (cR >= cG && cR >= cB && cR > cU) return 'R';
  if (cG >= cR && cG >= cB && cG > cU) return 'G';
  if (cB >= cR && cB >= cG && cB > cU) return 'B';
  return 'U';
}

// ============================================================
//  ULTRASONIC
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
//  MOTOR CONTROL
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, constrain(-leftSpeed, 0, 255));
  }
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENB_PIN, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH);
    ledcWrite(ENB_PIN, constrain(-rightSpeed, 0, 255));
  }
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);
}

// ============================================================
//  DEBUG DASHBOARD
// ============================================================
void printDashboard() {
  Serial.print("["); Serial.print(stateNames[currentState]); Serial.print("]");
  Serial.print(" IR:");
  for (int i = 0; i < 5; i++) Serial.print(irBinary[i]);
  Serial.print(" err:"); Serial.print(lastError);
  // Show current corner phase
  if (turning)        Serial.print(turnDirection < 0 ? " [PIVOT-L]" : " [PIVOT-R]");
  else if (reacquiring) Serial.print(" [REACQ]");
  else if (cornerApproach) Serial.print(" [SLOW]");
  else                Serial.print(" [PID]");
  Serial.print(" dist:"); Serial.print(lastDistCm, 1);
  Serial.print(" obs:"); Serial.print(obstaclesAvoided);
  Serial.print(" cube:"); Serial.println(cubePickedUp ? "YES" : "NO");
}

// ============================================================
//  TEST MODES
// ============================================================
void runTestMode() {
  Serial.println("╔══════════════════════════════╗");
  Serial.println("║     TEST MODE ACTIVE         ║");
  Serial.println("╚══════════════════════════════╝");

  if (TEST_MOTORS) {
    Serial.println("=== MOTOR TEST ===");
    while (true) {
      Serial.println("Forward..."); driveMotors(150, 150); delay(2000);
      stopMotors(); delay(500);
      Serial.println("Spin LEFT..."); driveMotors(-150, 150); delay(1500);
      stopMotors(); delay(500);
      Serial.println("Spin RIGHT..."); driveMotors(150, -150); delay(1500);
      stopMotors(); delay(500);
      Serial.println("Reverse..."); driveMotors(-150, -150); delay(2000);
      stopMotors(); delay(1000);
    }
  }

  if (TEST_IR_RAW) {
    Serial.println("=== IR SENSOR TEST ===");
    Serial.println("Labels: S1(fL) S2(mL) S3(C) S4(mR) S5(fR)");
    while (true) {
      Serial.print("RAW: ");
      for (int i = 0; i < 5; i++) {
        Serial.print("S"); Serial.print(i+1);
        Serial.print("="); Serial.print(digitalRead(irPins[i]));
        Serial.print("  ");
      }
      Serial.print("  BIN: ");
      readIRSensors();
      for (int i = 0; i < 5; i++) Serial.print(irBinary[i]);
      Serial.println();
      delay(300);
    }
  }

  if (TEST_COLOR) {
    Serial.println("=== COLOR SENSOR TEST ===");
    Serial.println("Point at cube and watch RAW values + color result.");
    while (true) {
      char color = readColorOnce();  // reads and prints RAW, returns R/G/U
      Serial.print("→ Detected: "); Serial.println(color);
      delay(500);
    }
  }

  if (TEST_ULTRASONIC) {
    Serial.println("=== ULTRASONIC TEST ===");
    while (true) {
      float d = readUltrasonic();
      Serial.print("Distance: "); Serial.print(d, 1); Serial.println(" cm");
      delay(200);
    }
  }

  if (TEST_SERVO) {
    Serial.println("=== SERVO TEST ===");
    while (true) {
      Serial.print("OPEN ("); Serial.print(SERVO_OPEN); Serial.println("°)");
      gateServo.write(SERVO_OPEN); delay(2000);
      Serial.print("CLOSED ("); Serial.print(SERVO_CLOSED); Serial.println("°)");
      gateServo.write(SERVO_CLOSED); delay(2000);
    }
  }

  if (CALIBRATE_COLOR) {
    Serial.println("=== COLOR CALIBRATION ===");
    Serial.println("Place WHITE first, then each cube. Note min/max values.");
    while (true) {
      long r, g, b;
      digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, LOW);  delay(12);
      r = pulseIn(TCS_OUT, LOW, 100000);
      digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH); delay(12);
      g = pulseIn(TCS_OUT, LOW, 100000);
      digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, HIGH); delay(12);
      b = pulseIn(TCS_OUT, LOW, 100000);
      Serial.print("RAW → R="); Serial.print(r);
      Serial.print(" G="); Serial.print(g);
      Serial.print(" B="); Serial.println(b);
      delay(1000);
    }
  }
}

// ============================================================
//  END OF CODE v7.8 — EC6090 Mini Project 2026 — Color Calibration
// ============================================================
