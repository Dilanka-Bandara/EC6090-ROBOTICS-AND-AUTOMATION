/*
 * ============================================================
 *  EC6090 MINI PROJECT 2026 — LINE FOLLOWING ROBOT v6.8
 *  TUNE — MORE TURN ANGLE ON TIGHT CORNERS
 * ============================================================
 *  Board: ESP32 DevKit V1 — 30 pin
 *  
 *  Components:
 *    - 5x IR Sensors (TCRT5000 analog) for line following
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
 *  NEW in v6.2 — TIGHT TURN UPGRADES:
 *
 *  ► SHARP TURN DETECTION: irBinary pattern matching detects 90°
 *    and 45° corners BEFORE PID error gets large. Triggers a
 *    dedicated pivot routine instead of relying on PID alone.
 *
 *  ► COUNTER-ROTATION ON SHARP CORNERS: When far-side sensor fires
 *    alone (S1 only or S5 only), robot PIVOTS (one wheel reverse)
 *    instead of just slowing one wheel. Much faster turn execution.
 *
 *  ► TIGHTER ERROR WEIGHTS: Changed from ±20/±10/0 to ±40/±20/0
 *    so PID sees a bigger error signal earlier on corners.
 *    This gives faster correction before the robot drifts off.
 *
 *  ► LOWER CURVE SPEED: CURVE_SPEED reduced 100→70 for tight turns.
 *    SHARP_TURN_SPEED added (55) for S1/S5-only pivot moves.
 *
 *  ► AGGRESSIVE KP/KD: Kp raised 1.8→3.2, Kd raised 2.0→4.5.
 *    Tested to handle 90° without oscillation on straights because
 *    CURVE_SPEED is also reduced to keep the correction proportional.
 *
 *  ► CORNER HOLD: After a sharp-turn pivot, robot holds the turn
 *    until a center-biased sensor (S2/S3/S4) sees the line again.
 *    Prevents premature straightening mid-corner.
 *
 *  ► INTEGRAL RESET ON SHARP TURNS: integralSum flushed to zero
 *    whenever a sharp-corner pattern is detected so accumulated
 *    error from the straight doesn't fight the corner correction.
 *
 *  CHANGES from v5.5 → v6.1:
 *
 *  1. DIGITAL IR SENSORS: Uses digitalRead() for 3-pin IR modules.
 *     Potentiometer on each module sets the detection threshold.
 *     Calibrate pots so LED ON = yellow tape, LED OFF = grey floor.
 *     ► If your sensors output HIGH on line, set INVERT_IR to false.
 *
 *  2. IMPROVED PID: Added integral term with anti-windup for
 *     smoother curve tracking. Tuned for the ~1.3m diameter
 *     curves in the arena.
 *
 *  3. ADAPTIVE SPEED: Slows down on curves (high error) and
 *     speeds up on straights (low error). Faster completion time.
 *
 *  4. COLOR SENSOR CALIBRATION: Added white/black reference
 *     calibration using map() for proper RGB output. Much more
 *     accurate red vs green detection.
 *
 *  5. SMARTER OBSTACLE AVOIDANCE: Uses proportional turn timing
 *     based on ultrasonic distance. Tries LEFT then RIGHT bypass
 *     depending on which outer sensors see the line.
 *
 *  6. ROBUST LINE REJOIN: Arc-sweep pattern instead of just
 *     creeping forward. Alternates left/right arcs to find line.
 *
 *  7. END ZONE DETECTION: Triple-confirm approach — line loss +
 *     timer after green pick + ultrasonic wall. More reliable.
 *
 *  8. DEBUG MODES: Compile-time flags for testing individual
 *     subsystems (motors, IR, color, ultrasonic, servo).
 *
 *  9. SERIAL DASHBOARD: Real-time state, sensor, and motor
 *     values printed at 10Hz for easy debugging.
 *
 *  10. WATCHDOG: If robot is stuck (no state change) for 15s,
 *      attempts recovery spin.
 *
 *  PIN MAP (30-pin DevKit V1):
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
 *  Author: EC6090 Mini Project 2026 — v6.2 Tight Turn Edition
 * ============================================================
 */

#include <ESP32Servo.h>

// ============================================================
//  DEBUG / TEST FLAGS — set ONE to true to test that subsystem
//  Set ALL to false for normal autonomous operation
// ============================================================
#define TEST_MOTORS       false   // test motor directions & PWM
#define TEST_IR_RAW       false   // print raw IR values continuously
#define TEST_COLOR        false   // print color sensor values
#define TEST_ULTRASONIC   false   // print distance continuously
#define TEST_SERVO        false   // sweep servo back and forth
#define CALIBRATE_COLOR   false   // run color calibration routine
#define PRINT_DEBUG       true    // print state/sensor dashboard at 10Hz

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// IR Sensors — front of robot: S1=Far Left ... S5=Far Right
#define IR_S1   35    // Far  Left  (input-only pin)
#define IR_S2   34    // Mid  Left  (input-only pin)
#define IR_S3   39    // Center     (input-only, VN)
#define IR_S4   36    // Mid  Right (input-only, VP)
#define IR_S5   32    // Far  Right

// HC-SR04 Ultrasonic
#define TRIG_PIN  22
#define ECHO_PIN  23

// L298N Motor Driver
#define IN1_PIN   16  // Left  motor dir A
#define IN2_PIN   17  // Left  motor dir B
#define IN3_PIN   18  // Right motor dir A
#define IN4_PIN   19  // Right motor dir B
#define ENA_PIN   25  // Left  motor PWM
#define ENB_PIN   26  // Right motor PWM

// SG90 Servo (drop gate)
#define SERVO_PIN 27

// TCS230 Color Sensor
#define TCS_S0   13
#define TCS_S1   14
#define TCS_S2   21
#define TCS_S3   4
#define TCS_OUT  33

// ============================================================
//  LEDC PWM CONFIG
// ============================================================
#define PWM_FREQ        5000
#define PWM_RESOLUTION  8       // 0–255

// ============================================================
//  TUNABLE CONSTANTS
// ============================================================

// --- Motor Speeds (0–255) ---
#define MAX_SPEED       200   // max speed on straights
#define BASE_SPEED      170   // normal line following (slightly reduced for safety)
#define CURVE_SPEED      80   // reduced speed on moderate curves
#define SHARP_TURN_SPEED 100   // pivot speed for tight corners — increased for more turn angle
#define MIN_SPEED        80   // minimum motor speed (below this motors stall)
#define AVOID_SPEED     130   // obstacle avoidance maneuver
#define CREEP_SPEED      90   // slow approach to green cube
#define SPIN_SPEED      120   // lost-line spin recovery

// --- PID Gains ---
// Tuned for tight 45° / 90° corners in the arena:
//   Kp raised (1.8→3.2): stronger reaction to corner error
//   Kd raised (2.0→4.5): damps oscillation from higher Kp
//   Ki kept small — integral still helps steady-state on straights
//   CURVE_SPEED lowered so correction stays proportional at corners
float Kp = 3.2;
float Ki = 0.01;
float Kd = 4.5;
float integralSum = 0.0;
#define INTEGRAL_MAX  500.0   // anti-windup clamp

// --- Ultrasonic ---
#define DETECT_DIST_CM    20    // distance to detect object on path
#define END_WALL_DIST_CM   8    // distance to detect end-zone wall

// --- Servo Positions (degrees) ---
#define SERVO_OPEN    85    // gate open — cube drops through
#define SERVO_CLOSED  15    // gate closed — cube held

// --- Timing ---
#define END_ZONE_TIMEOUT_MS   15000  // safety fallback after pick-up
#define STUCK_TIMEOUT_MS      15000  // watchdog: no state change
#define LINE_LOSS_CONFIRM_MS    300  // confirm line truly lost (not gap)

// --- IR Sensor Type ---
// Your sensors are 3-pin DIGITAL modules (VCC, GND, OUT)
// The onboard potentiometer sets the detection threshold — adjust it
// so the module LED turns ON over yellow tape and OFF over grey floor.
//
// INVERT_IR: Most modules output LOW when detecting a reflective surface
// (yellow tape) and HIGH when on dark/less reflective surface (grey floor).
// If your sensors work the opposite way, change this to false.
#define INVERT_IR       true    // true = LOW from sensor means "on line"

// --- Color Sensor Calibration (fill from CALIBRATE_COLOR mode) ---
// These are pulseIn values: lower = more of that color
// Calibrate with white and black reference + your actual cubes
long colorCal_R_min = 20,  colorCal_R_max = 300;   // Red channel range
long colorCal_G_min = 25,  colorCal_G_max = 350;   // Green channel range
long colorCal_B_min = 20,  colorCal_B_max = 280;   // Blue channel range

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
Servo gateServo;

int irRaw[5];
int irBinary[5];
int irPins[5] = {IR_S1, IR_S2, IR_S3, IR_S4, IR_S5};

int   lastError        = 0;
int   lastTightDir     = 0;   // -1=last tight was LEFT, +1=RIGHT, 0=none
bool  cubePickedUp     = false;
int   obstaclesAvoided = 0;
unsigned long pickUpTime     = 0;
unsigned long stateEntryTime = 0;
unsigned long lastDebugPrint = 0;
unsigned long lineLostStart  = 0;
bool  lineLostTiming   = false;

// ============================================================
//  STATE MACHINE
// ============================================================
enum RobotState {
  STATE_LINE_FOLLOW,      // follow yellow line
  STATE_IDENTIFY_OBJECT,  // object detected — read color
  STATE_AVOID_RED,        // navigate around red obstacle
  STATE_REJOIN_LINE,      // find line after avoidance
  STATE_APPROACH_GREEN,   // slow approach to green cube
  STATE_PICK_GREEN,       // close servo to grip cube
  STATE_CARRY_TO_END,     // follow line with cube
  STATE_DROP_CUBE,        // open servo at end zone
  STATE_DONE              // all tasks complete
};

RobotState currentState = STATE_LINE_FOLLOW;
RobotState prevState    = STATE_LINE_FOLLOW;

// State names for debug printing
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
  Serial.println("  EC6090 Robot v6.8 — More Turn Angle on Tight Corners");
  Serial.println("============================================");
  Serial.print("IR Sensors: DIGITAL (3-pin), INVERT=");
  Serial.println(INVERT_IR ? "YES (LOW=line)" : "NO (HIGH=line)");

  // Motor direction pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // LEDC PWM for motors (ESP32 core v3.x API)
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RESOLUTION);
  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // TCS230 color sensor
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  // Frequency scaling 20% (S0=HIGH, S1=LOW) — best for ESP32 timing
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  // IR sensor pins — all set as INPUT for digitalRead()
  // GPIO 35,34,39,36 are input-only pins, GPIO 32 is normal
  pinMode(IR_S1, INPUT);
  pinMode(IR_S2, INPUT);
  pinMode(IR_S3, INPUT);
  pinMode(IR_S4, INPUT);
  pinMode(IR_S5, INPUT);

  // Servo — start with gate OPEN (no cube held yet)
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(SERVO_OPEN);
  delay(500);

  // ── Run test mode if any test flag is set ──
  if (TEST_MOTORS || TEST_IR_RAW || TEST_COLOR ||
      TEST_ULTRASONIC || TEST_SERVO || CALIBRATE_COLOR) {
    runTestMode();
    // runTestMode() loops forever — never reaches here
  }

  // Normal startup — place robot on the yellow line, then power on
  Serial.println("Place robot on LINE, then wait for GO...");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  stateEntryTime = millis();
  Serial.println("GO! — Line following started");
}

// ============================================================
//  MAIN LOOP — State Machine
// ============================================================
void loop() {
  readIRSensors();

  // Watchdog: if stuck in same state too long, attempt recovery
  if (currentState != STATE_DONE) {
    if (millis() - stateEntryTime > STUCK_TIMEOUT_MS &&
        currentState == prevState) {
      Serial.println("⚠ WATCHDOG: Stuck too long — attempting recovery spin");
      stopMotors();
      delay(200);
      // Spin in direction of last known line
      if (lastError >= 0) {
        driveMotors(SPIN_SPEED, -SPIN_SPEED);
      } else {
        driveMotors(-SPIN_SPEED, SPIN_SPEED);
      }
      delay(600);
      stopMotors();
      stateEntryTime = millis();
      if (currentState != STATE_LINE_FOLLOW && currentState != STATE_CARRY_TO_END) {
        changeState(STATE_REJOIN_LINE);
      }
    }
  }
  prevState = currentState;

  // Debug dashboard at 10Hz
  if (PRINT_DEBUG && millis() - lastDebugPrint > 100) {
    printDashboard();
    lastDebugPrint = millis();
  }

  switch (currentState) {

    // ── Normal line following ─────────────────────────────
    case STATE_LINE_FOLLOW: {
      // Check ultrasonic for objects ahead
      float dist = readUltrasonic();
      if (dist > 2 && dist < DETECT_DIST_CM) {
        stopMotors();
        delay(200);
        Serial.print("Object at ");
        Serial.print(dist, 1);
        Serial.println(" cm — checking color...");
        changeState(STATE_IDENTIFY_OBJECT);
        break;
      }
      followLineAdaptive();
      break;
    }

    // ── Identify object color ─────────────────────────────
    case STATE_IDENTIFY_OBJECT: {
      // Move a tiny bit closer for better color reading
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(150);
      stopMotors();
      delay(200);

      char color = readColorVoted(9);  // 9 votes for reliability
      Serial.print("Color result: ");
      Serial.println(color);

      if (color == 'R') {
        Serial.println("→ RED obstacle — avoiding");
        changeState(STATE_AVOID_RED);
      } else if (color == 'G' && !cubePickedUp) {
        Serial.println("→ GREEN cube — picking up");
        changeState(STATE_APPROACH_GREEN);
      } else {
        // Unknown color or already carrying — treat as obstacle
        Serial.println("→ Unknown/other — avoiding as obstacle");
        changeState(STATE_AVOID_RED);
      }
      break;
    }

    // ── Navigate around red obstacle ──────────────────────
    case STATE_AVOID_RED: {
      avoidObstacle();
      obstaclesAvoided++;
      Serial.print("Obstacles avoided: ");
      Serial.println(obstaclesAvoided);
      changeState(STATE_REJOIN_LINE);
      break;
    }

    // ── Find line after avoidance ─────────────────────────
    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) {
        changeState(cubePickedUp ? STATE_CARRY_TO_END : STATE_LINE_FOLLOW);
      } else {
        // Couldn't find line — spin toward last known direction & retry
        Serial.println("Rejoin timeout — spinning to recover...");
        if (lastError >= 0) {
          driveMotors(SPIN_SPEED, -SPIN_SPEED);
        } else {
          driveMotors(-SPIN_SPEED, SPIN_SPEED);
        }
        delay(500);
        stopMotors();
        // Stay in REJOIN state, watchdog will catch if truly stuck
      }
      break;
    }

    // ── Slow approach to green cube ───────────────────────
    case STATE_APPROACH_GREEN: {
      // Creep forward to position cube inside the drop gate
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(500);   // tune: distance from color sensor to gate
      stopMotors();
      delay(200);
      changeState(STATE_PICK_GREEN);
      break;
    }

    // ── Close servo to grip cube ──────────────────────────
    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      changeState(STATE_CARRY_TO_END);
      break;
    }

    // ── Follow line while carrying cube to end zone ───────
    case STATE_CARRY_TO_END: {
      // --- END ZONE DETECTION (triple-confirm) ---

      // Method 1: Safety timeout after pick-up
      if (millis() - pickUpTime > END_ZONE_TIMEOUT_MS) {
        stopMotors();
        Serial.println("End zone — timeout fallback");
        changeState(STATE_DROP_CUBE);
        break;
      }

      // Method 2: Ultrasonic detects close wall/barrier at end
      float dist = readUltrasonic();
      if (dist > 0 && dist < END_WALL_DIST_CM) {
        stopMotors();
        Serial.println("End zone — wall detected");
        changeState(STATE_DROP_CUBE);
        break;
      }

      // Method 3: All IR sensors lose the line (line terminates)
      // But only after enough travel time from pick-up (>3s)
      if (millis() - pickUpTime > 3000) {
        int sensorCount = 0;
        for (int i = 0; i < 5; i++) sensorCount += irBinary[i];
        if (sensorCount == 0) {
          // Confirm it's not just a gap in the tape
          delay(LINE_LOSS_CONFIRM_MS);
          readIRSensors();
          sensorCount = 0;
          for (int i = 0; i < 5; i++) sensorCount += irBinary[i];
          if (sensorCount == 0) {
            stopMotors();
            Serial.println("End zone — line terminated");
            changeState(STATE_DROP_CUBE);
            break;
          }
        }
      }

      // If object detected while carrying — must avoid (could be 2nd red obstacle)
      if (dist > 2 && dist < DETECT_DIST_CM) {
        stopMotors();
        delay(200);
        char color = readColorVoted(5);
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

    // ── Drop cube at end zone ─────────────────────────────
    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      changeState(STATE_DONE);
      break;
    }

    // ── Task complete ─────────────────────────────────────
    case STATE_DONE: {
      stopMotors();
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 3000) {
        Serial.println("★★★ TASK COMPLETE — All objectives done! ★★★");
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
  integralSum    = 0.0;  // reset PID integral on state change
}

// ============================================================
//  READ IR SENSORS (using fixed threshold)
// ============================================================
void readIRSensors() {
  for (int i = 0; i < 5; i++) {
    int raw = digitalRead(irPins[i]);
    irRaw[i] = raw;  // store 0 or 1 for debug

    if (INVERT_IR) {
      // Most 3-pin modules: LOW = reflective surface (yellow tape) = on line
      irBinary[i] = (raw == LOW) ? 1 : 0;
    } else {
      // Some modules: HIGH = reflective surface (yellow tape) = on line
      irBinary[i] = (raw == HIGH) ? 1 : 0;
    }
  }
}

// ============================================================
//  LINE FOLLOWING — PID + Tight Turn (v6.7)
//
//  ROOT CAUSE OF PREVIOUS BUG (v6.6):
//    readFarSensorStable() was re-reading IR_S1/IR_S5 raw pins
//    separately from irBinary[]. On ANY gentle left curve S1
//    would also read 1, so the tight-turn triggered constantly
//    and stopped the robot even on normal curves.
//
//  FIX — CORRECT TIGHT TURN TRIGGER CONDITION:
//    Use irBinary[] which is already read once per loop().
//    Tight turn fires ONLY when:
//      • S1=1 AND S2=0 AND S3=0   → line is at extreme far left
//      • S5=1 AND S4=0 AND S3=0   → line is at extreme far right
//    This means the line has moved completely off the middle
//    sensors → it is genuinely a tight/90° corner, not a gentle
//    curve (on gentle curves S2 or S3 would still be active).
//
//  SEQUENCE (from research + friend's code combined):
//    1. Detect tight corner via irBinary[] pattern above
//    2. Stop motors fully — kill momentum
//    3. Small settle delay (50ms)
//    4. Set turning=true, record direction
//    5. Every loop(): pivot on the spot (PID completely OFF)
//    6. Check only irBinary[2] (S3 center) to exit pivot
//    7. When S3 sees line → stop pivot → PID resumes
// ============================================================

// Turning state — persists across loop() calls
bool  turning       = false;
int   turnDirection = 0;          // -1=LEFT, +1=RIGHT
unsigned long turnStartTime = 0;

void followLineAdaptive() {

  // Use irBinary[] — already freshly read in loop() via readIRSensors()
  bool s1 = irBinary[0];   // Far  Left
  bool s2 = irBinary[1];   // Mid  Left
  bool s3 = irBinary[2];   // Center
  bool s4 = irBinary[3];   // Mid  Right
  bool s5 = irBinary[4];   // Far  Right

  // ── TIGHT TURN TRIGGER ───────────────────────────────────────
  //  Condition: far sensor ON + both middle-center sensors OFF
  //  This separates a genuine tight corner from a normal curve.
  //  On a normal curve S2 or S3 would still be touching the line.
  //  On a 45°/90° corner the line jumps to S1 (or S5) only.

  if (!turning) {

    // TIGHT LEFT: S1 active, S2 and S3 both off
    if (s1 && !s2 && !s3) {
      Serial.println("TIGHT TURN LEFT DETECTED → stopping + pivoting");
      turning       = true;
      turnDirection = -1;
      lastTightDir  = -1;
      integralSum   = 0.0;

      // Step 1: Full stop — kill all forward momentum
      stopMotors();
      delay(50);

      // Step 2: Record time for exit-check guard
      turnStartTime = millis();
    }

    // TIGHT RIGHT: S5 active, S4 and S3 both off
    else if (s5 && !s4 && !s3) {
      Serial.println("TIGHT TURN RIGHT DETECTED → stopping + pivoting");
      turning       = true;
      turnDirection = 1;
      lastTightDir  = 1;
      integralSum   = 0.0;

      stopMotors();
      delay(50);

      turnStartTime = millis();
    }
  }

  // ── TURNING MODE — pivot until center S3 finds the line ──────
  if (turning) {

    if (turnDirection == -1) {
      // Pivot LEFT: left wheel BACKWARD, right wheel FORWARD
      driveMotors(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
    } else {
      // Pivot RIGHT: left wheel FORWARD, right wheel BACKWARD
      driveMotors(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);
    }

    // Guard: wait at least 150ms before checking exit condition
    // More time = more rotation = robot points further into the corner
    // This was 80ms before — increasing it fixes "not enough turn angle"
    if (millis() - turnStartTime > 150) {
      readIRSensors();   // fresh read inside turning loop
      // Exit when S2 (mid-left) OR S3 (center) sees the line
      // Using S2||S3 means the robot turns until the line is properly
      // centred under the middle sensors — not just barely touching S3
      if (irBinary[1] == 1 || irBinary[2] == 1) {
        stopMotors();
        delay(20);
        turning   = false;
        lastError = 0;
        Serial.println("TURN COMPLETE → PID resuming");
      }
    }

    return;   // PID completely OFF while turning
  }

  // ── NORMAL PID LINE FOLLOWING ────────────────────────────────
  int weights[5] = {-40, -20, 0, 20, 40};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  // ── Lost-line recovery ──
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

    // Longer loss — spin toward last known tight turn direction
    int spinDir = (lastTightDir != 0) ? lastTightDir : (lastError >= 0 ? 1 : -1);
    if (spinDir > 0) driveMotors(SPIN_SPEED, -SPIN_SPEED);
    else             driveMotors(-SPIN_SPEED, SPIN_SPEED);
    return;
  }

  lineLostTiming = false;

  int error = weightedSum / sensorSum;

  integralSum += (float)error;
  integralSum  = constrain(integralSum, -INTEGRAL_MAX, INTEGRAL_MAX);
  float derivative = (float)(error - lastError);
  float correction = (Kp * (float)error) + (Ki * integralSum) + (Kd * derivative);
  lastError = error;

  int absError = abs(error);
  int baseSpd  = BASE_SPEED;
  if      (absError <= 5)  baseSpd = MAX_SPEED;
  else if (absError <= 20) baseSpd = BASE_SPEED;
  else                     baseSpd = CURVE_SPEED;

  int leftSpeed  = constrain(baseSpd - (int)correction, -120, 255);
  int rightSpeed = constrain(baseSpd + (int)correction, -120, 255);

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  FOLLOW LINE (non-adaptive, for simple cases)
// ============================================================
void followLine() {
  followLineAdaptive();
}

// ============================================================
//  AVOID OBSTACLE — Intelligent bypass
//
//  Strategy: Check which side has more line visible to decide
//  whether to go LEFT or RIGHT around the obstacle.
//  Default: RIGHT bypass (arena obstacles tend to be on curves)
//
//  Maneuver steps:
//    1. Turn away from obstacle
//    2. Drive forward past it
//    3. Turn back toward line
//    4. Drive forward to cross line
// ============================================================
void avoidObstacle() {
  Serial.println("── Avoiding obstacle ──");

  // Decide direction: check outer sensors
  readIRSensors();
  bool lineOnLeft  = (irBinary[0] == 1 || irBinary[1] == 1);
  bool lineOnRight = (irBinary[3] == 1 || irBinary[4] == 1);

  // If line is visible on left but not right, obstacle is more to the right
  // → go LEFT. Otherwise default RIGHT bypass.
  bool goRight = true;
  if (lineOnLeft && !lineOnRight) {
    goRight = false;
    Serial.println("  Direction: LEFT bypass");
  } else {
    Serial.println("  Direction: RIGHT bypass");
  }

  int turnSign = goRight ? 1 : -1;

  // Step 1: Turn away from obstacle
  driveMotors(turnSign * AVOID_SPEED, -turnSign * AVOID_SPEED);
  delay(400);      // ~45° turn — tune for your robot
  stopMotors();
  delay(100);

  // Step 2: Drive forward to clear the obstacle
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(700);      // clear the cube length — tune for your robot
  stopMotors();
  delay(100);

  // Step 3: Turn back toward the line
  driveMotors(-turnSign * AVOID_SPEED, turnSign * AVOID_SPEED);
  delay(400);      // ~45° back — tune for your robot
  stopMotors();
  delay(100);

  // Step 4: Drive forward to reach the line
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(450);
  stopMotors();
  delay(150);

  Serial.println("  Avoidance maneuver complete");
}

// ============================================================
//  REJOIN LINE — Arc-sweep search
//
//  After obstacle avoidance, the robot may be offset from the
//  line. This function sweeps in arcs to find it.
// ============================================================
bool rejoinLine() {
  Serial.println("Searching for line (arc sweep)...");
  unsigned long start = millis();

  // First: creep forward with slight bias toward expected line side
  int sweepDir = (lastError >= 0) ? 1 : -1;  // bias toward last known line side

  while (millis() - start < 5000) {  // 5 second timeout
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];

    if (count >= 1) {
      Serial.println("Line found!");
      return true;
    }

    // Alternate arc patterns
    unsigned long elapsed = millis() - start;
    if (elapsed < 1500) {
      // Arc forward with bias
      driveMotors(90 + sweepDir * 30, 90 - sweepDir * 30);
    } else if (elapsed < 3000) {
      // Arc other direction
      driveMotors(90 - sweepDir * 30, 90 + sweepDir * 30);
    } else {
      // Wider sweep — almost spinning
      driveMotors(sweepDir * 100, -sweepDir * 100);
    }
    delay(30);
  }

  stopMotors();
  Serial.println("Line NOT found within 5s timeout");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
// ============================================================
void pickGreenCube() {
  Serial.println("── Picking up green cube ──");
  stopMotors();
  delay(200);

  // Ensure gate is open so cube can enter the mechanism
  gateServo.write(SERVO_OPEN);
  delay(400);

  // Drive forward a tiny bit to position cube inside gate
  driveMotors(CREEP_SPEED, CREEP_SPEED);
  delay(300);
  stopMotors();
  delay(200);

  // Close gate to hold cube
  gateServo.write(SERVO_CLOSED);
  delay(800);

  Serial.println("Cube gripped!");
}

// ============================================================
//  DROP CUBE AT END ZONE
// ============================================================
void dropCubeAtEnd() {
  Serial.println("── Dropping cube at end zone ──");
  stopMotors();
  delay(300);

  // Open gate — cube drops
  gateServo.write(SERVO_OPEN);
  delay(1000);

  // Reverse slightly to clear the cube
  driveMotors(-100, -100);
  delay(500);
  stopMotors();

  cubePickedUp = false;
  Serial.println("Cube released!");
}

// ============================================================
//  TCS230 COLOR SENSOR — single reading with mapping
//
//  Lower pulseIn(LOW) value = higher frequency = MORE of color
//  Maps raw pulse values to 0-255 scale using calibration
// ============================================================
char readColorOnce() {
  long r = 0, g = 0, b = 0;

  // Read Red channel (S2=LOW, S3=LOW)
  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, LOW);
  delay(12);
  r = pulseIn(TCS_OUT, LOW, 100000);

  // Read Green channel (S2=HIGH, S3=HIGH)
  digitalWrite(TCS_S2, HIGH);
  digitalWrite(TCS_S3, HIGH);
  delay(12);
  g = pulseIn(TCS_OUT, LOW, 100000);

  // Read Blue channel (S2=LOW, S3=HIGH)
  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, HIGH);
  delay(12);
  b = pulseIn(TCS_OUT, LOW, 100000);

  // Timeout check — any 0 means that channel timed out
  if (r == 0 || g == 0 || b == 0) return 'U';

  // Map raw pulse values to 0-255 scale (inverted: smaller pulse = more color)
  int rMapped = map(constrain(r, colorCal_R_min, colorCal_R_max),
                    colorCal_R_min, colorCal_R_max, 255, 0);
  int gMapped = map(constrain(g, colorCal_G_min, colorCal_G_max),
                    colorCal_G_min, colorCal_G_max, 255, 0);
  int bMapped = map(constrain(b, colorCal_B_min, colorCal_B_max),
                    colorCal_B_min, colorCal_B_max, 255, 0);

  // Debug: uncomment to see mapped values
  /*
  Serial.print("  RGB mapped: R="); Serial.print(rMapped);
  Serial.print(" G="); Serial.print(gMapped);
  Serial.print(" B="); Serial.println(bMapped);
  */

  // Determine dominant color using mapped values
  // Red cube: high R, low G, low B
  if (rMapped > gMapped + 30 && rMapped > bMapped + 30) return 'R';
  // Green cube: high G, low R, low B
  if (gMapped > rMapped + 30 && gMapped > bMapped + 20) return 'G';
  // Blue (not expected in this arena but included for completeness)
  if (bMapped > rMapped + 30 && bMapped > gMapped + 30) return 'B';

  // Check with ratio approach as fallback
  // For red: r is smallest raw pulse (most light), g and b are larger
  if (r < g && r < b && g > r * 1.3) return 'R';
  if (g < r && g < b && r > g * 1.3) return 'G';

  return 'U';  // unknown
}

// ============================================================
//  TCS230 COLOR SENSOR — voted reading (more reliable)
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
    delay(20);
  }

  Serial.print("Color votes — R:"); Serial.print(cR);
  Serial.print(" G:"); Serial.print(cG);
  Serial.print(" B:"); Serial.print(cB);
  Serial.print(" U:"); Serial.println(cU);

  // Need clear majority (> unknown votes)
  if (cR >= cG && cR >= cB && cR > cU) return 'R';
  if (cG >= cR && cG >= cB && cG > cU) return 'G';
  if (cB >= cR && cB >= cG && cB > cU) return 'B';
  return 'U';
}

// ============================================================
//  ULTRASONIC SENSOR — returns distance in cm
// ============================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);  // 25ms timeout ≈ 4.3m
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

// ============================================================
//  MOTOR CONTROL
//  Positive = forward, Negative = reverse
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, constrain(-leftSpeed, 0, 255));
  }

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENB_PIN, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    ledcWrite(ENB_PIN, constrain(-rightSpeed, 0, 255));
  }
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);
}

// ============================================================
//  DEBUG DASHBOARD — prints every 100ms if PRINT_DEBUG is true
// ============================================================
void printDashboard() {
  Serial.print("[");
  Serial.print(stateNames[currentState]);
  Serial.print("] IR:");
  for (int i = 0; i < 5; i++) {
    Serial.print(irBinary[i]);
  }
  Serial.print(" raw:");
  for (int i = 0; i < 5; i++) {
    Serial.print(irRaw[i]);
    Serial.print(",");
  }
  Serial.print(" err:");
  Serial.print(lastError);
  float dist = readUltrasonic();
  Serial.print(" dist:");
  Serial.print(dist, 1);
  Serial.print(" obs:");
  Serial.print(obstaclesAvoided);
  Serial.print(" cube:");
  Serial.println(cubePickedUp ? "YES" : "NO");
}

// ============================================================
//  TEST MODES — for debugging individual subsystems
// ============================================================
void runTestMode() {
  Serial.println("╔══════════════════════════════╗");
  Serial.println("║     TEST MODE ACTIVE         ║");
  Serial.println("╚══════════════════════════════╝");

  // ── Motor Test ──
  if (TEST_MOTORS) {
    Serial.println("=== MOTOR TEST ===");
    while (true) {
      Serial.println("Forward...");
      driveMotors(150, 150);
      delay(2000);
      stopMotors(); delay(500);

      Serial.println("Spin LEFT (counterclockwise)...");
      driveMotors(-150, 150);
      delay(1500);
      stopMotors(); delay(500);

      Serial.println("Spin RIGHT (clockwise)...");
      driveMotors(150, -150);
      delay(1500);
      stopMotors(); delay(500);

      Serial.println("Reverse...");
      driveMotors(-150, -150);
      delay(2000);
      stopMotors(); delay(1000);

      Serial.println("--- Cycle complete, repeating ---");
    }
  }

  // ── IR Sensor Test ──
  if (TEST_IR_RAW) {
    Serial.println("=== IR SENSOR TEST (DIGITAL) ===");
    Serial.println("Move robot over line/floor. 0 or 1 per sensor.");
    Serial.println("After adjusting pots: should show different values on line vs floor.");
    Serial.println("INVERT_IR is: "); Serial.println(INVERT_IR ? "ON (LOW=line)" : "OFF (HIGH=line)");
    while (true) {
      Serial.print("RAW: ");
      for (int i = 0; i < 5; i++) {
        int val = digitalRead(irPins[i]);
        Serial.print("S"); Serial.print(i + 1);
        Serial.print("="); Serial.print(val);
        Serial.print("  ");
      }
      Serial.print("  →  BINARY: ");
      readIRSensors();
      for (int i = 0; i < 5; i++) {
        Serial.print(irBinary[i]);
      }
      Serial.println();
      delay(300);
    }
  }

  // ── Color Sensor Test ──
  if (TEST_COLOR) {
    Serial.println("=== COLOR SENSOR TEST ===");
    Serial.println("Place colored objects in front of sensor.");
    while (true) {
      long r, g, b;

      digitalWrite(TCS_S2, LOW); digitalWrite(TCS_S3, LOW);
      delay(12);
      r = pulseIn(TCS_OUT, LOW, 100000);

      digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH);
      delay(12);
      g = pulseIn(TCS_OUT, LOW, 100000);

      digitalWrite(TCS_S2, LOW); digitalWrite(TCS_S3, HIGH);
      delay(12);
      b = pulseIn(TCS_OUT, LOW, 100000);

      Serial.print("R="); Serial.print(r);
      Serial.print("\tG="); Serial.print(g);
      Serial.print("\tB="); Serial.print(b);
      Serial.print("\t→ ");

      char color = readColorVoted(5);
      Serial.println(color);
      delay(500);
    }
  }

  // ── Ultrasonic Test ──
  if (TEST_ULTRASONIC) {
    Serial.println("=== ULTRASONIC TEST ===");
    while (true) {
      float d = readUltrasonic();
      Serial.print("Distance: ");
      Serial.print(d, 1);
      Serial.println(" cm");
      delay(200);
    }
  }

  // ── Servo Test ──
  if (TEST_SERVO) {
    Serial.println("=== SERVO TEST ===");
    while (true) {
      Serial.print("OPEN ("); Serial.print(SERVO_OPEN); Serial.println("°)");
      gateServo.write(SERVO_OPEN);
      delay(2000);
      Serial.print("CLOSED ("); Serial.print(SERVO_CLOSED); Serial.println("°)");
      gateServo.write(SERVO_CLOSED);
      delay(2000);
    }
  }

  // ── Color Calibration Mode ──
  if (CALIBRATE_COLOR) {
    Serial.println("=== COLOR CALIBRATION ===");
    Serial.println("Place WHITE reference first, then colored objects.");
    Serial.println("Note the min/max R, G, B values for calibration.");
    while (true) {
      long r, g, b;

      digitalWrite(TCS_S2, LOW); digitalWrite(TCS_S3, LOW);
      delay(12);
      r = pulseIn(TCS_OUT, LOW, 100000);

      digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH);
      delay(12);
      g = pulseIn(TCS_OUT, LOW, 100000);

      digitalWrite(TCS_S2, LOW); digitalWrite(TCS_S3, HIGH);
      delay(12);
      b = pulseIn(TCS_OUT, LOW, 100000);

      Serial.print("RAW → R="); Serial.print(r);
      Serial.print(" G="); Serial.print(g);
      Serial.print(" B="); Serial.println(b);
      Serial.println("  Record these values for white, black, red, & green cubes");
      delay(1000);
    }
  }
}

// ============================================================
//  END OF CODE v6.2 — EC6090 Mini Project 2026 — Tight Turns
// ============================================================
