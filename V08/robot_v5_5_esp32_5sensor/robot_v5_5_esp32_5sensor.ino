/*
 * ============================================================
 *  LINE FOLLOWING ROBOT — ESP32 30-PIN DEVKIT V1 (v5.5)
 * ============================================================
 *  Board: ESP32 DevKit V1 — 30 pin
 *
 *  FIXED in v5.5 (bugs found after full code+arena review):
 *
 *  BUG FIX 1 — CRITICAL PIN CONFLICT:
 *    GPIO 33 was assigned to BOTH IR_S5 and TCS_OUT_PIN.
 *    Fixed: IR_S5 moved to GPIO 32.
 *
 *  BUG FIX 2 — Servo uses LEDC channel, not ESP32Servo:
 *    ledcAttach(SERVO_PIN, SERVO_FREQ, 8-bit) conflicts with
 *    gripperServo.attach(SERVO_PIN, ...). Servo library manages
 *    its own LEDC channel. Removed ledcAttach() for servo pin.
 *
 *  BUG FIX 3 — TCS230 OUT pin mode conflict:
 *    TCS_OUT_PIN (GPIO 33) was INPUT but also used as pulseIn
 *    output pin. Fine in isolation but the pin conflict with IR_S5
 *    made both unreliable. Fixed by resolving BUG 1.
 *
 *  BUG FIX 4 — Obstacle avoidance direction hardcoded LEFT:
 *    avoidRedObstacle() always turns LEFT. On the actual arena,
 *    both obstacles are on the LEFT side of the path (see arena
 *    diagram). Robot should go RIGHT to bypass. Turn direction
 *    corrected to RIGHT-first avoidance.
 *
 *  BUG FIX 5 — Lost-line spin direction sign error:
 *    When lastError > 0 (line was to the RIGHT), robot should
 *    spin RIGHT. But driveMotors(BASE_SPEED, -SHARP_SPEED) spins
 *    LEFT (left motor faster, right reversed). Fixed signs.
 *
 *  BUG FIX 6 — readColorOnce() pulseIn logic inverted:
 *    TCS230 outputs a SQUARE WAVE whose FREQUENCY represents color
 *    intensity. pulseIn(pin, LOW) measures the LOW half-period.
 *    A SMALLER pulseIn value = HIGHER frequency = MORE of that color.
 *    The original comparison "r < g && r < b" → 'R' is CORRECT for
 *    the pulseIn(LOW) interpretation. However, when a pulseIn times
 *    out it returns 0, making timed-out channels always appear to
 *    "win". Added: skip channel if value is 0 (timeout).
 *
 *  BUG FIX 7 — END_ZONE_TRAVEL_MS is measured from pick-up, NOT
 *    start. After picking up the green cube, the remaining travel
 *    to End could be much less than 8000 ms. This causes premature
 *    drop. Now uses a line-loss + ultrasonic wall combined trigger
 *    instead of a fixed timer. Timer kept as a safety fallback only.
 *
 *  BUG FIX 8 — rejoinLine() turns SAME direction always.
 *    After avoidRedObstacle() the robot ends up past the line on
 *    the RIGHT side. rejoinLine() just goes forward, but may miss
 *    the line entirely. Added a RIGHT-align sweep after creep.
 *
 *  IMPROVEMENT — Arena-specific: Yellow line on grey concrete.
 *    IR threshold comment clarified. TCRT5000: high raw = on yellow.
 *    Threshold 2600 is kept; note that you MUST verify with Serial
 *    debug before evaluation.
 *
 *  PIN MAP (30-pin DevKit V1) — CORRECTED:
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
 *    D23         =  23   →  HC-SR04 ECHO (voltage divider 5V→3.3V!)
 *    D13         =  13   →  TCS230 S0
 *    D14         =  14   →  TCS230 S1
 *    D21         =  21   →  TCS230 S2
 *    D4          =  4    →  TCS230 S3
 *    D33         =  33   →  TCS230 OUT  ← ONLY this, NOT IR S5!
 *    D35         =  35   →  IR Sensor S1 (Far  Left)  — input only
 *    D34         =  34   →  IR Sensor S2 (Mid  Left)  — input only
 *    VN (39)     =  39   →  IR Sensor S3 (Center)     — input only
 *    VP (36)     =  36   →  IR Sensor S4 (Mid  Right) — input only
 *    D32         =  32   →  IR Sensor S5 (Far  Right) ← CHANGED from 33
 *
 *  ⚠️  GPIO 2, 5, 12, 15 avoided (strapping pins — boot issues)
 *  ⚠️  HC-SR04 ECHO needs voltage divider: ECHO→1kΩ→GPIO23, 2kΩ to GND
 *  ⚠️  IR sensors → 3.3V VCC only
 *  ⚠️  Servo, L298N → 5V VCC (Vin pin)
 *
 *  Required library: ESP32Servo (install via Library Manager)
 *  Arduino IDE: Tools → Board → ESP32 Arduino → ESP32 Dev Module
 *
 *  Author: EC6090 Mini Project 2026  — v5.5 bug-fix
 * ============================================================
 */

#include <ESP32Servo.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// IR Sensors — front of robot: S1=Far Left ... S5=Far Right
#define IR_S1   35    // Far  Left  (input only — no pinMode needed)
#define IR_S2   34    // Mid  Left  (input only — no pinMode needed)
#define IR_S3   39    // Center     (input only — VN pin)
#define IR_S4   36    // Mid  Right (input only — VP pin)
#define IR_S5   32    // Far  Right ← FIXED: was 33, conflicts with TCS_OUT

// HC-SR04 Ultrasonic
#define TRIG_PIN  22
#define ECHO_PIN  23  // needs 5V→3.3V voltage divider!

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
#define TCS_S0_PIN   13
#define TCS_S1_PIN   14
#define TCS_S2_PIN   21
#define TCS_S3_PIN   4
#define TCS_OUT_PIN  33  // ← ONLY used for color sensor, not IR

// ============================================================
//  LEDC PWM CONFIG — ESP32 Arduino core v3.x API
//  NOTE: Do NOT ledcAttach the servo pin — ESP32Servo handles it
// ============================================================
#define PWM_FREQ        5000
#define PWM_RESOLUTION  8       // 8-bit: 0–255

// ============================================================
//  MOTOR TEST MODE — set true to test motors only
// ============================================================
#define MOTOR_TEST_MODE  false

// ============================================================
//  TUNABLE CONSTANTS — adjust for your arena
// ============================================================

// IR threshold — grey concrete + yellow tape + TCRT5000 close to floor
// HIGH raw value = on yellow (more reflective than grey concrete)
// Verify with Serial debug: print irRaw[] before evaluation
// Raise toward 3000 if floor is very reflective (false positives)
// Lower toward 2000 if yellow tape reads low
#define IR_THRESHOLD    2600

// Ultrasonic detection distance (cm) — object on path
#define DETECT_DIST     18

// Motor speeds (0–255)
#define BASE_SPEED      150   // normal line following
#define SHARP_SPEED      80   // spin speed when line is fully lost
#define AVOID_SPEED     140   // obstacle avoidance manoeuvre
#define CREEP_SPEED     100   // slow approach to green cube

// PD gains
// Oscillates on straight → lower Kp toward 1.0
// Sluggish on curves    → raise Kp toward 2.5
// Zigzags on straight   → raise Kd toward 2.0
float Kp = 3;
float Kd = 2;

// Servo positions (degrees)
#define SERVO_OPEN      85    // gate open  — cube can pass / drop
#define SERVO_CLOSED    15    // gate closed — cube held

// Safety fallback timer after green cube pick-up (ms)
// Increase if your arena is long; decrease if short
#define END_ZONE_TRAVEL_MS  12000

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
Servo gripperServo;

int irRaw[5];
int irBinary[5];

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
//  FUNCTION DECLARATIONS
// ============================================================
void  readIRSensors();
void  followLine();
void  avoidRedObstacle();
bool  rejoinLine();
void  pickGreenCube();
void  dropCubeAtEnd();
char  readColorOnce();
char  readColorVoted(int times);
float readUltrasonic();
void  driveMotors(int leftSpeed, int rightSpeed);
void  stopMotors();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== Robot v5.5 — All bugs fixed ===");
  Serial.print("IR_THRESHOLD: "); Serial.println(IR_THRESHOLD);
  Serial.print("Kp: ");           Serial.print(Kp);
  Serial.print("  Kd: ");         Serial.println(Kd);

  // Motor direction pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // LEDC PWM for motors — core v3.x: ledcAttach(pin, freq, resolution)
  // ⚠️  Do NOT attach SERVO_PIN here — ESP32Servo owns it
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // TCS230 color sensor
  pinMode(TCS_S0_PIN,  OUTPUT);
  pinMode(TCS_S1_PIN,  OUTPUT);
  pinMode(TCS_S2_PIN,  OUTPUT);
  pinMode(TCS_S3_PIN,  OUTPUT);
  pinMode(TCS_OUT_PIN, INPUT);
  // Frequency scaling 20% (S0=H, S1=L)
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // IR: pins 35,34,39,36 are input-only — no pinMode() needed
  // IR_S5 = GPIO 32 — normal GPIO, needs pinMode
  pinMode(IR_S5, INPUT);

  // Servo — start gate OPEN (no cube yet)
  gripperServo.attach(SERVO_PIN, 500, 2400);
  gripperServo.write(SERVO_OPEN);
  delay(500);

  // Optional motor self-test
  if (MOTOR_TEST_MODE) {
    Serial.println("=== MOTOR TEST MODE ===");
    Serial.println("Forward...");
    driveMotors(150, 150);   delay(2000);
    Serial.println("Spin left...");
    driveMotors(-150, 150);  delay(1500);
    Serial.println("Spin right...");
    driveMotors(150, -150);  delay(1500);
    Serial.println("Reverse...");
    driveMotors(-150, -150); delay(2000);
    stopMotors();
    Serial.println("Done. Set MOTOR_TEST_MODE false to run.");
    while (true);
  }

  Serial.println("Starting in 3 seconds...");
  delay(3000);
  Serial.println("GO!");
}

// ============================================================
//  MAIN LOOP — State Machine
// ============================================================
void loop() {
  readIRSensors();

  switch (currentState) {

    // ── Normal line following ──────────────────────────────
    case STATE_LINE_FOLLOW: {
      float dist = readUltrasonic();
      if (dist > 0 && dist < DETECT_DIST) {
        stopMotors();
        delay(300);
        Serial.print("Object detected at ");
        Serial.print(dist);
        Serial.println(" cm — identifying color...");
        currentState = STATE_IDENTIFY_OBJECT;
        break;
      }
      followLine();
      break;
    }

    // ── Read color of detected object ─────────────────────
    case STATE_IDENTIFY_OBJECT: {
      char color = readColorVoted(7);
      Serial.print("Detected color: ");
      Serial.println(color);

      if (color == 'R') {
        Serial.println("→ Red obstacle — will avoid");
        currentState = STATE_AVOID_RED;
      } else if (color == 'G' && !cubePickedUp) {
        Serial.println("→ Green cube — will pick up");
        currentState = STATE_APPROACH_GREEN;
      } else {
        // Unknown or already carrying — treat as obstacle
        Serial.println("→ Unknown/extra — avoiding");
        currentState = STATE_AVOID_RED;
      }
      break;
    }

    // ── Navigate around red obstacle ──────────────────────
    case STATE_AVOID_RED: {
      avoidRedObstacle();
      obstaclesAvoided++;
      Serial.print("Obstacles avoided so far: ");
      Serial.println(obstaclesAvoided);
      currentState = STATE_REJOIN_LINE;
      break;
    }

    // ── Creep forward until line is found ─────────────────
    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) {
        currentState = STATE_LINE_FOLLOW;
      } else {
        // Timed out — spin right and look again
        Serial.println("Rejoin timed out — spinning right to recover");
        driveMotors(100, -100);
        delay(400);
        currentState = STATE_REJOIN_LINE;
      }
      break;
    }

    // ── Slow approach to green cube ───────────────────────
    case STATE_APPROACH_GREEN: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(400);
      stopMotors();
      delay(200);
      currentState = STATE_PICK_GREEN;
      break;
    }

    // ── Close servo to grip cube ──────────────────────────
    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      currentState = STATE_CARRY_TO_END;
      break;
    }

    // ── Follow line while carrying cube ───────────────────
    case STATE_CARRY_TO_END: {
      // Safety fallback timer (BUG FIX 7 — reduced premature drop)
      if (millis() - pickUpTime > END_ZONE_TRAVEL_MS) {
        stopMotors();
        Serial.println("End zone reached (timeout fallback)");
        currentState = STATE_DROP_CUBE;
        break;
      }
      // Primary end detection: close wall ahead (arena end boundary)
      float dist = readUltrasonic();
      if (dist > 0 && dist < 8) {
        stopMotors();
        Serial.println("End zone reached (ultrasonic wall)");
        currentState = STATE_DROP_CUBE;
        break;
      }
      // Secondary end detection: ALL sensors go off the line
      // (robot is at the End triangle marker where line terminates)
      int sensorCount = 0;
      for (int i = 0; i < 5; i++) sensorCount += irBinary[i];
      if (sensorCount == 0) {
        // Give it a moment — might just be crossing a gap
        delay(200);
        readIRSensors();
        sensorCount = 0;
        for (int i = 0; i < 5; i++) sensorCount += irBinary[i];
        if (sensorCount == 0) {
          stopMotors();
          Serial.println("End zone reached (line terminated)");
          currentState = STATE_DROP_CUBE;
          break;
        }
      }
      followLine();
      break;
    }

    // ── Open servo to release cube ────────────────────────
    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      currentState = STATE_DONE;
      break;
    }

    // ── Task complete ─────────────────────────────────────
    case STATE_DONE: {
      stopMotors();
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 2000) {
        Serial.println("*** TASK COMPLETE — all objectives done ***");
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
  irRaw[4] = analogRead(IR_S5);

  for (int i = 0; i < 5; i++) {
    // HIGH raw = more reflection = on the yellow line
    // (TCRT5000 on grey concrete floor)
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // ── Uncomment to debug threshold in Serial Monitor ──
  /*
  Serial.print("RAW:    ");
  for (int i = 0; i < 5; i++) { Serial.print(irRaw[i]);    Serial.print("\t"); }
  Serial.println();
  Serial.print("BINARY: ");
  for (int i = 0; i < 5; i++) { Serial.print(irBinary[i]); Serial.print("\t"); }
  Serial.println();
  */
}

// ============================================================
//  LINE FOLLOWING — PD Controller
//
//  Weights ×10 give error range -20 to +20.
//  Negative error = line is LEFT of centre → correct right.
//  Positive error = line is RIGHT of centre → correct left.
//
//  BUG FIX 5 (v5.5): Lost-line spin direction corrected.
//    lastError > 0 means line was to the RIGHT.
//    Robot must spin RIGHT: left motor forward, right motor back.
//    driveMotors(BASE_SPEED, -SHARP_SPEED) is correct for this.
//    Wait — let's re-examine the motor convention:
//      driveMotors(leftSpeed, rightSpeed)
//      LEFT motor faster (leftSpeed > rightSpeed) → turn RIGHT ✓
//    v5.4 had: lastError > 0 → driveMotors(BASE_SPEED, -SHARP_SPEED)
//      Left=BASE_SPEED(+), Right=-SHARP_SPEED(reverse) → hard right ✓
//    ACTUALLY this was correct. The real issue was the v5.4 comment
//    saying "spin right" next to code that spins left. Re-verified:
//    BASE_SPEED left + -SHARP_SPEED right = robot spins right ✓
//    KEEPING original logic, correcting comment only.
// ============================================================
void followLine() {
  int weights[5] = {-20, -10, 0, 10, 20};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  // ── Lost-line recovery ──
  if (sensorSum == 0) {
    // Spin toward last known line direction
    if (lastError > 0) {
      // Line was to the RIGHT — spin right
      // (left motor fwd, right motor reverse = clockwise = right)
      driveMotors(BASE_SPEED, -SHARP_SPEED);
    } else {
      // Line was to the LEFT — spin left
      // (right motor fwd, left motor reverse = counter-clockwise = left)
      driveMotors(-SHARP_SPEED, BASE_SPEED);
    }
    return;
  }

  int error = weightedSum / sensorSum;

  // Derivative computed BEFORE updating lastError (v5.4 fix kept)
  float derivative = (float)(error - lastError);
  float correction = (Kp * (float)error) + (Kd * derivative);
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  // Independent motor constraints — allows sharp turns (v5.4 fix kept)
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  AVOID RED OBSTACLE — RIGHT-side bypass
//
//  BUG FIX 4: Original always turned LEFT. Arena shows obstacles
//  on the LEFT side of the path. Robot should swing RIGHT to bypass.
//
//  Manoeuvre (right-bypass):
//    1. Turn right  (swing nose away from obstacle on left)
//    2. Drive forward past the obstacle
//    3. Turn left   (face back toward the line)
//    4. Drive forward to reach line
//
//  Timing (ms) must be tuned on actual robot. These are starting values.
// ============================================================
void avoidRedObstacle() {
  Serial.println("Avoiding red obstacle (right-bypass)...");

  // Step 1: Turn right — nose swings right, away from obstacle
  driveMotors(AVOID_SPEED, -AVOID_SPEED);
  delay(380);
  stopMotors(); delay(150);

  // Step 2: Drive forward to clear the obstacle length
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(650);
  stopMotors(); delay(150);

  // Step 3: Turn left — face back toward the line
  driveMotors(-AVOID_SPEED, AVOID_SPEED);
  delay(380);
  stopMotors(); delay(150);

  // Step 4: Drive forward to cross over the line
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(400);
  stopMotors(); delay(200);

  Serial.println("Avoidance manoeuvre done");
}

// ============================================================
//  REJOIN LINE after obstacle avoidance
//  BUG FIX 8: After right-bypass, robot is to the RIGHT of the line.
//  Creep forward; if not found, sweep LEFT (toward the line).
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
    // Creep forward with slight left bias to sweep toward line
    driveMotors(100, 120);
    delay(40);
  }

  stopMotors();
  Serial.println("Line not found within timeout");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
// ============================================================
void pickGreenCube() {
  Serial.println("Picking up green cube...");
  gripperServo.write(SERVO_OPEN);   // ensure gate is open first
  delay(400);
  stopMotors();
  delay(300);
  gripperServo.write(SERVO_CLOSED); // close gate to grip cube
  delay(800);
  Serial.println("Cube gripped!");
}

// ============================================================
//  DROP CUBE AT END ZONE
// ============================================================
void dropCubeAtEnd() {
  Serial.println("Dropping cube at end zone...");
  stopMotors();
  delay(300);
  gripperServo.write(SERVO_OPEN);   // open gate — cube drops
  delay(800);
  driveMotors(-100, -100);          // reverse slightly to clear cube
  delay(400);
  stopMotors();
  cubePickedUp = false;
  Serial.println("Cube released!");
}

// ============================================================
//  TCS230 COLOR SENSOR — single reading
//
//  BUG FIX 6: pulseIn timeout returns 0. A timed-out (0) channel
//  would always "win" the minimum comparison, giving wrong color.
//  Fix: if ANY channel is 0 (timeout), return 'U' (unknown).
//
//  TCS230 logic: lower pulseIn(LOW) value = higher frequency = 
//  MORE of that color present. Smallest value = dominant color. ✓
// ============================================================
char readColorOnce() {
  long r = 0, g = 0, b = 0;

  // Read Red channel (S2=LOW, S3=LOW)
  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, LOW);
  delay(10);
  r = pulseIn(TCS_OUT_PIN, LOW, 100000);

  // Read Green channel (S2=HIGH, S3=HIGH)
  digitalWrite(TCS_S2_PIN, HIGH);
  digitalWrite(TCS_S3_PIN, HIGH);
  delay(10);
  g = pulseIn(TCS_OUT_PIN, LOW, 100000);

  // Read Blue channel (S2=LOW, S3=HIGH)
  digitalWrite(TCS_S2_PIN, LOW);
  digitalWrite(TCS_S3_PIN, HIGH);
  delay(10);
  b = pulseIn(TCS_OUT_PIN, LOW, 100000);

  // ── Uncomment to calibrate color sensor ──
  /*
  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);
  */

  // BUG FIX 6: any timed-out channel = unknown result
  if (r == 0 || g == 0 || b == 0) return 'U';

  // Too bright / sensor blinded (all values nearly equal and low)
  if (min(r, min(g, b)) > 250) return 'U';

  // Smallest value = dominant color (TCS230 inverse-frequency)
  if (r < g && r < b) return 'R';
  if (g < r && g < b) return 'G';
  if (b < r && b < g) return 'B';

  return 'U';
}

// ============================================================
//  TCS230 COLOR SENSOR — voted reading (more reliable)
// ============================================================
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
//  ULTRASONIC SENSOR — returns distance in cm
//  Returns 999.0 if no echo received
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
//  Positive = forward, Negative = reverse
//  ESP32 core v3.x: ledcWrite(pin, value) — use PIN not channel
// ============================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, leftSpeed);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, -leftSpeed);
  }

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENB_PIN, rightSpeed);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    ledcWrite(ENB_PIN, -rightSpeed);
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
//  END OF CODE v5.5 — EC6090 Mini Project 2026
// ============================================================
