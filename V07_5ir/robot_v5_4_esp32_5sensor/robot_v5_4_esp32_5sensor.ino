/*
 * ============================================================
 *  LINE FOLLOWING ROBOT — ESP32 30-PIN DEVKIT V1 (v5.4)
 * ============================================================
 *  Board: ESP32 DevKit V1 — 30 pin
 *
 *  FIXED in v5.4 (PD line following bugs):
 *    BUG 1 FIXED: Weight scale ×10, Kp/Kd re-tuned to match
 *    BUG 2 FIXED: Lost-line → direct driveMotors() spin, skips PD
 *    BUG 3 FIXED: Removed bad SHARP_SPEED symmetric cap (was blocking turns)
 *    BUG 4 FIXED: lastError now updated AFTER derivative is calculated
 *
 *  THRESHOLD updated:
 *    IR_THRESHOLD 2600 — tuned for grey concrete + dull yellow tape
 *    (TCRT5000 sensors, sensor close to floor)
 *
 *  PIN MAP (30-pin DevKit V1):
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
 *    D33         =  33   →  TCS230 OUT
 *    D35         =  35   →  IR Sensor S1 (Far  Left)  — input only
 *    D34         =  34   →  IR Sensor S2 (Mid  Left)  — input only
 *    VN (39)     =  39   →  IR Sensor S3 (Center)     — input only
 *    VP (36)     =  36   →  IR Sensor S4 (Mid  Right) — input only
 *    D33         =  33   →  IR Sensor S5 (Far  Right)
 *
 *  ⚠️  GPIO 2, 5, 12, 15 avoided (strapping pins — boot issues)
 *  ⚠️  HC-SR04 ECHO needs voltage divider: ECHO→1kΩ→GPIO23, 2kΩ to GND
 *  ⚠️  IR sensors → 3.3V VCC only
 *  ⚠️  Servo, L298N → 5V VCC (Vin pin)
 *
 *  Required library: ESP32Servo (install via Library Manager)
 *  Arduino IDE: Tools → Board → ESP32 Arduino → ESP32 Dev Module
 *
 *  Author: EC6090 Mini Project 2026
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
#define IR_S5   33    // Far  Right

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
#define TCS_OUT_PIN  33

// ============================================================
//  LEDC PWM CONFIG — ESP32 Arduino core v3.x API
// ============================================================
#define PWM_FREQ        5000
#define PWM_RESOLUTION  8       // 8-bit: 0–255
#define SERVO_FREQ      50      // 50Hz for servo

// ============================================================
//  MOTOR TEST MODE — set true to test motors only
// ============================================================
#define MOTOR_TEST_MODE  false

// ============================================================
//  TUNABLE CONSTANTS — adjust these for your arena
// ============================================================

// IR threshold — grey concrete floor + dull yellow tape + TCRT5000 close to floor
// If binary output is wrong: raise toward 3000 (floor too reflective)
//                            lower toward 2000 (line not bright enough)
#define IR_THRESHOLD    2600

// Ultrasonic detection distance (cm)
#define DETECT_DIST     18

// Motor speeds (0–255)
#define BASE_SPEED      150   // normal line following speed
#define SHARP_SPEED      80   // spin speed when line is fully lost
#define AVOID_SPEED     140   // speed during obstacle avoidance manoeuvre
#define CREEP_SPEED     100   // slow approach to green cube

// PD gains — re-tuned for ×10 weight scale (v5.4)
// If robot oscillates on straight: lower Kp toward 1.2
// If robot is sluggish on curves:  raise Kp toward 2.5
// If robot zigzags on straight:    raise Kd toward 1.5
float Kp = 1.2;
float Kd = 1.5;

// Servo positions (degrees)
#define SERVO_OPEN      85    // gate open  — cube can pass/drop
#define SERVO_CLOSED    15    // gate closed — cube held

// Time-based end zone detection (ms) — adjust for your arena length
#define END_ZONE_TRAVEL_MS  8000

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
void readIRSensors();
void followLine();
void avoidRedObstacle();
bool rejoinLine();
void pickGreenCube();
void dropCubeAtEnd();
char readColorOnce();
char readColorVoted(int times);
float readUltrasonic();
void driveMotors(int leftSpeed, int rightSpeed);
void stopMotors();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== Robot v5.4 — PD bugs fixed ===");
  Serial.print("IR_THRESHOLD: "); Serial.println(IR_THRESHOLD);
  Serial.print("Kp: "); Serial.print(Kp);
  Serial.print("  Kd: "); Serial.println(Kd);

  // Motor direction pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // LEDC PWM — core v3.x: ledcAttach(pin, freq, resolution)
  ledcAttach(ENA_PIN,   PWM_FREQ,   PWM_RESOLUTION);
  ledcAttach(ENB_PIN,   PWM_FREQ,   PWM_RESOLUTION);
  ledcAttach(SERVO_PIN, SERVO_FREQ, PWM_RESOLUTION);

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
  // Frequency scaling 20%
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  // IR: pins 35,34,39,36 are input-only — no pinMode() needed
  // Only S5 (GPIO33) is a normal GPIO — needs pinMode
  pinMode(IR_S5, INPUT);

  // Servo — start with gate open
  gripperServo.attach(SERVO_PIN, 500, 2400);
  gripperServo.write(SERVO_OPEN);
  delay(500);

  // Optional motor test
  if (MOTOR_TEST_MODE) {
    Serial.println("=== MOTOR TEST MODE ===");
    Serial.println("Forward...");
    driveMotors(150, 150);  delay(2000);
    Serial.println("Turn left...");
    driveMotors(0, 150);    delay(1500);
    Serial.println("Turn right...");
    driveMotors(150, 0);    delay(1500);
    Serial.println("Reverse...");
    driveMotors(-150,-150); delay(2000);
    stopMotors();
    Serial.println("Done. Set MOTOR_TEST_MODE false to run.");
    while(true);
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

    // ── State: Normal line following ──
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

    // ── State: Read color of detected object ──
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
        // Unknown or second green (already picked) — treat as obstacle
        Serial.println("→ Unknown/extra — avoiding");
        currentState = STATE_AVOID_RED;
      }
      break;
    }

    // ── State: Navigate around red obstacle ──
    case STATE_AVOID_RED: {
      avoidRedObstacle();
      obstaclesAvoided++;
      Serial.print("Obstacles avoided so far: ");
      Serial.println(obstaclesAvoided);
      currentState = STATE_REJOIN_LINE;
      break;
    }

    // ── State: Creep forward until line is found ──
    case STATE_REJOIN_LINE: {
      bool found = rejoinLine();
      if (found) {
        currentState = STATE_LINE_FOLLOW;
      } else {
        // Timed out — try a small spin and look again
        Serial.println("Rejoin timed out — trying spin recovery");
        driveMotors(-100, 100);
        delay(400);
        currentState = STATE_REJOIN_LINE;
      }
      break;
    }

    // ── State: Slow approach to green cube ──
    case STATE_APPROACH_GREEN: {
      driveMotors(CREEP_SPEED, CREEP_SPEED);
      delay(400);
      stopMotors();
      delay(200);
      currentState = STATE_PICK_GREEN;
      break;
    }

    // ── State: Close servo to grip cube ──
    case STATE_PICK_GREEN: {
      pickGreenCube();
      cubePickedUp = true;
      pickUpTime   = millis();
      currentState = STATE_CARRY_TO_END;
      break;
    }

    // ── State: Follow line while carrying cube ──
    case STATE_CARRY_TO_END: {
      // Time-based end zone trigger
      if (millis() - pickUpTime > END_ZONE_TRAVEL_MS) {
        stopMotors();
        Serial.println("End zone reached (timeout)");
        currentState = STATE_DROP_CUBE;
        break;
      }
      // Wall/end detection via ultrasonic
      float dist = readUltrasonic();
      if (dist > 0 && dist < 8) {
        stopMotors();
        Serial.println("End zone reached (ultrasonic)");
        currentState = STATE_DROP_CUBE;
        break;
      }
      followLine();
      break;
    }

    // ── State: Open servo to release cube ──
    case STATE_DROP_CUBE: {
      dropCubeAtEnd();
      currentState = STATE_DONE;
      break;
    }

    // ── State: Task complete ──
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
    // HIGH raw = more reflection = on the yellow line (TCRT5000 on dark/grey floor)
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // ── Uncomment to debug in Serial Monitor ──
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
//  LINE FOLLOWING — PD Controller (FIXED v5.4)
//
//  BUG 3 FIX: Removed SHARP_SPEED symmetric cap.
//             Motors now turn freely — one fast, one slow/reverse.
//  BUG 4 FIX: Derivative is calculated BEFORE lastError is updated.
//  BUG 1 FIX: Weights scaled ×10, error is averaged — wider dynamic range.
//  BUG 2 FIX: Lost-line → direct driveMotors() spin, bypasses PD entirely.
// ============================================================
void followLine() {
  // Weights ×10 give a wider error range (-20 to +20)
  // Dividing by sensorSum gives a weighted average position
  int weights[5] = {-20, -10, 0, 10, 20};
  int weightedSum = 0;
  int sensorSum   = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  // ── Lost-line recovery (all sensors off line) ──
  if (sensorSum == 0) {
    // Spin toward last known direction — bypass PD
    if (lastError > 0) {
      driveMotors(BASE_SPEED, -SHARP_SPEED);  // line was right → spin right
    } else {
      driveMotors(-SHARP_SPEED, BASE_SPEED);  // line was left  → spin left
    }
    return;
  }

  // Weighted average position: negative = line left, positive = line right
  int error = weightedSum / sensorSum;

  // ── BUG 4 FIX: compute derivative BEFORE updating lastError ──
  float derivative = (float)(error - lastError);

  // PD correction
  float correction = (Kp * (float)error) + (Kd * derivative);

  // Update lastError AFTER derivative is calculated
  lastError = error;

  int leftSpeed  = BASE_SPEED - (int)correction;
  int rightSpeed = BASE_SPEED + (int)correction;

  // ── BUG 3 FIX: constrain each motor independently (no symmetric cap) ──
  // Negative values = motor runs in reverse = sharp turn
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  driveMotors(leftSpeed, rightSpeed);
}

// ============================================================
//  AVOID RED OBSTACLE (fixed U-shape manoeuvre)
//  Timing may need adjustment based on your robot's actual speed
// ============================================================
void avoidRedObstacle() {
  Serial.println("Avoiding red obstacle...");

  // Step 1: Turn left (away from obstacle)
  driveMotors(-AVOID_SPEED, AVOID_SPEED);
  delay(380);
  stopMotors(); delay(150);

  // Step 2: Drive forward past the obstacle
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(650);
  stopMotors(); delay(150);

  // Step 3: Turn right to face the line again
  driveMotors(AVOID_SPEED, -AVOID_SPEED);
  delay(380);
  stopMotors(); delay(150);

  // Step 4: Small forward push to get over the line
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(400);
  stopMotors(); delay(200);

  Serial.println("Avoidance manoeuvre done");
}

// ============================================================
//  REJOIN LINE after obstacle avoidance
//  Creeps forward until any sensor detects the line (max 4s)
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
  Serial.println("Line not found within timeout");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE
//  Robot stops, closes the drop gate servo to hold the cube
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
//  Stops, opens gate to release cube, backs away slightly
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
//  TCS230: lower pulse width = higher frequency = more of that color
//  So a smaller pulseIn value means that color dominates
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

  // Timeout / no signal
  if (r == 0 && g == 0 && b == 0) return 'U';

  // Too bright / sensor blinded
  if (min(r, min(g, b)) > 250) return 'U';

  // Smallest value = dominant color (TCS230 = inverse frequency)
  if (r < g && r < b) return 'R';
  if (g < r && g < b) return 'G';
  if (b < r && b < g) return 'B';

  return 'U';
}

// ============================================================
//  TCS230 COLOR SENSOR — voted reading (more reliable)
//  Reads multiple times and returns the majority color
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
//  Returns 999.0 if no echo received (nothing in range)
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
//  Positive value = forward, negative value = reverse
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
//  END OF CODE v5.4 — EC6090 Mini Project 2026
// ============================================================
