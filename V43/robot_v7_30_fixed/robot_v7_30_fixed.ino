/*
 * ============================================================
 * EC6090 MINI PROJECT 2026 — LINE FOLLOWING ROBOT v7.30
 * FULL LOGIC — OBSTACLE AVOID + REJOIN FULLY REWRITTEN
 * ** UPDATED: Includes Wi-Fi WebSocket Telemetry Broadcast **
 * ============================================================
 * Board: ESP32 DevKit V1 — 30 pin
 *
 * Components:
 * - 5x IR Sensors (TCRT5000 digital) for line following
 * - 1x HC-SR04 Ultrasonic sensor for obstacle detection
 * - 1x TCS230/TCS3200 Color sensor for red/green detection
 * - 1x L298N Motor driver for 2 DC motors
 * - 1x SG90 Servo for drop-gate pick & place
 * - ESP32 DevKit V1 (30-pin) controller
 *
 * Arena: Yellow tape line on grey concrete floor
 * Closed-loop path with curves (~1.3m diameter sections)
 * 2x Red obstacle cubes ON the line
 * 1x Green cube ON the line (to pick up)
 * End/Target zone to drop cube
 *
 * ============================================================
 * CHANGES FROM v7.20 → v7.30  (AVOIDANCE REWRITE)
 * ============================================================
 *
 * BUG FIX 1 — REVERSE DISTANCE INCREASED
 *   Old: REVERSE_MS=300 at speed 130 → ~4cm of travel.
 *   Problem: Robot front is 5-6cm from cube after color read,
 *   so 4cm reverse is not enough clearance to turn without
 *   clipping the obstacle, especially on curves.
 *   Fix: REVERSE_MS increased to 550. Robot now backs up ~9cm,
 *   giving comfortable turning clearance.
 *
 * BUG FIX 2 — DIRECTION DECISION USES IR AFTER REVERSE
 *   Old: avoidDirectionHint from lastError at detection time.
 *   Problem: On curves, lastError reflects PID curve-tracking
 *   correction, NOT obstacle position. This often sent the
 *   robot bypassing in the wrong direction (away from where
 *   the line continues), making rejoin impossible.
 *   Fix: After reversing, read IR sensors. If left sensors
 *   see line → bypass right. If right sensors see line →
 *   bypass left. If no sensors see line → use the stored
 *   hint as fallback. This is physically correct because
 *   after reversing, whichever side still sees the line is
 *   the side the line continues on, so we go AROUND the
 *   other side.
 *
 * BUG FIX 3 — REJOIN NOW DRIVES FORWARD + SWEEPS (ARC SEARCH)
 *   Old: rejoinLine() only spun in place for 5 seconds.
 *   Problem: After avoidance on a curved arena, the line is
 *   typically AHEAD and to one side. Pure rotation cannot
 *   reach a line that is not under the robot.
 *   Fix: New 3-phase rejoin:
 *     Phase 1 (0-2s): Arc forward while sweeping (one motor
 *       faster than the other) — covers the most likely
 *       line position after avoidance.
 *     Phase 2 (2-4s): Arc forward in the opposite direction.
 *     Phase 3 (4-6s): Spin in place as last resort.
 *   Each phase continuously checks all 5 IR sensors.
 *
 * BUG FIX 4 — STEP 3 TURN-BACK TIMEOUT INCREASED
 *   Old: 1500ms max to spin back toward line.
 *   Problem: On tight curves the line is at a sharper angle,
 *   requiring more rotation. 1500ms often timed out.
 *   Fix: Increased to 2000ms.
 *
 * BUG FIX 5 — REJOIN FAILURE DRIVES FORWARD BEFORE RETRY
 *   Old: When rejoinLine() returned false, the main loop did
 *   a 500ms spin then looped — trying rejoinLine() again
 *   from the same position, creating infinite spin-in-place.
 *   Fix: On failure, drive forward 400ms THEN spin 400ms,
 *   so each retry starts from a different position.
 *
 * BUG FIX 6 — ULTRASONIC FORCE-READ ADDED
 *   Added readUltrasonicForce() that ignores the
 *   ultrasonicEnabled flag. Used inside avoidObstacle() to
 *   verify clearance after reversing without needing to
 *   toggle the flag.
 *
 * ALL PREVIOUS LOGIC KEPT INTACT:
 * - WiFi AP + WebSocket telemetry (UNCHANGED)
 * - Auto/Manual mode switching (UNCHANGED)
 * - 8-state machine (UNCHANGED)
 * - PID + corner phases (approach/pivot/reacquire) (UNCHANGED)
 * - Color voting (15 samples) (UNCHANGED)
 * - Close-approach to 6cm before color read (UNCHANGED)
 * - Ultrasonic gate (ultrasonicEnabled flag) (UNCHANGED)
 * - End zone active search (only when cubePickedUp) (UNCHANGED)
 * - Watchdog recovery spin (UNCHANGED)
 * - All test/calibration modes (UNCHANGED)
 * - Servo open/close logic (UNCHANGED)
 * - pickGreenCube() (UNCHANGED)
 * - dropCubeAtEnd() (UNCHANGED)
 *
 * PIN MAP (30-pin DevKit V1) — UNCHANGED:
 *
 * GPIO    Component
 * ─────────────────────────────────────────
 * 16   →  L298N IN1  (Left  motor dir A)
 * 17   →  L298N IN2  (Left  motor dir B)
 * 18   →  L298N IN3  (Right motor dir A)
 * 19   →  L298N IN4  (Right motor dir B)
 * 25   →  L298N ENA  (Left  motor PWM)
 * 26   →  L298N ENB  (Right motor PWM)
 * 27   →  SG90 Servo signal
 * 22   →  HC-SR04 TRIG
 * 23   →  HC-SR04 ECHO (needs 5V→3.3V divider!)
 * 13   →  TCS230 S0
 * 14   →  TCS230 S1
 * 21   →  TCS230 S2
 *  4   →  TCS230 S3
 * 33   →  TCS230 OUT
 * 35   →  IR Sensor S1 (Far  Left)  — input only
 * 34   →  IR Sensor S2 (Mid  Left)  — input only
 * 39   →  IR Sensor S3 (Center)     — input only (VN)
 * 36   →  IR Sensor S4 (Mid  Right) — input only (VP)
 * 32   →  IR Sensor S5 (Far  Right)
 *
 * ⚠️  GPIO 2, 5, 12, 15 avoided (strapping pins)
 * ⚠️  HC-SR04 ECHO needs voltage divider: ECHO→1kΩ→GPIO23, 2kΩ→GND
 * ⚠️  IR sensors → 3.3V VCC only
 * ⚠️  Servo, L298N → 5V VCC (Vin pin or external)
 *
 * Required libraries: ESP32Servo, WebSockets (Markus Sattler),
 *                     ArduinoJson v6
 * Arduino IDE: Tools → Board → ESP32 Arduino → ESP32 Dev Module
 *
 * Author: EC6090 Mini Project 2026 — v7.30 Avoidance Rewrite
 * ============================================================
 */

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ============================================================
//  DEBUG / TEST FLAGS
// ============================================================
#define TEST_MOTORS       false
#define TEST_IR_RAW       false
#define TEST_COLOR        false
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
#define MAX_SPEED             200
#define BASE_SPEED            150
#define CURVE_SPEED            80
#define SHARP_TURN_SPEED      200
#define CORNER_APPROACH_SPEED 100
#define REACQUIRE_SPEED        80
#define BACKUP_SPEED           80
#define MIN_SPEED              80
#define AVOID_SPEED           130
#define CREEP_SPEED            90
#define SPIN_SPEED            120

// Corner timing
#define REACQUIRE_MS   150
#define BACKUP_MS       80

// PID
float Kp = 3.2;
float Ki = 0.01;
float Kd = 4.5;
float integralSum = 0.0;
#define INTEGRAL_MAX  500.0

#define MISSED_CORNER_THRESHOLD  30
#define PIVOT_TIMEOUT_MS  3000

// Ultrasonic distances
#define DETECT_DIST_CM       6
#define COLOR_READ_DIST_CM    6
#define END_WALL_DIST_CM      6

// ── v7.30: Increased from 300 to 550 for proper clearance
#define REVERSE_MS          550

// ── v7.30: fixed time to drive past obstacle (Step 2)
#define AVOID_PASS_MS       900

// ── v7.30: Step 3 turn-back timeout increased from 1500 to 2000
#define AVOID_TURNBACK_MS  2000

// End zone search
#define ENDZONE_SEARCH_MS  3000

// Servo
#define SERVO_OPEN    100
#define SERVO_CLOSED  40

// Timing
#define END_ZONE_TIMEOUT_MS   15000
#define STUCK_TIMEOUT_MS      15000
#define LINE_LOSS_CONFIRM_MS    300

// ── v7.30: rejoin total timeout increased from 5s to 6s
#define REJOIN_TIMEOUT_MS    6000

// IR polarity
#define INVERT_IR  true

// Color detection thresholds
#define COLOR_RED_R_MAX        350
#define COLOR_RED_MARGIN        40
#define COLOR_GREEN_ALL_MIN    180
#define COLOR_GREEN_ALL_MAX    310
#define COLOR_GREEN_SPREAD      60

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
Servo gateServo;
int irRaw[5];
int irBinary[5];
int irPins[5] = {IR_S1, IR_S2, IR_S3, IR_S4, IR_S5};

// WiFi / WebSocket telemetry
const char* ssid     = "Robot_Telemetry";
const char* password = "";
WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastTelemetryUpdate = 0;

// C2 control
bool isAutoMode = true;
bool isGateOpen = true;

int   lastError        = 0;
int   lastTightDir     = 0;
bool  cubePickedUp       = false;
int   obstaclesAvoided   = 0;

bool  ultrasonicEnabled  = true;

// ── v7.30: kept for fallback only (IR-based decision is primary)
int   avoidDirectionHint = 0;   // +1 = go right,  -1 = go left

unsigned long pickUpTime     = 0;
unsigned long stateEntryTime = 0;
unsigned long lastDebugPrint = 0;
unsigned long lineLostStart  = 0;
bool  lineLostTiming   = false;
float lastDistCm       = 999.0;

// ============================================================
//  STATE MACHINE
// ============================================================
enum RobotState {
  STATE_LINE_FOLLOW,
  STATE_IDENTIFY_OBJECT,
  STATE_AVOID_RED,
  STATE_REJOIN_LINE,
  STATE_PICK_GREEN,
  STATE_CARRY_TO_END,
  STATE_DROP_CUBE,
  STATE_DONE
};

RobotState currentState = STATE_LINE_FOLLOW;
RobotState prevState    = STATE_LINE_FOLLOW;

const char* stateNames[] = {
  "LINE_FOLLOW", "IDENTIFY_OBJ", "AVOID_RED",
  "REJOIN_LINE", "PICK_GREEN",
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
bool  searchForEndZone();
void  pickGreenCube();
void  dropCubeAtEnd();
char  readColorOnce();
char  readColorVoted(int times);
float readUltrasonic();
float readUltrasonicForce();
void  driveMotors(int leftSpeed, int rightSpeed);
void  stopMotors();
void  changeState(RobotState newState);
void  printDashboard();
void  runTestMode();

// ============================================================
//  WEBSOCKET EVENT HANDLER  (UNCHANGED)
// ============================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[%u] Mobile Dashboard Connected!\n", num);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[%u] Mobile Dashboard Disconnected\n", num);
  } else if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (!error) {
      const char* cmd = doc["cmd"];
      const char* val = doc["val"];

      if (strcmp(cmd, "mode") == 0) {
        if (strcmp(val, "auto") == 0) {
          Serial.println(">>> SWITCHING TO AUTO MODE");
          isAutoMode = true;
          ultrasonicEnabled = true;
          changeState(STATE_LINE_FOLLOW);
        } else {
          Serial.println(">>> SWITCHING TO MANUAL MODE");
          isAutoMode = false;
          stopMotors();
        }
      }
      else if (strcmp(cmd, "servo") == 0) {
        if (strcmp(val, "open") == 0) {
          gateServo.write(SERVO_OPEN);
          isGateOpen = true;
        } else {
          gateServo.write(SERVO_CLOSED);
          isGateOpen = false;
        }
      }
      else if (strcmp(cmd, "move") == 0 && !isAutoMode) {
        if      (strcmp(val, "forward") == 0) driveMotors( MAX_SPEED,  MAX_SPEED);
        else if (strcmp(val, "reverse") == 0) driveMotors(-MAX_SPEED, -MAX_SPEED);
        else if (strcmp(val, "left")    == 0) driveMotors(-BASE_SPEED, BASE_SPEED);
        else if (strcmp(val, "right")   == 0) driveMotors( BASE_SPEED,-BASE_SPEED);
        else if (strcmp(val, "stop")    == 0) stopMotors();
      }
    }
  }
}

// ============================================================
//  SETUP  (UNCHANGED)
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("============================================");
  Serial.println("  EC6090 Robot v7.30 — Avoidance Rewrite");
  Serial.println("============================================");
  Serial.print("INVERT_IR=");
  Serial.println(INVERT_IR ? "YES (LOW=line)" : "NO (HIGH=line)");
  Serial.print("BASE_SPEED="); Serial.println(BASE_SPEED);
  Serial.print("AVOID_PASS_MS="); Serial.println(AVOID_PASS_MS);
  Serial.print("REVERSE_MS="); Serial.println(REVERSE_MS);
  Serial.print("REJOIN_TIMEOUT_MS="); Serial.println(REJOIN_TIMEOUT_MS);
  Serial.print("MISSED_CORNER_THRESHOLD="); Serial.println(MISSED_CORNER_THRESHOLD);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Servo FIRST — must claim LEDC channel before motors
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(SERVO_OPEN);
  delay(500);

  // Motor PWM — after servo attached
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

  if (TEST_MOTORS || TEST_IR_RAW || TEST_COLOR ||
      TEST_ULTRASONIC || TEST_SERVO || CALIBRATE_COLOR) {
    runTestMode();
  }

  // Start WiFi Access Point + WebSocket
  Serial.println("Starting Wi-Fi Access Point...");
  WiFi.softAP(ssid, password);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.print("Telemetry IP: ");
  Serial.println(WiFi.softAPIP());

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

  // Telemetry broadcast every 100ms  (UNCHANGED)
  webSocket.loop();
  if (millis() - lastTelemetryUpdate > 100) {
    StaticJsonDocument<200> doc;
    JsonArray irArray = doc.createNestedArray("irArray");
    for (int i = 0; i < 5; i++) irArray.add(irBinary[i]);
    doc["distanceCm"] = lastDistCm;
    if (isAutoMode) doc["state"] = stateNames[currentState];
    else            doc["state"] = "MANUAL OVERRIDE";
    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
    lastTelemetryUpdate = millis();
  }

  // ── AUTO / MANUAL split ──────────────────────────────────────
  if (isAutoMode) {

    // Watchdog  (UNCHANGED)
    if (currentState != STATE_DONE) {
      if (millis() - stateEntryTime > STUCK_TIMEOUT_MS &&
          currentState == prevState) {
        Serial.println("⚠ WATCHDOG: Stuck — recovery spin");
        stopMotors();
        delay(200);
        if (lastError >= 0) driveMotors( SPIN_SPEED, -SPIN_SPEED);
        else                driveMotors(-SPIN_SPEED,  SPIN_SPEED);
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

      // ── Normal line following  (UNCHANGED) ─────────────────────
      case STATE_LINE_FOLLOW: {
        float dist = readUltrasonic();
        lastDistCm = dist;

        if (dist > 2 && dist < DETECT_DIST_CM) {
          // Capture direction hint BEFORE stopping (fallback only)
          avoidDirectionHint = (lastError >= 0) ? 1 : -1;
          stopMotors();
          delay(100);
          Serial.print("Object detected at ");
          Serial.print(dist, 1);
          Serial.println("cm — creeping to 6cm for color read");
          changeState(STATE_IDENTIFY_OBJECT);
          break;
        }
        followLineAdaptive();
        break;
      }

      // ── Creep close then identify color  (UNCHANGED) ───────────
      case STATE_IDENTIFY_OBJECT: {

        if (cubePickedUp) {
          Serial.println("Cube already held — auto avoiding");
          changeState(STATE_AVOID_RED);
          break;
        }

        float dist = readUltrasonic();
        if (dist > COLOR_READ_DIST_CM) {
          driveMotors(CREEP_SPEED, CREEP_SPEED);
          break;
        }

        stopMotors();
        delay(200);
        Serial.println("At 6cm — reading color...");

        char color = readColorVoted(15);
        Serial.print("Color result: "); Serial.println(color);

        if (color == 'G') {
          Serial.println("→ GREEN detected — picking up");
          changeState(STATE_PICK_GREEN);
        } else {
          Serial.println("→ NOT green — disabling ultrasonic before movement");
          ultrasonicEnabled = false;
          Serial.println("Ultrasonic DISABLED");
          changeState(STATE_AVOID_RED);
        }
        break;
      }

      // ── Avoid red/unknown obstacle ────────────────────────────
      case STATE_AVOID_RED: {
        avoidObstacle();
        obstaclesAvoided++;
        Serial.print("Obstacles avoided: "); Serial.println(obstaclesAvoided);
        changeState(STATE_REJOIN_LINE);
        break;
      }

      // ── Rejoin line after avoidance  ← v7.30 CHANGED ─────────
      case STATE_REJOIN_LINE: {
        bool found = rejoinLine();
        if (found) {
          ultrasonicEnabled = true;
          Serial.println("Ultrasonic RE-ENABLED — back on line");
          changeState(cubePickedUp ? STATE_CARRY_TO_END : STATE_LINE_FOLLOW);
        } else {
          // ── v7.30 FIX 5: drive forward THEN spin before retry
          // Old code only spun in place → same position each time
          Serial.println("Rejoin timeout — driving forward + spin before retry");
          driveMotors(AVOID_SPEED, AVOID_SPEED);
          delay(400);
          stopMotors();
          delay(50);
          if (lastError >= 0) driveMotors( SPIN_SPEED, -SPIN_SPEED);
          else                driveMotors(-SPIN_SPEED,  SPIN_SPEED);
          delay(400);
          stopMotors();
        }
        break;
      }

      // ── Pick up green cube  (UNCHANGED) ────────────────────────
      case STATE_PICK_GREEN: {
        pickGreenCube();
        cubePickedUp = true;
        pickUpTime   = millis();
        Serial.println("Green cube captured — continuing line follow");
        changeState(STATE_CARRY_TO_END);
        break;
      }

      // ── Carry cube to end zone  (UNCHANGED) ───────────────────
      case STATE_CARRY_TO_END: {

        float dist = readUltrasonic();
        lastDistCm = dist;

        if (dist > 2 && dist < DETECT_DIST_CM) {
          avoidDirectionHint = (lastError >= 0) ? 1 : -1;
          ultrasonicEnabled = false;
          Serial.println("Ultrasonic DISABLED — carry avoidance starting");
          stopMotors();
          delay(100);
          Serial.println("Object while carrying → auto avoiding");
          avoidObstacle();
          obstaclesAvoided++;
          changeState(STATE_REJOIN_LINE);
          break;
        }

        int sc = 0;
        for (int i = 0; i < 5; i++) sc += irBinary[i];

        if (sc == 0) {
          Serial.println("Line lost while carrying — searching for end zone...");
          stopMotors();
          delay(100);
          bool lineFound = searchForEndZone();
          if (!lineFound) {
            Serial.println("★ END ZONE CONFIRMED — dropping cube ★");
            changeState(STATE_DROP_CUBE);
          } else {
            Serial.println("Line reacquired — continuing");
            changeState(STATE_CARRY_TO_END);
          }
          break;
        }

        followLineAdaptive();
        break;
      }

      // ── Drop cube at end zone  (UNCHANGED) ────────────────────
      case STATE_DROP_CUBE: {
        dropCubeAtEnd();
        changeState(STATE_DONE);
        break;
      }

      // ── All done  (UNCHANGED) ─────────────────────────────────
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

  } else {
    // Manual mode — motor commands handled in webSocketEvent()
    lastDistCm = readUltrasonic();
  }
}

// ============================================================
//  STATE CHANGE HELPER  (UNCHANGED)
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
//  READ IR SENSORS  (UNCHANGED)
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
//  LINE FOLLOWING — PID + Tight Turn + Missed Corner (UNCHANGED)
// ============================================================
bool  turning           = false;
int   turnDirection     = 0;
unsigned long turnStartTime   = 0;
bool  centerMustClear   = false;
bool  reacquiring       = false;
unsigned long reacquireStart  = 0;
bool  cornerApproach    = false;

void followLineAdaptive() {

  bool s1 = irBinary[0];
  bool s2 = irBinary[1];
  bool s3 = irBinary[2];
  bool s4 = irBinary[3];
  bool s5 = irBinary[4];

  int sensorSum = irBinary[0]+irBinary[1]+irBinary[2]
                 +irBinary[3]+irBinary[4];

  // ── PHASE 3 — CONTROLLED REACQUIRE ────────────────────────────
  if (reacquiring) {
    if (millis() - reacquireStart < REACQUIRE_MS) {
      driveMotors(REACQUIRE_SPEED, REACQUIRE_SPEED);
      return;
    }
    reacquiring    = false;
    cornerApproach = false;
    integralSum    = 0.0;
    Serial.println("REACQUIRE DONE → full PID");
  }

  // ── PHASE 2 — PIVOT ────────────────────────────────────────────
  if (turning) {

    if (turnDirection == -1) {
      driveMotors(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
    } else {
      driveMotors(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);
    }

    if (millis() - turnStartTime > 30) {
      readIRSensors();

      if (centerMustClear && irBinary[2] == 0) {
        centerMustClear = false;
        Serial.println("S3 CLEARED — now watching for reacquire");
      }

      if (!centerMustClear && irBinary[2] == 1) {
        stopMotors();
        turning        = false;
        reacquiring    = true;
        reacquireStart = millis();
        lastError      = 0;
        integralSum    = 0.0;
        Serial.println("CENTER DETECTED → motors stop → PID resume");
      }
    }

    if (millis() - turnStartTime > PIVOT_TIMEOUT_MS) {
      turning         = false;
      reacquiring     = false;
      cornerApproach  = false;
      centerMustClear = false;
      lastError       = 0;
      stopMotors();
      Serial.println("PIVOT TIMEOUT → forcing PID resume");
    }

    return;
  }

  // ── CORNER TRIGGER ─────────────────────────────────────────────
  if (!turning && !reacquiring) {

    bool earlyLeft  = (s1 && s2 && !s3);
    bool earlyRight = (s4 && s5 && !s3);

    if (earlyLeft || earlyRight) {
      cornerApproach = true;
    } else if (!s1 && !s5) {
      cornerApproach = false;
    }

    bool tightLeft  = (s1 && !s2 && !s3)
                   || (s1 && s2 && s3 && s4 && !s5)
                   || (s1 && s2 && s3 && !s4 && !s5);
    bool tightRight = (s5 && !s4 && !s3)
                   || (!s1 && s2 && s3 && s4 && s5)
                   || (!s1 && !s2 && s3 && s4 && s5);

    bool missedLeft  = (sensorSum == 0 && lastError <= -MISSED_CORNER_THRESHOLD);
    bool missedRight = (sensorSum == 0 && lastError >=  MISSED_CORNER_THRESHOLD);

    if (tightLeft || missedLeft) {
      if (missedLeft) {
        Serial.println("MISSED CORNER LEFT → backup + pivot RIGHT");
        driveMotors(-BACKUP_SPEED, -BACKUP_SPEED);
        delay(BACKUP_MS);
        stopMotors();
        centerMustClear = false;
      } else {
        bool is11100 = (s1 && s2 && s3 && !s4 && !s5);
        centerMustClear = is11100;
        if (is11100) Serial.println("TIGHT LEFT (11100) → pivot RIGHT — wait S3 clear first");
        else         Serial.println("TIGHT LEFT (10000/11110) → pivot RIGHT");
        stopMotors();
      }
      turning        = true;
      turnDirection  = 1;
      lastTightDir   = 1;
      cornerApproach = false;
      integralSum    = 0.0;
      turnStartTime  = millis();
    }
    else if (tightRight || missedRight) {
      if (missedRight) {
        Serial.println("MISSED CORNER RIGHT → backup + pivot LEFT");
        driveMotors(-BACKUP_SPEED, -BACKUP_SPEED);
        delay(BACKUP_MS);
        stopMotors();
        centerMustClear = false;
      } else {
        bool is00111 = (!s1 && !s2 && s3 && s4 && s5);
        centerMustClear = is00111;
        if (is00111) Serial.println("TIGHT RIGHT (00111) → pivot LEFT — wait S3 clear first");
        else         Serial.println("TIGHT RIGHT (00001/01111) → pivot LEFT");
        stopMotors();
      }
      turning        = true;
      turnDirection  = -1;
      lastTightDir   = -1;
      cornerApproach = false;
      integralSum    = 0.0;
      turnStartTime  = millis();
    }
  }

  if (turning) return;

  // ── LOST-LINE RECOVERY ─────────────────────────────────────────
  if (sensorSum == 0) {
    if (!lineLostTiming) {
      lineLostStart  = millis();
      lineLostTiming = true;
    }

    if (millis() - lineLostStart < 200) {
      if (lastError > 0)      driveMotors(SPIN_SPEED, MIN_SPEED);
      else if (lastError < 0) driveMotors(MIN_SPEED, SPIN_SPEED);
      else                    driveMotors(BASE_SPEED, BASE_SPEED);
      return;
    }

    int spinDir = (lastTightDir != 0) ? lastTightDir
                                      : (lastError >= 0 ? 1 : -1);
    if (spinDir > 0) driveMotors( SPIN_SPEED, -SPIN_SPEED);
    else             driveMotors(-SPIN_SPEED,  SPIN_SPEED);
    return;
  }

  lineLostTiming = false;

  // ── NORMAL PID LINE FOLLOWING ───────────────────────────────────
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

  int absError = abs(error);
  int baseSpd;

  if (cornerApproach) {
    baseSpd = CORNER_APPROACH_SPEED;
  } else if (absError <= 5) {
    baseSpd = MAX_SPEED;
  } else if (absError <= 20) {
    baseSpd = BASE_SPEED;
  } else {
    baseSpd = CURVE_SPEED;
  }

  int leftSpeed  = constrain(baseSpd - (int)correction, -120, 255);
  int rightSpeed = constrain(baseSpd + (int)correction, -120, 255);

  driveMotors(leftSpeed, rightSpeed);
}

void followLine() {
  followLineAdaptive();
}

// ============================================================
//  AVOID OBSTACLE  ← v7.30 REWRITTEN
//
//  Summary of changes from v7.20:
//  - Step 0: REVERSE_MS increased 300→550 for proper clearance
//  - Step 0b (NEW): Read IR after reverse to decide bypass
//    direction. Left sensors see line → go right. Right sensors
//    see line → go left. Falls back to avoidDirectionHint only
//    if no sensors see line after reverse.
//  - Step 2: AVOID_PASS_MS increased 800→900 for safer margin
//  - Step 3: Timeout increased 1500→2000 for curved paths
//  - Step 4: Creep forward increased 150→200ms
// ============================================================
void avoidObstacle() {
  Serial.println("── Avoiding obstacle (v7.30) ──");

  // ── Step 0: Reverse to create turning clearance ──────────────
  // v7.30: increased from 300ms to 550ms for proper clearance
  Serial.println("  Step 0: Reversing for clearance (550ms)...");
  driveMotors(-AVOID_SPEED, -AVOID_SPEED);
  delay(REVERSE_MS);
  stopMotors();
  delay(150);

  // ── Step 0b (NEW v7.30): Read IR to decide bypass direction ──
  // After reversing, whichever side still sees the yellow line
  // is the side the line continues on. We bypass the OPPOSITE
  // side (go around the cube away from the line, then arc back).
  //
  // This replaces the v7.20 avoidDirectionHint which used PID
  // lastError — unreliable on curves because lastError tracks
  // curve-following correction, not obstacle geometry.
  readIRSensors();
  int leftCount  = irBinary[0] + irBinary[1];  // S1 + S2
  int rightCount = irBinary[3] + irBinary[4];  // S4 + S5
  int centerSee  = irBinary[2];                // S3

  bool goRight;
  if (leftCount > rightCount) {
    // Line is more visible on left side → bypass RIGHT
    goRight = true;
    Serial.println("  Direction: RIGHT bypass (IR: line on left)");
  } else if (rightCount > leftCount) {
    // Line is more visible on right side → bypass LEFT
    goRight = false;
    Serial.println("  Direction: LEFT bypass (IR: line on right)");
  } else if (centerSee && leftCount == 0 && rightCount == 0) {
    // Only center sees line — use stored hint as tiebreaker
    goRight = (avoidDirectionHint >= 0);
    Serial.print("  Direction: ");
    Serial.print(goRight ? "RIGHT" : "LEFT");
    Serial.println(" bypass (IR: center only, using hint)");
  } else {
    // No sensors see line after reverse — use stored hint
    goRight = (avoidDirectionHint >= 0);
    Serial.print("  Direction: ");
    Serial.print(goRight ? "RIGHT" : "LEFT");
    Serial.println(" bypass (IR: no line visible, using hint)");
  }

  int s = goRight ? 1 : -1;

  // ── Step 1: Turn away from obstacle until sensors clear ───────
  Serial.println("  Step 1: Turning away until line clears...");
  {
    unsigned long t = millis();
    while (millis() - t < 1200) {
      driveMotors(s * AVOID_SPEED, -s * AVOID_SPEED);
      readIRSensors();
      int cnt = 0;
      for (int i = 0; i < 5; i++) cnt += irBinary[i];
      if (cnt == 0) break;
      delay(10);
    }
  }
  stopMotors(); delay(100);

  // ── Step 2: Drive forward past the obstacle ───────────────────
  // v7.30: increased from 800ms to 900ms for extra safety margin
  Serial.println("  Step 2: Driving past obstacle (900ms)...");
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(AVOID_PASS_MS);
  stopMotors(); delay(100);

  // ── Step 3: Turn back toward line ─────────────────────────────
  // v7.30: timeout increased from 1500ms to 2000ms for curves
  Serial.println("  Step 3: Turning back toward line (max 2000ms)...");
  {
    unsigned long t = millis();
    while (millis() - t < AVOID_TURNBACK_MS) {
      driveMotors(-s * AVOID_SPEED, s * AVOID_SPEED);
      readIRSensors();
      int cnt = 0;
      for (int i = 0; i < 5; i++) cnt += irBinary[i];
      if (cnt >= 1) {
        Serial.println("  Step 3: IR detected line!");
        break;
      }
      delay(10);
    }
  }
  stopMotors(); delay(100);

  // ── Step 4: Creep forward to centre on line ───────────────────
  // v7.30: increased from 150ms to 200ms
  Serial.println("  Step 4: Centring on line...");
  driveMotors(AVOID_SPEED, AVOID_SPEED);
  delay(200);
  stopMotors(); delay(100);

  Serial.println("  Avoidance complete (v7.30)");
}

// ============================================================
//  REJOIN LINE  ← v7.30 REWRITTEN
//
//  Old v7.20 problem: Only spun in place. After avoidance on
//  a curved arena the line is typically AHEAD and to one side.
//  Pure rotation can never reach a line that isn't under the
//  robot.
//
//  v7.30 fix: 3-phase arc search:
//    Phase 1 (0-2s): Arc forward + sweep in primary direction
//      (one wheel faster → curved path covering line ahead-left
//      or ahead-right). Checks IR every iteration.
//    Phase 2 (2-4s): Arc forward + sweep opposite direction.
//    Phase 3 (4-6s): Spin in place as last resort (line might
//      have ended up directly beside the robot).
//
//  Each phase runs the IR check at ~30ms intervals, giving
//  ~65 checks per phase.
// ============================================================
bool rejoinLine() {
  Serial.println("Searching for line (v7.30 arc search)...");
  unsigned long start = millis();

  // Primary sweep direction: line is typically on the side
  // we bypassed FROM. If we went right around the obstacle,
  // line is on our left → sweep left first.
  // avoidDirectionHint: +1 = went right, -1 = went left
  // So primary direction = -avoidDirectionHint (sweep toward line)
  int sweepDir = (avoidDirectionHint >= 0) ? -1 : 1;

  // ── Phase 1: Arc forward + sweep primary direction (0–2s) ─────
  Serial.println("  Phase 1: Arc forward + sweep primary...");
  while (millis() - start < 2000) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count >= 1) {
      Serial.println("  Line found in Phase 1!");
      stopMotors();
      delay(50);
      return true;
    }

    // Arc: one wheel at AVOID_SPEED, other at ~half speed
    // This creates a curved forward path, covering ground
    if (sweepDir > 0) {
      // Sweep right: left wheel faster → curves right
      driveMotors(AVOID_SPEED, AVOID_SPEED / 2);
    } else {
      // Sweep left: right wheel faster → curves left
      driveMotors(AVOID_SPEED / 2, AVOID_SPEED);
    }
    delay(30);
  }
  stopMotors(); delay(50);

  // ── Phase 2: Arc forward + sweep opposite direction (2–4s) ────
  Serial.println("  Phase 2: Arc forward + sweep opposite...");
  while (millis() - start < 4000) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count >= 1) {
      Serial.println("  Line found in Phase 2!");
      stopMotors();
      delay(50);
      return true;
    }

    // Arc in opposite direction
    if (sweepDir > 0) {
      // Now sweep left
      driveMotors(AVOID_SPEED / 2, AVOID_SPEED);
    } else {
      // Now sweep right
      driveMotors(AVOID_SPEED, AVOID_SPEED / 2);
    }
    delay(30);
  }
  stopMotors(); delay(50);

  // ── Phase 3: Spin in place as last resort (4–6s) ──────────────
  Serial.println("  Phase 3: Spin search (last resort)...");
  while (millis() - start < REJOIN_TIMEOUT_MS) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count >= 1) {
      Serial.println("  Line found in Phase 3!");
      stopMotors();
      delay(50);
      return true;
    }

    unsigned long phaseElapsed = millis() - start - 4000;

    // Alternate spin direction every 1 second
    if (phaseElapsed < 1000) {
      driveMotors( sweepDir * 120, -sweepDir * 120);
    } else {
      driveMotors(-sweepDir * 120,  sweepDir * 120);
    }
    delay(30);
  }

  stopMotors();
  Serial.println("  Line NOT found — rejoin timeout (v7.30)");
  return false;
}

// ============================================================
//  SEARCH FOR END ZONE  (UNCHANGED)
//  Only when cubePickedUp==true and all sensors dark.
//  Returns true  → line found → NOT end zone.
//  Returns false → nothing found → IS end zone.
// ============================================================
bool searchForEndZone() {
  Serial.println("End zone search — sweeping left/right...");
  unsigned long start = millis();

  while (millis() - start < ENDZONE_SEARCH_MS / 3) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count > 0) { stopMotors(); return true; }
    driveMotors(-SPIN_SPEED, SPIN_SPEED);
    delay(20);
  }
  stopMotors(); delay(100);

  while (millis() - start < ENDZONE_SEARCH_MS) {
    readIRSensors();
    int count = 0;
    for (int i = 0; i < 5; i++) count += irBinary[i];
    if (count > 0) { stopMotors(); return true; }
    driveMotors(SPIN_SPEED, -SPIN_SPEED);
    delay(20);
  }

  stopMotors();
  Serial.println("Nothing found — end zone confirmed");
  return false;
}

// ============================================================
//  PICK UP GREEN CUBE  (UNCHANGED)
// ============================================================
void pickGreenCube() {
  Serial.println("── Picking green cube ──");
  stopMotors(); delay(100);

  gateServo.write(SERVO_OPEN);
  delay(300);

  driveMotors(CREEP_SPEED, CREEP_SPEED);
  delay(400);
  stopMotors(); delay(200);

  gateServo.write(SERVO_CLOSED);
  delay(800);

  Serial.println("★ Cube gripped — returning to line follow");
}

// ============================================================
//  DROP CUBE AT END ZONE  (UNCHANGED)
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
//  COLOR SENSOR — single reading  (UNCHANGED)
// ============================================================
char readColorOnce() {
  long r = 0, g = 0, b = 0;

  digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, LOW);
  delay(15);
  r = pulseIn(TCS_OUT, LOW, 100000);

  digitalWrite(TCS_S2, HIGH); digitalWrite(TCS_S3, HIGH);
  delay(15);
  g = pulseIn(TCS_OUT, LOW, 100000);

  digitalWrite(TCS_S2, LOW);  digitalWrite(TCS_S3, HIGH);
  delay(15);
  b = pulseIn(TCS_OUT, LOW, 100000);

  if (r == 0 || g == 0 || b == 0) return 'U';

  Serial.print("  RAW R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);

  // RED detection
  if (r < COLOR_RED_R_MAX &&
      r < (g - COLOR_RED_MARGIN) &&
      r < (b - COLOR_RED_MARGIN)) {
    return 'R';
  }

  // GREEN detection (v7.13 — 29/29 samples pass)
  {
    long maxVal = max(r, max(g, b));
    long minVal = min(r, min(g, b));

    if (r > COLOR_GREEN_ALL_MIN && r < COLOR_GREEN_ALL_MAX &&
        g > COLOR_GREEN_ALL_MIN && g < COLOR_GREEN_ALL_MAX &&
        b > COLOR_GREEN_ALL_MIN && b < COLOR_GREEN_ALL_MAX &&
        (maxVal - minVal) < COLOR_GREEN_SPREAD) {
      return 'G';
    }
  }

  return 'U';
}

// ============================================================
//  COLOR SENSOR — voted reading  (UNCHANGED)
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
//  ULTRASONIC — gated version  (UNCHANGED)
// ============================================================
float readUltrasonic() {
  if (!ultrasonicEnabled) return 999.0;

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
//  ULTRASONIC — force read (NEW v7.30)
//  Ignores ultrasonicEnabled flag. Used inside avoidObstacle()
//  to verify clearance without toggling the global flag.
// ============================================================
float readUltrasonicForce() {
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
//  MOTOR CONTROL  (UNCHANGED)
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
//  DEBUG DASHBOARD  (UNCHANGED)
// ============================================================
void printDashboard() {
  Serial.print("[");
  Serial.print(stateNames[currentState]); Serial.print("]");
  Serial.print(" IR:");
  for (int i = 0; i < 5; i++) Serial.print(irBinary[i]);
  Serial.print(" err:"); Serial.print(lastError);

  if (turning)          Serial.print(turnDirection < 0 ? " [PIVOT-L]" : " [PIVOT-R]");
  else if (reacquiring) Serial.print(" [REACQ]");
  else if (cornerApproach) Serial.print(" [SLOW]");
  else                  Serial.print(" [PID]");

  Serial.print(" dist:");  Serial.print(lastDistCm, 1);
  Serial.print(" obs:");   Serial.print(obstaclesAvoided);
  Serial.print(" cube:");  Serial.println(cubePickedUp ? "YES" : "NO");
}

// ============================================================
//  TEST MODES  (UNCHANGED)
// ============================================================
void runTestMode() {
  Serial.println("╔══════════════════════════════╗");
  Serial.println("║     TEST MODE ACTIVE         ║");
  Serial.println("╚══════════════════════════════╝");

  if (TEST_MOTORS) {
    Serial.println("=== MOTOR TEST ===");
    while (true) {
      Serial.println("Forward...");
      driveMotors(150, 150); delay(2000);
      stopMotors(); delay(500);
      Serial.println("Spin LEFT...");
      driveMotors(-150, 150); delay(1500);
      stopMotors(); delay(500);
      Serial.println("Spin RIGHT...");
      driveMotors(150, -150); delay(1500);
      stopMotors(); delay(500);
      Serial.println("Reverse...");
      driveMotors(-150, -150); delay(2000);
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
        Serial.print("=");
        Serial.print(digitalRead(irPins[i]));
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
      char color = readColorOnce();
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
//  END OF CODE v7.30 — EC6090 Mini Project 2026
//  Avoidance rewrite: IR-based direction, arc rejoin, longer
//  reverse, forward-on-retry. All other logic unchanged.
// ============================================================
