/*
 * ============================================================
 *  IR SENSOR ARRAY TEST — ESP32 (5 sensors)
 * ============================================================
 *  Board  : ESP32-WROOM-32 30-pin DevKit V1
 *  Purpose: Test all 5 IR sensors individually
 *           Shows raw values, binary state, and visual bar
 *
 *  Pin map:
 *    S1 → D35 (GPIO35) Far  Left
 *    S2 → D34 (GPIO34) Mid  Left
 *    S3 → VN  (GPIO39) Center
 *    S4 → VP  (GPIO36) Mid  Right
 *    S5 → D33 (GPIO33) Far  Right
 *
 *  Wiring:
 *    IR VCC → Vin (5V)
 *    IR GND → GND
 *    IR OUT → pin above + voltage divider!
 *
 *  Open Serial Monitor at 115200 baud
 * ============================================================
 */

// ── PIN DEFINITIONS ──
#define IR_S1   35    // D35 — Far  Left  (input only)
#define IR_S2   34    // D34 — Mid  Left  (input only)
#define IR_S3   39    // VN  — Center     (input only)
#define IR_S4   36    // VP  — Mid  Right (input only)
#define IR_S5   33    // D33 — Far  Right

// ── THRESHOLD ──
// ESP32 ADC = 12-bit (0–4095)
// Start at 2000 — calibrate below!
// ON yellow line  = higher value
// OFF yellow line = lower value
#define IR_THRESHOLD  200

// Pin array for easy looping
const int IR_PINS[5] = {IR_S1, IR_S2, IR_S3, IR_S4, IR_S5};
const char* IR_NAMES[5] = {"S1-FarLeft", "S2-MidLeft", "S3-Center ", "S4-MidRight", "S5-FarRight"};

int irRaw[5];
int irBinary[5];

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Input only pins (35,34,39,36) don't need pinMode
  pinMode(IR_S5, INPUT);

  Serial.println("============================================");
  Serial.println("   IR SENSOR ARRAY TEST — ESP32 5 sensors  ");
  Serial.println("============================================");
  Serial.println("Wiring check:");
  Serial.println("  IR VCC → Vin (5V)");
  Serial.println("  IR GND → GND");
  Serial.println("  IR OUT → pin (via voltage divider)");
  Serial.println("--------------------------------------------");
  Serial.println("HOW TO CALIBRATE IR_THRESHOLD:");
  Serial.println("  1. Place robot on bare floor");
  Serial.println("     Note the RAW values (should be LOW)");
  Serial.println("  2. Place robot on yellow line");
  Serial.println("     Note the RAW values (should be HIGH)");
  Serial.println("  3. Set threshold halfway between them");
  Serial.println("     e.g. floor=800, line=3200 → use 2000");
  Serial.println("============================================");
  delay(2000);
}

void loop() {

  // ── READ ALL SENSORS ──
  for (int i = 0; i < 5; i++) {
    irRaw[i]    = analogRead(IR_PINS[i]);
    irBinary[i] = (irRaw[i] > IR_THRESHOLD) ? 1 : 0;
  }

  // ── PRINT RAW VALUES ──
  Serial.println("--------------------------------------------");
  Serial.println("RAW VALUES (0-4095):");
  for (int i = 0; i < 5; i++) {
    Serial.print("  ");
    Serial.print(IR_NAMES[i]);
    Serial.print(": ");
    Serial.print(irRaw[i]);

    // Show bar graph
    Serial.print("  [");
    int bars = map(irRaw[i], 0, 4095, 0, 20);
    for (int b = 0; b < 20; b++) {
      Serial.print(b < bars ? "=" : " ");
    }
    Serial.print("]");

    // Show state
    if (irBinary[i] == 1) {
      Serial.println("  ON LINE ██");
    } else {
      Serial.println("  off line");
    }
  }

  // ── VISUAL SENSOR MAP ──
  Serial.println();
  Serial.println("SENSOR MAP (front of robot):");
  Serial.println("  Far Left              Far Right");
  Serial.print("  [");
  for (int i = 0; i < 5; i++) {
    Serial.print(irBinary[i] == 1 ? " ██ " : " ░░ ");
  }
  Serial.println("]");
  Serial.print("   S1   S2   S3   S4   S5");
  Serial.println();

  // ── WEIGHTED ERROR ──
  int weights[5]  = {-2, -1, 0, 1, 2};
  int weightedSum = 0;
  int sensorSum   = 0;
  for (int i = 0; i < 5; i++) {
    weightedSum += irBinary[i] * weights[i];
    sensorSum   += irBinary[i];
  }

  Serial.println();
  Serial.print("WEIGHTED ERROR: ");
  if (sensorSum == 0) {
    Serial.println("NO LINE DETECTED!");
  } else {
    int error = weightedSum;
    Serial.print(error);
    if      (error == 0)  Serial.println("  → Perfectly centered ✓");
    else if (error == -1) Serial.println("  → Slightly LEFT");
    else if (error == 1)  Serial.println("  → Slightly RIGHT");
    else if (error == -2) Serial.println("  → Moderately LEFT");
    else if (error == 2)  Serial.println("  → Moderately RIGHT");
    else if (error <= -3) Serial.println("  → HARD LEFT (sharp turn!)");
    else if (error >= 3)  Serial.println("  → HARD RIGHT (sharp turn!)");
  }

  // ── DIAGNOSTIC WARNINGS ──
  Serial.println();

  // Check if all sensors read same value (wiring problem)
  bool allSame = true;
  for (int i = 1; i < 5; i++) {
    if (abs(irRaw[i] - irRaw[0]) > 200) { allSame = false; break; }
  }
  if (allSame) {
    Serial.println("WARNING: All sensors reading same value!");
    Serial.println("  → Check VCC and GND connections");
    Serial.println("  → Check voltage dividers on OUT pins");
  }

  // Check if all sensors read 0
  bool allZero = true;
  for (int i = 0; i < 5; i++) {
    if (irRaw[i] > 50) { allZero = false; break; }
  }
  if (allZero) {
    Serial.println("WARNING: All sensors reading 0!");
    Serial.println("  → IR sensors not powered (check Vin 5V)");
    Serial.println("  → Check OUT wire connections");
  }

  // Check if all sensors read max (4095)
  bool allMax = true;
  for (int i = 0; i < 5; i++) {
    if (irRaw[i] < 4000) { allMax = false; break; }
  }
  if (allMax) {
    Serial.println("WARNING: All sensors saturated (4095)!");
    Serial.println("  → Voltage divider missing or wrong values");
    Serial.println("  → 5V signal going directly to ESP32 GPIO!");
  }

  // Show threshold status
  Serial.println();
  Serial.print("THRESHOLD: "); Serial.println(IR_THRESHOLD);
  Serial.println("  Sensors ABOVE threshold = ON LINE (1)");
  Serial.println("  Sensors BELOW threshold = off line (0)");

  Serial.println("============================================");
  delay(300);   // update every 300ms — fast enough to see movement
}
