/*
 * ============================================================
 *  TCS230 COLOR SENSOR — TEST CODE
 * ============================================================
 *  Purpose : Test if TCS230 is working correctly
 *            Shows raw R,G,B frequency values on Serial Monitor
 *            and detected color name
 *
 *  Pin map (matches robot v4):
 *    TCS230 VCC  → 5V
 *    TCS230 GND  → GND
 *    TCS230 S0   → D9
 *    TCS230 S1   → D12
 *    TCS230 S2   → D13  (shared with OUT — switched in code)
 *    TCS230 S3   → A5
 *    TCS230 OUT  → D13  (shared with S2 — switched in code)
 *
 *  How to use:
 *    1. Upload this code
 *    2. Open Serial Monitor → set baud to 9600
 *    3. Hold each colored object in front of sensor (~5-10cm)
 *    4. Read the R, G, B values printed
 *    5. Note the values for RED cube and GREEN cube separately
 *    6. Update thresholds in robot v4 code accordingly
 *
 *  Author  : EC6090 Mini Project 2026
 * ============================================================
 */

// ── PIN DEFINITIONS (v4 mapping) ──
#define TCS_S0_PIN   9
#define TCS_S1_PIN   12
#define TCS_D13_PIN  13   // shared S2 output + OUT input
#define TCS_S3_PIN   A5

// ── TEST DISTANCE ──
// Hold object at this approximate distance for consistent readings
// Recommended: 5–10 cm from sensor face

void setup() {
  Serial.begin(9600);

  pinMode(TCS_S0_PIN,  OUTPUT);
  pinMode(TCS_S1_PIN,  OUTPUT);
  pinMode(TCS_S3_PIN,  OUTPUT);
  pinMode(TCS_D13_PIN, OUTPUT);

  // Frequency scaling: S0=HIGH S1=LOW = 20%
  // If readings are always 0 → try S0=HIGH S1=HIGH (100%)
  // If readings are too fast → try S0=LOW  S1=HIGH (2%)
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  Serial.println("============================================");
  Serial.println("  TCS230 Color Sensor Test — v4 Pin Map   ");
  Serial.println("============================================");
  Serial.println("Hold object 5-10cm in front of sensor");
  Serial.println("Reading every 1 second...");
  Serial.println("--------------------------------------------");
  Serial.println("FORMAT:  R=xxx  G=xxx  B=xxx  → COLOR");
  Serial.println("Lower value = stronger that color");
  Serial.println("============================================");
  delay(1000);
}

void loop() {
  long r, g, b;

  // ── READ RED channel ──
  // S2=LOW, S3=LOW
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);    // S2 = LOW
  digitalWrite(TCS_S3_PIN,  LOW);    // S3 = LOW
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  r = pulseIn(TCS_D13_PIN, LOW, 100000);  // longer timeout for test

  // ── READ GREEN channel ──
  // S2=HIGH, S3=HIGH
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, HIGH);   // S2 = HIGH
  digitalWrite(TCS_S3_PIN,  HIGH);   // S3 = HIGH
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  g = pulseIn(TCS_D13_PIN, LOW, 100000);

  // ── READ BLUE channel ──
  // S2=LOW, S3=HIGH
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);    // S2 = LOW
  digitalWrite(TCS_S3_PIN,  HIGH);   // S3 = HIGH
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  b = pulseIn(TCS_D13_PIN, LOW, 100000);

  // ── PRINT RAW VALUES ──
  Serial.print("R=");
  Serial.print(r);
  Serial.print("\t G=");
  Serial.print(g);
  Serial.print("\t B=");
  Serial.print(b);
  Serial.print("\t → ");

  // ── DETECT AND PRINT COLOR ──
  if (r == 0 && g == 0 && b == 0) {
    Serial.println("NO SIGNAL (check wiring!)");
  }
  else if (r > 5000 && g > 5000 && b > 5000) {
    Serial.println("TOO FAR / NO OBJECT");
  }
  else {
    // Find minimum — lowest value = dominant color
    long minVal = min(r, min(g, b));

    if (r == minVal) {
      Serial.print("RED");
      // Show confidence
      long diff = min(g, b) - r;
      if (diff < 30)       Serial.println("  (weak — borderline)");
      else if (diff < 100) Serial.println("  (moderate)");
      else                 Serial.println("  (strong)");
    }
    else if (g == minVal) {
      Serial.print("GREEN");
      long diff = min(r, b) - g;
      if (diff < 30)       Serial.println("  (weak — borderline)");
      else if (diff < 100) Serial.println("  (moderate)");
      else                 Serial.println("  (strong)");
    }
    else {
      Serial.print("BLUE");
      long diff = min(r, g) - b;
      if (diff < 30)       Serial.println("  (weak — borderline)");
      else if (diff < 100) Serial.println("  (moderate)");
      else                 Serial.println("  (strong)");
    }
  }

  // ── DIAGNOSTIC WARNINGS ──
  if (r == 0 || g == 0 || b == 0) {
    Serial.println("  ⚠ WARNING: One channel reads 0!");
    Serial.println("  Check: D13 wiring, S2/OUT connection");
  }
  if (r == g && g == b) {
    Serial.println("  ⚠ WARNING: All channels equal!");
    Serial.println("  Check: S3 on A5, sensor facing object");
  }

  Serial.println("--------------------------------------------");
  delay(1000);  // read every 1 second
}
