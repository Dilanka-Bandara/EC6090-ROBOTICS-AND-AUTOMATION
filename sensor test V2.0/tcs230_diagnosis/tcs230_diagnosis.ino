/*
 * ============================================================
 *  TCS230 WIRING DIAGNOSIS CODE
 * ============================================================
 *  Use this to find exactly which wire/pin is faulty
 *  Upload and open Serial Monitor at 9600 baud
 *
 *  Pin map (v4):
 *    S0  → D9
 *    S1  → D12
 *    S2  → D13 (shared with OUT)
 *    S3  → A5
 *    OUT → D13 (shared with S2)
 *    OE  → GND (must be grounded!)
 * ============================================================
 */

#define TCS_S0_PIN   9
#define TCS_S1_PIN   12
#define TCS_D13_PIN  13   // S2 + OUT shared
#define TCS_S3_PIN   A5

void setup() {
  Serial.begin(9600);
  pinMode(TCS_S0_PIN,  OUTPUT);
  pinMode(TCS_S1_PIN,  OUTPUT);
  pinMode(TCS_S3_PIN,  OUTPUT);

  // Frequency scaling 20%
  digitalWrite(TCS_S0_PIN, HIGH);
  digitalWrite(TCS_S1_PIN, LOW);

  Serial.println("========================================");
  Serial.println("  TCS230 WIRING DIAGNOSIS              ");
  Serial.println("========================================");
  Serial.println("Hold RED or GREEN cube at 5cm");
  Serial.println("Watch each test result carefully");
  Serial.println("========================================\n");
  delay(2000);
}

void loop() {

  Serial.println("════ TEST 1: S3 switching check ════");
  Serial.println("Watching if S3 (A5) affects readings...");
  Serial.println("Both readings should be DIFFERENT if S3 works\n");

  // Reading with S3=LOW
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);   // S2=LOW
  digitalWrite(TCS_S3_PIN,  LOW);   // S3=LOW  → RED filter
  delay(20);
  pinMode(TCS_D13_PIN, INPUT);
  long reading_S3low = pulseIn(TCS_D13_PIN, LOW, 100000);

  // Reading with S3=HIGH
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);   // S2=LOW
  digitalWrite(TCS_S3_PIN,  HIGH);  // S3=HIGH → BLUE filter
  delay(20);
  pinMode(TCS_D13_PIN, INPUT);
  long reading_S3high = pulseIn(TCS_D13_PIN, LOW, 100000);

  Serial.print("  S3=LOW  reading: "); Serial.println(reading_S3low);
  Serial.print("  S3=HIGH reading: "); Serial.println(reading_S3high);

  long diff1 = abs(reading_S3low - reading_S3high);
  Serial.print("  Difference: "); Serial.println(diff1);

  if (diff1 < 10) {
    Serial.println("  ❌ FAIL: S3 wire NOT working!");
    Serial.println("  → Check A5 → TCS230 S3 connection");
  } else {
    Serial.println("  ✅ PASS: S3 wire OK");
  }

  Serial.println();
  delay(500);

  // ─────────────────────────────────────────
  Serial.println("════ TEST 2: S2 switching check ════");
  Serial.println("Watching if S2 (D13 output) affects readings...");
  Serial.println("Both readings should be DIFFERENT if S2 works\n");

  // Reading with S2=LOW, S3=LOW (RED filter)
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);   // S2=LOW
  digitalWrite(TCS_S3_PIN,  LOW);   // S3=LOW
  delay(20);
  pinMode(TCS_D13_PIN, INPUT);
  long reading_S2low = pulseIn(TCS_D13_PIN, LOW, 100000);

  // Reading with S2=HIGH, S3=HIGH (GREEN filter)
  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, HIGH);  // S2=HIGH
  digitalWrite(TCS_S3_PIN,  HIGH);  // S3=HIGH
  delay(20);
  pinMode(TCS_D13_PIN, INPUT);
  long reading_S2high = pulseIn(TCS_D13_PIN, LOW, 100000);

  Serial.print("  S2=LOW  S3=LOW  (RED filter):   "); Serial.println(reading_S2low);
  Serial.print("  S2=HIGH S3=HIGH (GREEN filter):  "); Serial.println(reading_S2high);

  long diff2 = abs(reading_S2low - reading_S2high);
  Serial.print("  Difference: "); Serial.println(diff2);

  if (diff2 < 10) {
    Serial.println("  ❌ FAIL: S2 switching NOT working!");
    Serial.println("  → D13 shared pin may have issue");
    Serial.println("  → Check D13 → TCS230 S2 AND OUT connection");
  } else {
    Serial.println("  ✅ PASS: S2 switching OK");
  }

  Serial.println();
  delay(500);

  // ─────────────────────────────────────────
  Serial.println("════ TEST 3: All 3 channels raw ════");
  Serial.println("R, G, B should show DIFFERENT values\n");

  long r, g, b;

  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);
  digitalWrite(TCS_S3_PIN,  LOW);
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  r = pulseIn(TCS_D13_PIN, LOW, 100000);

  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, HIGH);
  digitalWrite(TCS_S3_PIN,  HIGH);
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  g = pulseIn(TCS_D13_PIN, LOW, 100000);

  pinMode(TCS_D13_PIN, OUTPUT);
  digitalWrite(TCS_D13_PIN, LOW);
  digitalWrite(TCS_S3_PIN,  HIGH);
  delay(10);
  pinMode(TCS_D13_PIN, INPUT);
  b = pulseIn(TCS_D13_PIN, LOW, 100000);

  Serial.print("  R="); Serial.print(r);
  Serial.print("  G="); Serial.print(g);
  Serial.print("  B="); Serial.println(b);

  long spread = max(r, max(g, b)) - min(r, min(g, b));
  Serial.print("  Channel spread: "); Serial.println(spread);

  if (spread < 15) {
    Serial.println("  ❌ FAIL: All channels same!");
    Serial.println("  → Filter switching completely broken");
    Serial.println("  → Check S2, S3 wiring immediately");
  } else if (spread < 40) {
    Serial.println("  ⚠ WEAK: Small difference detected");
    Serial.println("  → Move cube closer (try 3-5cm)");
    Serial.println("  → Check S2/S3 connections");
  } else {
    Serial.println("  ✅ PASS: Channels showing difference");
  }

  Serial.println();
  delay(500);

  // ─────────────────────────────────────────
  Serial.println("════ TEST 4: OE pin check ════");
  Serial.println("OE pin MUST be connected to GND");
  Serial.println("If OE is floating → output disabled → all readings wrong");
  Serial.println("  → Verify: TCS230 OE pin wired to GND");
  Serial.println("  → If not wired: connect OE to GND now!");

  Serial.println();
  Serial.println("════════════════════════════════════");
  Serial.println("Repeating in 4 seconds...");
  Serial.println("════════════════════════════════════\n");
  delay(4000);
}
