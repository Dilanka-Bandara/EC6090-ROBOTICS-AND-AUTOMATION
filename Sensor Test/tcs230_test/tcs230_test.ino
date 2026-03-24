void setup() {
  Serial.begin(115200);
  delay(2000);  // longer delay to let Serial stabilize
  
  int candidates[] = {7, 8, 9, 10, 11, 20, 21, 24};
  int count = 8;
  
  Serial.println("=== PIN SCAN ===");
  
  for (int i = 0; i < count; i++) {
    pinMode(candidates[i], INPUT);
    delay(100);
    int val = digitalRead(candidates[i]);
    Serial.print("GPIO"); 
    Serial.print(candidates[i]);
    Serial.print(" = "); 
    Serial.println(val);
    delay(100);
  }
  Serial.println("=== DONE ===");
}

void loop() {}