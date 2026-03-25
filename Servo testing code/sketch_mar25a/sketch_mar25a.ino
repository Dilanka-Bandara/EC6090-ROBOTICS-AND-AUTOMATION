#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 27;

void setup() {
  Serial.begin(115200);
  
  // Attach servo to pin 27
  myServo.attach(SERVO_PIN, 500, 2400); // min/max pulse width in microseconds
  
  Serial.println("Servo Test Started");
  Serial.println("Commands: '0' = 0deg, '9' = 90deg, '1' = 180deg, 's' = sweep");
}

void loop() {
  
  // Sweep from 0 to 180
  Serial.println("Moving to 0 degrees");
  myServo.write(0);
  delay(1000);

  Serial.println("Moving to 90 degrees");
  myServo.write(90);
  delay(1000);

  Serial.println("Moving to 180 degrees");
  myServo.write(180);
  delay(1000);

  Serial.println("Moving back to 90 degrees");
  myServo.write(90);
  delay(1000);
}