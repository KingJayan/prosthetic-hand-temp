#include <Servo.h>

Servo testServo;

int potPin = A0;
int servoPin = 2;

void setup() {
  Serial.begin(9600);
  testServo.attach(servoPin);
  Serial.println("simple test running...");
}

void loop() {
  int rawValue = analogRead(potPin);
  int angle = map(rawValue, 0, 1023, 0, 180);
  
  testServo.write(angle);

  Serial.print("Raw: ");
  Serial.print(rawValue);
  Serial.print("\t Angle: ");
  Serial.println(angle);

  delay(15);
}
