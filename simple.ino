#include <Servo.h>

Servo testServo;

int potPin = A2;
int servoPin = 9;

void setup() {
  Serial.begin(9600);
  testServo.attach(servoPin);
  Serial.println("init");
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
