/*
  fsr tester
  A4 = index
*/

const uint8_t FSR_PIN = A4;
const unsigned long PRINT_INTERVAL = 100;

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("FSR Tester");
}

void loop() {

  int fsr1 = analogRead(FSR_PIN);

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    Serial.print("Index: ");
    Serial.println(fsr1);
  }
}
