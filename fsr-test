/*
  fsr tester
  A4 = index
  A5 = middle
*/

const uint8_t FSR_PIN[2] = {A4, A5};
const unsigned long PRINT_INTERVAL = 100;

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("FSR Tester");
}

void loop() {

  int fsr1 = analogRead(FSR_PIN[0]);
  int fsr2 = analogRead(FSR_PIN[1]);

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    Serial.print("Index: ");
    Serial.print(fsr1);
    Serial.print("\tMiddle: ");
    Serial.println(fsr2);
  }
}
