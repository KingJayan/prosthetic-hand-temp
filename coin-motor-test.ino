/*
  coin motor tester

  D3 = index
  D5 = middle
  D6 = palm
  wired with h-bridge
*/

const uint8_t VIB_PIN[3] = {3, 5, 6};

const uint8_t STEP = 5;
const uint8_t STEP_DELAY = 20;

void setup() {

  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(VIB_PIN[i], OUTPUT);
    analogWrite(VIB_PIN[i], 0);
  }

  Serial.println("Coin Motor Tester");
}

void sweepMotor(uint8_t pin) {

  for (int pwm = 0; pwm <= 255; pwm += STEP) {
    analogWrite(pin, pwm);
    delay(STEP_DELAY);
  }

  delay(500);

  for (int pwm = 255; pwm >= 0; pwm -= STEP) {
    analogWrite(pin, pwm);
    delay(STEP_DELAY);
  }

  delay(500);
}

void loop() {

  sweepMotor(VIB_PIN[0]);
  sweepMotor(VIB_PIN[1]);
  sweepMotor(VIB_PIN[2]);
}
