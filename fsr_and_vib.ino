/*
  fsr + haptic test

  A4 -> index FSR

  D3 -> index motor
*/

const uint8_t FSR_PIN[1] = {A4};
const uint8_t VIB_PIN[1] = {3};

const int FSR_THRESHOLD = 120;
const int FSR_CEILING = 800;

const uint8_t VIB_MIN_PWM = 70;
const uint8_t VIB_MAX_PWM = 255;

const unsigned long PRINT_INTERVAL = 100;

unsigned long lastPrint = 0;

int fsrRaw;
uint8_t vibDuty;

uint8_t forceToDuty(int adc) {

  if (adc < FSR_THRESHOLD)
    return 0;

  int duty = map(
    adc,
    FSR_THRESHOLD,
    FSR_CEILING,
    VIB_MIN_PWM,
    VIB_MAX_PWM
  );

  return constrain(duty, VIB_MIN_PWM, VIB_MAX_PWM);
}

void setup() {

  Serial.begin(9600);

  pinMode(VIB_PIN, OUTPUT);
  analogWrite(VIB_PIN[0], 0);

  Serial.println("touch + haptics test");
}

void loop() {

  fsrRaw = analogRead(FSR_PIN[0]);
  vibDuty = forceToDuty(fsrRaw);

  analogWrite(VIB_PIN, vibDuty);

  if (millis() - lastPrint >= PRINT_INTERVAL) {

    lastPrint = millis();

    Serial.print("FSR: ");
    Serial.println(fsrRaw);
    Serial.print("  PWM: ");
    Serial.println(vibDuty);
  }
}
