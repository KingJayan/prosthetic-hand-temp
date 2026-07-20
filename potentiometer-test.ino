/*
  potentiometer tester
  p1-4 -> A0-A3

  each:
    L  -> GND
    R -> 5V
    M -> Analog pin
*/

const uint8_t NUM_POTS = 1;
const uint8_t POT_PIN[NUM_POTS] = {A0};



// exponential input shaping flag
#define ENABLE_INPUT_SHAPING 0

// smoothing alpha
const float ALPHA = 0.20f;

// const uint8_t SERVO_MIN[NUM_POTS] = {0, 10, 10, 10};
// const uint8_t SERVO_MAX[NUM_POTS] = {180, 160, 160, 160};
const uint8_t SERVO_MIN[NUM_POTS] = {0};
const uint8_t SERVO_MAX[NUM_POTS] = {180};

const unsigned long PRINT_INTERVAL = 100;



float filtered[NUM_POTS];
unsigned long lastPrint = 0;

int potToAngle(uint8_t i, int adc) {
  int angle = map(adc, 0, 1023, SERVO_MIN[i], SERVO_MAX[i]);
  return constrain(angle, SERVO_MIN[i], SERVO_MAX[i]);
}

void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < NUM_POTS; i++) {
    filtered[i] = analogRead(POT_PIN[i]);
  }

  Serial.println(F("----"));
  Serial.println(F("Potentiometer Tester"));
#if ENABLE_INPUT_SHAPING
  Serial.println(F("input shaping: enabled"));
#else
  Serial.println(F("input shaping: disabled"));
#endif
  Serial.println(F("----"));
}

void loop() {

  int raw[NUM_POTS];
  int value[NUM_POTS];
  int angle[NUM_POTS];

  for (uint8_t i = 0; i < NUM_POTS; i++) {

    raw[i] = analogRead(POT_PIN[i]);

#if ENABLE_INPUT_SHAPING
    filtered[i] = ALPHA * raw[i] + (1.0f - ALPHA) * filtered[i];
    value[i] = (int)filtered[i];
#else
    value[i] = raw[i];
#endif

    angle[i] = potToAngle(i, value[i]);
  }

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();

    Serial.println(F("-------------------------------------------------------------"));

    for (uint8_t i = 0; i < NUM_POTS; i++) {
      /*
      Serial.print(F("Pot "));
      Serial.print(i + 1);

      Serial.print(F(" | Raw: "));
      Serial.print(raw[i]);

      Serial.print(F(" | Used: "));
      Serial.print(value[i]);

      Serial.print(F(" | Angle: "));
      Serial.print(angle[i]);
      Serial.println(F("°"));
      */
      Serial.print(analogRead(A0));
      Serial.print('\t');
      Serial.print(analogRead(A1));
      Serial.print('\t');
      Serial.print(analogRead(A2));
      Serial.print('\t');
      Serial.println(analogRead(A3));
    }

    Serial.println();
  }
}
