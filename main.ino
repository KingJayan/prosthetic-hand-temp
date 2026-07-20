/*
 prosthetic arm
 4x servo, 4x pot, 2x FSR, 3x vib motor via L293D
*/

#include <Servo.h>

const uint8_t NUM_JOINTS = 2;
const uint8_t POT_PIN[NUM_JOINTS] = { A2, A3 };
const uint8_t SERVO_PIN[NUM_JOINTS] = { 9, 10 };
const uint8_t FSR_PIN[2] = { A4, A5 };
const uint8_t VIB_PIN[3] = { 3, 5 };

const int SERVO_MIN[NUM_JOINTS] = { 5, 5 };
const int SERVO_MAX[NUM_JOINTS] = { 175, 175 };

const int FSR_THRESHOLD = 120;
const int FSR_CEILING = 800;
const uint8_t VIB_MIN_PWM = 70;
const uint8_t VIB_MAX_PWM = 255;

const float ALPHA = 0.2f;
const uint8_t DEADBAND = 3;

// glitch rejection
const int ADC_FLOOR = 8;
const int ADC_ROOF = 1015;
const int JUMP_THRESHOLD = 120;
const uint8_t CONFIRM_N = 4;
const int CONFIRM_TOL = 40;
const uint16_t FAULT_TICKS = 20;

const unsigned long PRINT_INTERVAL = 200;

Servo joint[NUM_JOINTS];
float filteredPot[NUM_JOINTS];
int lastAngle[NUM_JOINTS];
int lastGoodRaw[NUM_JOINTS];
int candidate[NUM_JOINTS];
uint8_t candCount[NUM_JOINTS];
uint16_t rejects[NUM_JOINTS];
bool chanFaulted[NUM_JOINTS];

int fsrRaw[2];
uint8_t vibDuty[3];
unsigned long lastPrint = 0;
unsigned long lastServoUpdate = 0;

int potToAngle(uint8_t i, int adc) {
  int a = map(adc, 0, 1023, SERVO_MIN[i], SERVO_MAX[i]);
  return constrain(a, SERVO_MIN[i], SERVO_MAX[i]);
}

uint8_t forceToDuty(int adc) {
  if (adc < FSR_THRESHOLD) return 0;
  int d = map(adc, FSR_THRESHOLD, FSR_CEILING, VIB_MIN_PWM, VIB_MAX_PWM);
  return (uint8_t)constrain(d, VIB_MIN_PWM, VIB_MAX_PWM);
}

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

int readFilteredADC(uint8_t pin) {
  analogRead(pin);
  int m[3];
  for (uint8_t k = 0; k < 3; k++) {
    long sum = 0;
    for (uint8_t i = 0; i < 4; i++) sum += analogRead(pin);
    m[k] = sum >> 2;
  }
  return median3(m[0], m[1], m[2]);
}

int readFSR(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    joint[i].attach(SERVO_PIN[i]);
    filteredPot[i] = readFilteredADC(POT_PIN[i]);
    lastGoodRaw[i] = (int)filteredPot[i];
    candidate[i] = lastGoodRaw[i];
    candCount[i] = 0;
    rejects[i] = 0;
    chanFaulted[i] = false;
    lastAngle[i] = potToAngle(i, lastGoodRaw[i]);
    joint[i].write(lastAngle[i]);
  }

  for (uint8_t m = 0; m < 3; m++) {
    pinMode(VIB_PIN[m], OUTPUT);
    analogWrite(VIB_PIN[m], 0);
    vibDuty[m] = 0;
  }

  Serial.println(F("online"));
  Serial.println(F("A0 A1 A2 A3 | rejects"));
  delay(300);
}

void updateJoint(uint8_t i) {
  int raw = readFilteredADC(POT_PIN[i]);

    if (raw < ADC_FLOOR) {
      raw = ADC_FLOOR;
    } else if (raw > ADC_ROOF) {
      raw = ADC_ROOF;
    }
  
  if (abs(raw - lastGoodRaw[i]) > JUMP_THRESHOLD) {
    rejects[i]++;
    if (abs(raw - candidate[i]) <= CONFIRM_TOL) {
      candCount[i]++;
    } else {
      candidate[i] = raw;
      candCount[i] = 1;
    }
    if (candCount[i] >= CONFIRM_N) {
      lastGoodRaw[i] = raw;   // sustained and stable, accept
      candCount[i] = 0;
    }
  } else {
    lastGoodRaw[i] = raw;
    candCount[i] = 0;
  }

  if (candCount[i] >= FAULT_TICKS && !chanFaulted[i]) {
    chanFaulted[i] = true;
    joint[i].detach();
    Serial.print(F("FAULT ch"));
    Serial.println(i);
  }
  if (chanFaulted[i]) return;

  filteredPot[i] = (ALPHA * lastGoodRaw[i]) + ((1.0f - ALPHA) * filteredPot[i]);

  int angle = potToAngle(i, (int)filteredPot[i]);
  if (abs(angle - lastAngle[i]) >= DEADBAND) {
    angle = constrain(angle, 0, 180);
    joint[i].write(angle);
    lastAngle[i] = angle;
  }
}

void loop() {
  if (millis() - lastServoUpdate >= 20) {
    lastServoUpdate = millis();
    for (uint8_t i = 0; i < NUM_JOINTS; i++) updateJoint(i);
  }

  fsrRaw[0] = readFSR(FSR_PIN[0]);
  fsrRaw[1] = readFSR(FSR_PIN[1]);

  vibDuty[0] = forceToDuty(fsrRaw[0]);
  vibDuty[1] = forceToDuty(fsrRaw[1]);
  vibDuty[2] = forceToDuty(max(fsrRaw[0], fsrRaw[1]));

  /*
  for (uint8_t m = 0; m < 3; m++) analogWrite(VIB_PIN[m], vibDuty[m]);
  */

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      Serial.print(lastGoodRaw[i]);
      Serial.print('\t');
    }
    Serial.print(F("| "));
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      Serial.print(rejects[i]);
      Serial.print(i == NUM_JOINTS - 1 ? '\n' : '\t');
    }
  }
}
