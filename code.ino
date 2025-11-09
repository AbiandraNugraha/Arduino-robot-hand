/*Servo (A0) + Stepper speed/direction (A1) with deadzone + coil-off idle
 Pins: IN1..IN4 -> D2..D5, Servo -> D6, VRx -> A0 (servo), Vry -> A1 (stepper)*/

#include <Servo.h>

const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;
const int SERVO_PIN = 6;
const int POT_SERVO = A0; // X -> servo
const int POT_STEPPER = A1; // Y -> stepper (speed/direction)

Servo myServo;

// 8 half-step sequence for 28BYJ-48
const uint8_t seq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

int stepIndex = 0;
unsigned long lastStepMicros = 0;

// Stepper speed mapping
const int DEADZONE = 40; // analog center dead zone ±40 (tweak if needed)
const int ADC_CENTER = 512;

// microsecond delays (bigger = slower)
const unsigned long STEP_DELAY_SLOW = 1200UL; // slowest (in microseconds)
const unsigned long STEP_DELAY_FAST = 160UL;  // fastest (in microseconds)

// Servo pulse range
const int SERVO_US_MIN = 700;
const int SERVO_US_MAX = 2300;
int currentUS = 1500;

unsigned long lastCoilOffMillis = 0;
const unsigned long COIL_OFF_TIMEOUT = 150; // ms: after this idle, turn coils off

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // start with coils off
  coilsOff();

  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(currentUS);

  delay(200);
  Serial.println("Ready: servo -> A0, stepper speed/direction -> A1");
  Serial.println("Center pot to idle; move pot away for direction+speed.");
}

void loop() {
  // --- Servo: exact mapping from A0 (X)
  int rawServo = analogRead(POT_SERVO); // 0..1023
  int targetUS = map(rawServo, 0, 1023, SERVO_US_MIN, SERVO_US_MAX);

  // Apply the servo pulse (smooth)
  if (abs(targetUS - currentUS) > 5) {
    if (targetUS > currentUS) currentUS += min(12, targetUS - currentUS);
    else currentUS -= min(12, currentUS - targetUS);
    myServo.writeMicroseconds(currentUS);
  } else {
    // refresh pulse occasionally
    myServo.writeMicroseconds(currentUS);
  }

  // --- Stepper: speed/direction from A1 (Y)
  int rawStep = analogRead(POT_STEPPER); // 0..1023
  int delta = rawStep - ADC_CENTER;

  // debug print every 120 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 120) {
    Serial.print("A0="); Serial.print(rawServo);
    Serial.print(" us="); Serial.print(targetUS);
    Serial.print("  | A1="); Serial.print(rawStep);
    Serial.print(" delta="); Serial.print(delta);
    Serial.print(" stepIdx="); Serial.println(stepIndex);
    lastPrint = millis();
  }

  if (abs(delta) <= DEADZONE) {
    // idle: stop stepping and after timeout, turn coils off
    if (millis() - lastCoilOffMillis > COIL_OFF_TIMEOUT) {
      coilsOff();
    } else {
      // keep coils on briefly to avoid chattering
      // do nothing
    }
  } else {
    // we need to move: ensure coils on
    coilsOnForIndex(stepIndex);

    // compute speed: magnitude of delta -> delay mapping
    int mag = abs(delta) - DEADZONE; // 0 .. (512 - DEADZONE)
    if (mag < 0) mag = 0;
    // map mag (0..(512-DEADZONE)) to delay (STEP_DELAY_SLOW .. STEP_DELAY_FAST)
    unsigned long maxMag = ADC_CENTER - DEADZONE;
    unsigned long stepDelay = STEP_DELAY_SLOW;
    if (maxMag > 0) {
      // inverse mapping: larger mag -> smaller delay (faster)
      stepDelay = map(mag, 0, maxMag, STEP_DELAY_SLOW, STEP_DELAY_FAST);
    }

    // direction: delta > 0 => CW (forward); delta < 0 => CCW (back)
    int direction = (delta > 0) ? -1 : 1;

    // step if time passed
    unsigned long now = micros();
    if ((long)(now - lastStepMicros) >= (long)stepDelay) {
      lastStepMicros = now;
      if (direction > 0) {
        stepIndex++;
        if (stepIndex >= 8) stepIndex = 0;
      } else {
        stepIndex--;
        if (stepIndex < 0) stepIndex = 7;
      }
      // output coil states
      digitalWrite(IN1, seq[stepIndex][0]);
      digitalWrite(IN2, seq[stepIndex][1]);
      digitalWrite(IN3, seq[stepIndex][2]);
      digitalWrite(IN4, seq[stepIndex][3]);
      // mark last active time for coil-off timeout
      lastCoilOffMillis = millis();
    }
  }

  delay(6); // small cooperative delay
}

// Turn all ULN2003 outputs low (coils off)
void coilsOff() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Re-apply coil outputs for a given stepIndex (used when waking from idle)
void coilsOnForIndex(int idx) {
  digitalWrite(IN1, seq[idx][0]);
  digitalWrite(IN2, seq[idx][1]);
  digitalWrite(IN3, seq[idx][2]);
  digitalWrite(IN4, seq[idx][3]);
}
