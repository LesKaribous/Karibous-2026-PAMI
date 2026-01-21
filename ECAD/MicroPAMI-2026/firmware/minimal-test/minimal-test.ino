#include <tinyNeoPixel.h>
#include <Servo.h>
#include <Tiny4kOLED.h>

// -------------------- PINS --------------------
#define PIXEL_PIN        PIN_PC1
#define COLOR_SELECTOR   PIN_PC2
#define TIRETTE_PIN      PIN_PC3
#define SERVO_PIN        PIN_PC0

#define SENSOR_RIGHT     PIN_PA2
#define SENSOR_LEFT      PIN_PA3

// Moteur droit : PB2 PB3 PB4 PB5
#define R_IN1 PIN_PB2
#define R_IN2 PIN_PB3
#define R_IN3 PIN_PB4
#define R_IN4 PIN_PB5

// Moteur gauche : PA7 PA6 PA5 PA4
#define L_IN1 PIN_PA7
#define L_IN2 PIN_PA6
#define L_IN3 PIN_PA5
#define L_IN4 PIN_PA4

// -------------------- NEOPIXEL --------------------
#define PIXEL_COUNT 1

tinyNeoPixel pixel(
  PIXEL_COUNT,
  PIXEL_PIN,
  NEO_GRB + NEO_KHZ800
);

// -------------------- SERVO --------------------
Servo servo;

bool servoDir = false;
unsigned long lastServoUpdate = 0;
const unsigned long servoPeriod = 300; // ms

// -------------------- STEPPERS (28BYJ-48) --------------------
// Demi-pas (8 états) pour 28BYJ-48
const uint8_t halfStep[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

int idxR = 0;
int idxL = 0;

void setCoilsRight(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  digitalWrite(R_IN1, a);
  digitalWrite(R_IN2, b);
  digitalWrite(R_IN3, c);
  digitalWrite(R_IN4, d);
}

void setCoilsLeft(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  digitalWrite(L_IN1, a);
  digitalWrite(L_IN2, b);
  digitalWrite(L_IN3, c);
  digitalWrite(L_IN4, d);
}

void stepRight(int dir) {
  idxR = (idxR + dir) & 7;
  setCoilsRight(halfStep[idxR][0], halfStep[idxR][1], halfStep[idxR][2], halfStep[idxR][3]);
}

void stepLeft(int dir) {
  idxL = (idxL + dir) & 7;
  setCoilsLeft(halfStep[idxL][0], halfStep[idxL][1], halfStep[idxL][2], halfStep[idxL][3]);
}

void releaseMotors() {
  setCoilsRight(0,0,0,0);
  setCoilsLeft(0,0,0,0);
}

// -------------------- SEQUENCE MOTEURS --------------------
// États : pause -> avant -> pause -> arrière -> pause -> rotation -> pause -> release -> pause -> boucle
enum MotorState {
  MS_IDLE = 0,
  MS_FWD,
  MS_PAUSE1,
  MS_BWD,
  MS_PAUSE2,
  MS_TURN,
  MS_PAUSE3,
  MS_RELEASE,
  MS_PAUSE4
};

MotorState motorState = MS_IDLE;

const uint16_t STEPS_PER_PHASE = 2000;     // ajuste si tu veux (512 = test court)
const uint16_t PAUSE_MS        = 300;

const uint16_t stepIntervalUs  = 1000;    // vitesse: 2000-5000 us typique (plus petit = plus rapide)

uint16_t stepsRemaining = 0;
unsigned long lastStepUs = 0;
unsigned long stateStartMs = 0;

void startMotorSequence() {
  motorState = MS_FWD;
  stepsRemaining = STEPS_PER_PHASE;
  lastStepUs = micros();
}

void stopMotorSequence() {
  motorState = MS_IDLE;
  releaseMotors();
}

void motorTick() {
  // Si en pause, gérer les transitions par millis()
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  switch (motorState) {
    case MS_IDLE:
      // rien
      break;

    case MS_FWD:
      if ((unsigned long)(nowUs - lastStepUs) >= stepIntervalUs) {
        lastStepUs = nowUs;
        stepRight(+1);
        stepLeft(+1);
        if (--stepsRemaining == 0) {
          motorState = MS_PAUSE1;
          stateStartMs = nowMs;
        }
      }
      break;

    case MS_PAUSE1:
      if (nowMs - stateStartMs >= PAUSE_MS) {
        motorState = MS_BWD;
        stepsRemaining = STEPS_PER_PHASE;
      }
      break;

    case MS_BWD:
      if ((unsigned long)(nowUs - lastStepUs) >= stepIntervalUs) {
        lastStepUs = nowUs;
        stepRight(-1);
        stepLeft(-1);
        if (--stepsRemaining == 0) {
          motorState = MS_PAUSE2;
          stateStartMs = nowMs;
        }
      }
      break;

    case MS_PAUSE2:
      if (nowMs - stateStartMs >= PAUSE_MS) {
        motorState = MS_TURN;
        stepsRemaining = STEPS_PER_PHASE;
      }
      break;

    case MS_TURN:
      if ((unsigned long)(nowUs - lastStepUs) >= stepIntervalUs) {
        lastStepUs = nowUs;
        // rotation sur place : droite avant, gauche arrière
        stepRight(+1);
        stepLeft(-1);
        if (--stepsRemaining == 0) {
          motorState = MS_PAUSE3;
          stateStartMs = nowMs;
        }
      }
      break;

    case MS_PAUSE3:
      if (nowMs - stateStartMs >= PAUSE_MS) {
        motorState = MS_RELEASE;
      }
      break;

    case MS_RELEASE:
      releaseMotors();
      motorState = MS_PAUSE4;
      stateStartMs = nowMs;
      break;

    case MS_PAUSE4:
      if (nowMs - stateStartMs >= 1000) {
        // boucle tant que tirette LOW
        motorState = MS_FWD;
        stepsRemaining = STEPS_PER_PHASE;
      }
      break;
  }
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  pinMode(COLOR_SELECTOR, INPUT_PULLUP);
  pinMode(TIRETTE_PIN, INPUT_PULLUP);

  pinMode(SENSOR_RIGHT, INPUT_PULLUP);
  pinMode(SENSOR_LEFT, INPUT_PULLUP);

  pixel.begin();
  pixel.show();

    // Initialisation I2C (Wire interne, pins PB0/PB1 automatiques)
  oled.begin();

  // Configuration écran 96x16
  oled.clear();
  oled.on();
  oled.setFont(FONT6X8);
  oled.setRotation(2);

  // Position curseur (x, ligne)
  oled.setCursor(0, 0);
  oled.print("OLED 96x16");

  oled.setCursor(0, 1);
  oled.print("I2C OK");

  servo.attach(SERVO_PIN);
  servo.write(90);

  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  releaseMotors();
}

void loop() {
  // Lecture tirette
  bool tiretteLow = (digitalRead(TIRETTE_PIN) == LOW);

  // Démarrage/arrêt de la séquence moteurs selon tirette
  static bool motorRunning = false;
  if (tiretteLow && !motorRunning) {
    motorRunning = true;
    startMotorSequence();
  } else if (!tiretteLow && motorRunning) {
    motorRunning = false;
    stopMotorSequence();
  }

  // Tick moteurs (non bloquant)
  if (motorRunning) {
    motorTick();
  }

  // Lecture capteurs (LOW = detecte)
  bool rightLow = (digitalRead(SENSOR_RIGHT) == LOW);
  bool leftLow  = (digitalRead(SENSOR_LEFT)  == LOW);
  bool obstacle = rightLow || leftLow;

  // ----- NeoPixel (priorité capteurs) -----
  if (obstacle) {
    pixel.setPixelColor(0, pixel.Color(0, 255, 0));          // Vert
  } else if (tiretteLow) {
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));          // Rouge
  } else {
    if (digitalRead(COLOR_SELECTOR) == HIGH) {
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));        // Bleu
    } else {
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));      // Jaune
    }
  }
  pixel.show();

  // ----- Servo -----
  if (tiretteLow) {
    if (millis() - lastServoUpdate >= servoPeriod) {
      lastServoUpdate = millis();
      servoDir = !servoDir;
      servo.write(servoDir ? 45 : 90);
    }
  } else {
    servo.write(90);
  }
}
