#include <Arduino.h>

#include <tinyNeoPixel.h>
#include <Servo.h>
#include <Tiny4kOLED.h>
#include <AccelStepper.h>

// -------------------- PINS --------------------
#define PIXEL_PIN        PIN_PC1
#define COLOR_SELECTOR   PIN_PC2
#define TIRETTE_PIN      PIN_PC3
#define SERVO_PIN        PIN_PC0

#define SENSOR_RIGHT     PIN_PA2
#define SENSOR_LEFT      PIN_PA3

// Moteur gauche : PB2 PB3 PB4 PB5
#define L_IN1 PIN_PB2
#define L_IN2 PIN_PB3
#define L_IN3 PIN_PB4
#define L_IN4 PIN_PB5

// Moteur droit : PA7 PA6 PA5 PA4
#define R_IN1 PIN_PA7
#define R_IN2 PIN_PA6
#define R_IN3 PIN_PA5
#define R_IN4 PIN_PA4

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
AccelStepper motorRight(AccelStepper::FULL4WIRE, R_IN1, R_IN3, R_IN2, R_IN4);
AccelStepper motorLeft(AccelStepper::FULL4WIRE, L_IN1, L_IN3, L_IN2, L_IN4);

// -------------------- SEQUENCE MOTEURS --------------------



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

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(90);

  // Moteurs pas à pas

  motorRight.setMaxSpeed(700.0);     // pas/s
  motorRight.setAcceleration(400.0);// pas/s^2

  motorLeft.setMaxSpeed(700.0);
  motorLeft.setAcceleration(400.0);

  motorRight.moveTo(-10048);
  motorLeft.moveTo(10048);
}

void loop() {
  // Lecture tirette
  bool tiretteLow = (digitalRead(TIRETTE_PIN) == LOW);

  // Démarrage/arrêt de la séquence moteurs selon tirette
  static bool motorRunning = false;
  if (tiretteLow && !motorRunning) {
    motorRunning = true;
    if (motorRight.distanceToGo() == 0) motorRight.moveTo(-motorRight.currentPosition());
    if (motorLeft.distanceToGo() == 0)  motorLeft.moveTo(-motorLeft.currentPosition());

  } else if (!tiretteLow && motorRunning) {
    motorRunning = false;
    motorRight.stop();
    motorLeft.stop();
  }

  // Tick moteurs (non bloquant)
  if (motorRunning) {
    motorRight.run();
    motorLeft.run();
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
  /*
  if (tiretteLow) {
    if (millis() - lastServoUpdate >= servoPeriod) {
      lastServoUpdate = millis();
      servoDir = !servoDir;
      servo.write(servoDir ? 45 : 90);
    }
  } else {
    servo.write(90);
  }
    */
}
