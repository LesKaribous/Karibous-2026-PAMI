#include <Arduino.h>

#include <tinyNeoPixel.h>
#include <Servo.h>
#include <Tiny4kOLED.h>
#include <AccelStepper.h>

#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>

// -------------------- PINS --------------------
#define PIXEL_PIN        PIN_PC1
#define COLOR_SELECTOR   PIN_PC2
#define TIRETTE_PIN      PIN_PC3
#define SERVO_PIN        PIN_PC0

#define SENSOR_RIGHT     PIN_PA2
#define SENSOR_LEFT      PIN_PA3

bool obstacle = false;

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

// -------------------- OLED --------------------

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// create a RoboEyes instance using an Adafruit_SSD1306 display driver
RoboEyes<Adafruit_SSD1306> roboEyes(display); 

// -------------------- SERVO --------------------
Servo servo;

bool servoDir = false;
unsigned long lastServoUpdate = 0;
const unsigned long servoPeriod = 300; // ms

// -------------------- STEPPERS (28BYJ-48) --------------------
AccelStepper motorRight(AccelStepper::FULL4WIRE, R_IN1, R_IN3, R_IN2, R_IN4);
AccelStepper motorLeft(AccelStepper::FULL4WIRE, L_IN1, L_IN3, L_IN2, L_IN4);

float motorMaxSpeed = 700.0;
float motorAccel = 400.0;

// -------------------- MATCH TIMER --------------------
unsigned long startTime = 0;
const unsigned long matchDuration = 5000UL; // 5 seconds

// -------------------- FUNCTIONS --------------------

void updateTeamColor();
bool getSensor();
void updateMatchTimer();
void endMatch();

void updateTeamColor() {
  if (digitalRead(COLOR_SELECTOR) == HIGH) {
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));        // Bleu
    } else {
      pixel.setPixelColor(0, pixel.Color(255, 255, 0));      // Jaune
    }
  pixel.show();
}

bool getSensor() {
  // Lecture capteurs (LOW = detecte)
  bool rightLow = (digitalRead(SENSOR_RIGHT) == LOW);
  bool leftLow  = (digitalRead(SENSOR_LEFT)  == LOW);
  bool previousObstacle = obstacle;
  obstacle = rightLow || leftLow;


  if (obstacle && !previousObstacle) {
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));          // RED
    pixel.show();
    roboEyes.setMood(ANGRY);
  }
  else if (!obstacle && previousObstacle) {
    pixel.setPixelColor(0, pixel.Color(0, 255, 0));          // VERT
    pixel.show();
    roboEyes.setMood(HAPPY);
  }
  return obstacle;
}

void updateMatchTimer(){
  if (millis()-startTime >= matchDuration){
    endMatch();
  } 
}

void endMatch() {
  motorRight.stop();
  motorLeft.stop();

  // NeoPixel
  pixel.setPixelColor(0, pixel.Color(255, 165, 0));          // ORANGE
  pixel.show();

  // RoboEyes -- TBD
  roboEyes.setMood(DEFAULT);

  // Servo alternate
  while(1) 
  {
    roboEyes.update(); // Update the eyes animations
    // Servo alternate
    if (millis() - lastServoUpdate >= servoPeriod) {
        lastServoUpdate = millis();
        servoDir = !servoDir;
        servo.write(servoDir ? 45 : 90);
      }
  }
}

// -------------------- SETUP / LOOP --------------------

void setup() {
  pinMode(COLOR_SELECTOR, INPUT_PULLUP);
  pinMode(TIRETTE_PIN, INPUT_PULLUP);

  pinMode(SENSOR_RIGHT, INPUT_PULLUP);
  pinMode(SENSOR_LEFT, INPUT_PULLUP);

  pixel.begin();
  pixel.setBrightness(20);
  pixel.show();

  // OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C or 0x3D
    for(;;); // Don't proceed, loop forever
  }
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100); // screen-width, screen-height, max framerate

  // Define some automated eyes behaviour
  roboEyes.setAutoblinker(ON, 3, 2); // autoblinker ON, every 3 seconds, blink duration 2 frames
  roboEyes.setIdleMode(ON, 2, 2); // idle-mode with look-around every 2 seconds

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(90);

  // Moteurs pas à pas

  motorRight.setMaxSpeed(motorMaxSpeed);
  motorRight.setAcceleration(motorAccel);

  motorLeft.setMaxSpeed(motorMaxSpeed);
  motorLeft.setAcceleration(motorAccel);

  // attendre que la tirette soit enlevée avant de démarrer
  roboEyes.close();
  while (digitalRead(TIRETTE_PIN) == HIGH) 
  {
    updateTeamColor();
    //roboEyes.update(); // Update the eyes animations
  }
  roboEyes.open(); // open eyes 
  roboEyes.setMood(DEFAULT);
  delay(500);
  while (digitalRead(TIRETTE_PIN) == LOW) 
  {
    updateTeamColor();
    roboEyes.update(); // Update the eyes animations
  }

  startTime = millis();

  pixel.setPixelColor(0, pixel.Color(0, 255, 0));          // VERT
  pixel.show();
  roboEyes.setMood(HAPPY);
  roboEyes.anim_laugh();
}

void loop() {
  
  updateMatchTimer();
  getSensor();
  roboEyes.update();

  motorRight.run();
  motorLeft.run();

}
