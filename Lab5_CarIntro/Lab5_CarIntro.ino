#include "protothreads.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4  //left motor
#define MotorPWM_B 5  //right motor

pt ptBlink;

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

#define FRONT_LEFT_TURN 49
#define FRONT_LEFT_HIGH 47
#define FRONT_LEFT_LOW 45
#define FRONT_RIGHT_TURN 43
#define FRONT_RIGHT_HIGH 41
#define FRONT_RIGHT_LOW 39
#define REAR_RIGHT_TURN 37
#define REAR_RIGHT_REVERSE 35
#define REAR_RIGHT_BRAKE 33
#define REAR_LEFT_TURN 31
#define REAR_LEFT_REVERSE 29
#define REAR_LEFT_BREAK 27

#define LEFT_ENCODER_FRONT 2
#define LEFT_ENCODER_REAR 3

#define RIGHT_ENCODER_FRONT 18
#define RIGHT_ENCODER_REAR 19

#define PIXY_CAMERA 10
#define SONAR 11

#define BUTTON_ONE 10
#define BUTTON_TWO 12

const uint8_t countPerRotation = 180;
const uint8_t wheelCircumference = 223.84;  //mm
const uint32_t line_length = 600.14;          //TODO: measure the actual length


static volatile int16_t count_left = 0;
static volatile int16_t count_right = 0;


// 3.215 = (60sec/0.1sec)/(48gear ratio * 4pulses/rev)
float rotation = 3.125;
float RPM = 0;

uint8_t base_speed = 155;

uint8_t base_right_speed = 180;
uint8_t base_left_speed = 175;

uint8_t right_speed = 180;
uint8_t left_speed = 175;

#define BAUD_RATE 9600

void nonblockingDelay(unsigned long ms) {
  unsigned long startTime = millis();
  while (millis() - startTime < ms) {
    yield();
  }
}

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  PT_INIT(&ptBlink);
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(FRONT_LEFT_TURN, OUTPUT);
  pinMode(FRONT_LEFT_HIGH, OUTPUT);
  pinMode(FRONT_LEFT_LOW, OUTPUT);
  pinMode(FRONT_RIGHT_TURN, OUTPUT);
  pinMode(FRONT_RIGHT_HIGH, OUTPUT);
  pinMode(FRONT_RIGHT_LOW, OUTPUT);
  pinMode(REAR_RIGHT_TURN, OUTPUT);
  pinMode(REAR_RIGHT_REVERSE, OUTPUT);
  pinMode(REAR_RIGHT_BRAKE, OUTPUT);
  pinMode(REAR_LEFT_TURN, OUTPUT);
  pinMode(REAR_LEFT_REVERSE, OUTPUT);
  pinMode(REAR_LEFT_BREAK, OUTPUT);

  pinMode(LEFT_ENCODER_FRONT, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_FRONT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_FRONT), ISRMotorLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_FRONT), ISRMotorRight, FALLING);
  Serial.begin(BAUD_RATE);

  for (int i = 0; i < 4; i++) {
    Forward3ft();
    leftTurn();
  }
}

/***************************************/
// This is the Interrupt Service Routine
// for the left motor.
/***************************************/
void ISRMotorLeft() {
  count_left++;
}

void ISRMotorRight() {
  count_right++;
}

// Method: Forward
// Input: speed – value [0-255]
// Rotate the motor in a clockwise fashion
void Forward() {
  analogWrite(MotorPWM_A, left_speed);
  analogWrite(MotorPWM_B, right_speed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

static PT_THREAD(rightTurn(struct pt *pt)) {
  static int i = 0;

  PT_BEGIN(pt);

  for (i = 0; i < 10; i++) {
    digitalWrite(FRONT_RIGHT_TURN, HIGH);
    digitalWrite(REAR_RIGHT_TURN, HIGH);
    PT_SLEEP(pt, 100);
    digitalWrite(FRONT_RIGHT_TURN, LOW);
    digitalWrite(REAR_RIGHT_TURN, LOW);
    PT_SLEEP(pt, 100);
  }

  PT_END(pt);
}

unsigned long prevTime = 0;

static PT_THREAD(leftTurnSignal(struct pt *pt)) {
  int i = 0;

  PT_BEGIN(pt);

  for (i = 0; i < 3; i++) {
    digitalWrite(FRONT_LEFT_TURN, HIGH);
    digitalWrite(REAR_LEFT_TURN, HIGH);
    PT_SLEEP(pt, 75);
    digitalWrite(FRONT_LEFT_TURN, LOW);
    digitalWrite(REAR_LEFT_TURN, LOW);
    PT_SLEEP(pt, 85);
  }

  PT_END(pt);
}

void leftTurn() {
  count_right = 0;
  leftTurnSignal(&ptBlink);
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 215);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);
  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);

  while (count_right < 95 * 3) {
    nonblockingDelay(10);
  }

  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);
  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);
  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void reverse() {
  digitalWrite(REAR_LEFT_REVERSE, HIGH);
  digitalWrite(REAR_RIGHT_REVERSE, HIGH);
  nonblockingDelay(1000);
  digitalWrite(REAR_LEFT_REVERSE, LOW);
  digitalWrite(REAR_RIGHT_REVERSE, LOW);
}

void brake(boolean on) {
  digitalWrite(REAR_RIGHT_BRAKE, on);
  digitalWrite(REAR_LEFT_BREAK, on);
}

float Kp = 0.5;
float Kd = 0;
float Ki = 0;

uint8_t rotations_left = 0;
uint8_t rotations_right = 0;

float integral = 0;

void Forward3ft() {
  count_left = 0;
  analogWrite(MotorPWM_A, base_left_speed);
  analogWrite(MotorPWM_B, base_right_speed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);

  while (float(count_left) / countPerRotation < float(line_length) / float(wheelCircumference)) {
    nonblockingDelay(10);
  }

  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW); 
}


//encoder reading to RPM
void loop() {
  // if (count_left >= 180) {
  //   count_left -= 180;
  //   rotations_left++;
  // }

  // if (count_right >= 180) {
  //   count_right -= 180;
  //   rotations_right++;
  // }

  // // const uint8_t wheelCircumference = 223.84;  //mm

  // //pidSpeedAdjust();

  // if (rotations_left * wheelCircumference >= line_length) {
  //   brake(true);
  //   left_speed = 0;
  // }
  // if (rotations_right * wheelCircumference >= line_length) {
  //   brake(true);
  //   right_speed = 0;
  // }
  // if (left_speed == 0 && right_speed == 0) {
  //   count_left = 0;
  //   count_right = 0;
  //   rotations_left = 0;
  //   rotations_right = 0;
  //   brake(false);
  //   right_speed = 0;
  //   left_speed = 0;
  // }
  // Forward();
}
