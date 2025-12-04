#include "protothreads.h"
#include "BlinkLight.h"
#include "Accelerometer.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Pixy2.h>
#include "Music.h"
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32



// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4  //left motor
#define MotorPWM_B 5  //right motor

pt ptMusic;
pt ptLine;
pt ptEcho;
pt ptCamera;

int buzzer = 12;
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

Accelerometer rpmGauge(SCREEN_ADDRESS);

//#define FRONT_LEFT_TURN 49
BlinkLight frontLeftTurn(49, 100);
//#define FRONT_LEFT_HIGH 47
BlinkLight frontLeftHigh(47, 0);
//#define FRONT_LEFT_LOW 45
BlinkLight frontLeftLow(45, 0);
//#define FRONT_RIGHT_TURN 43
BlinkLight frontRightTurn(43, 100);
//#define FRONT_RIGHT_HIGH 41
BlinkLight frontRightHigh(41, 0);
//#define FRONT_RIGHT_LOW 39
BlinkLight frontRightLow(39, 0);
//#define REAR_RIGHT_TURN 37
BlinkLight rearRightTurn(37, 100);
//#define REAR_RIGHT_REVERSE 35
BlinkLight rearRightRev(35, 0);
//#define REAR_RIGHT_BRAKE 33
BlinkLight rearRightBrake(33, 0);
//#define REAR_LEFT_TURN 31
BlinkLight rearLeftTurn(31, 100);
//#define REAR_LEFT_REVERSE 29
BlinkLight rearLeftRev(29, 0);
//#define REAR_LEFT_BREAK 27
BlinkLight rearLeftBrake(27, 0);

#define LEFT_ENCODER_FRONT 2
#define LEFT_ENCODER_REAR 3

#define RIGHT_ENCODER_FRONT 18
#define RIGHT_ENCODER_REAR 19

#define PIXY_CAMERA_MOTOR 9
#define SONAR_MOTOR 11

#define SONAR_ECHO 5
#define SONAR_TRIGGER 4

#define BUTTON_ONE 10
#define BUTTON_TWO 12

#define LINE_SENSOR_LEFT 8
#define LINE_SENSOR_CENTER 7
#define LINE_SENSOR_RIGHT 6

const uint8_t countPerRotation = 180;
const uint8_t wheelCircumference = 223.84;  //mm
const uint32_t line_length = 600.14;


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

float startTime = 0;

#define BAUD_RATE 9600

void nonblockingDelay(unsigned long ms) {
  unsigned long startTime = millis();
  while (millis() - startTime < ms) {
    yield();
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

static PT_THREAD(turnSignal(struct pt *pt)) {


  PT_BEGIN(pt);


  while (1) {
    Serial.println("Updating lights");
    frontRightTurn.update();
    frontLeftTurn.update();
    rearRightTurn.update();
    rearLeftTurn.update();

    frontLeftHigh.update();
    frontLeftLow.update();
    frontRightHigh.update();
    frontLeftHigh.update();

    rearRightBrake.update();
    rearLeftBrake.update();
    rearRightRev.update();
    rearLeftRev.update();

    PT_YIELD(pt);
  }

  PT_END(pt);
}

unsigned long prevTime = 0;


void leftTurn() {
  count_right = 0;
  frontLeftTurn.on();
  rearLeftTurn.on();
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 215);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);
  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);

  while (count_right < 100 * 3) {
    PT_SCHEDULE(turnSignal(&ptLine));
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

  frontLeftTurn.off();
  rearLeftTurn.off();
}

void reverse() {
  rearRightRev.on();
  rearLeftRev.on();
  nonblockingDelay(1000);
  rearRightRev.off();
  rearLeftRev.off();
}

void brake() {
  rearRightBrake.on();
  rearLeftBrake.on();
}

uint8_t rotations_left = 0;
uint8_t rotations_right = 0;

int iterator = 0;

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

void setup() {
  PT_INIT(&ptMusic);
  PT_INIT(&ptLine);
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_CENTER, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);


  frontRightTurn.begin();
  frontLeftTurn.begin();
  rearRightTurn.begin();
  rearLeftTurn.begin();

  frontLeftHigh.begin();
  frontLeftLow.begin();
  frontRightHigh.begin();
  frontLeftHigh.begin();

  rearRightBrake.begin();
  rearLeftBrake.begin();
  rearRightRev.begin();
  rearLeftRev.begin();

  pinMode(LEFT_ENCODER_FRONT, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_FRONT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_FRONT), ISRMotorLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_FRONT), ISRMotorRight, FALLING);
  Serial.begin(BAUD_RATE);
  Serial.println("Starting");
  Serial3.begin(BAUD_RATE);
}

void loop() {
  if (Serial3.available()) {
    String msg = "";
    while (Serial3.available()) {
      char c = Serial3.read();
      msg += c;
    }

    Serial.println(msg);
  }
  int left = digitalRead(LINE_SENSOR_LEFT);
  int center = digitalRead(LINE_SENSOR_CENTER);
  int right = digitalRead(LINE_SENSOR_RIGHT);

  int lineStatus = (left << 2) | (center << 1) | right;

  Serial.println(lineStatus);
  switch (lineStatus) {
    case 0b100:
      left_speed = base_left_speed;
      right_speed = 0;
      break;
    case 0b001:
      left_speed = 0;
      right_speed = base_right_speed;
      break;
    case 0b111:
    case 0b010:
      left_speed = base_left_speed;
      right_speed = base_right_speed;
      break;
    case 0b011:
      left_speed = base_left_speed / 2;
      right_speed = base_right_speed;
      break;
    case 0b110:
      left_speed = base_left_speed;
      right_speed = base_right_speed / 2;
      break;
    case 0b101:
    case 0b000:
    default:
      left_speed = 0;
      right_speed = 0;
      brake();
      break;
  }
  Forward();
}
