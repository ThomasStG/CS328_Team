#include "protothreads.h"
#include "BlinkLight.h"
#include "EchoLocator.h"
#include "Speedometer.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Pixy2.h>
#include "Music.h"
#include <math.h>

#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1
#include <Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#include "TheLionSleepsTonight.h"

EchoLocator echoLocator(9, 10, 11);

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4  //left motor
#define MotorPWM_B 5  //right motor

pt ptMusic;
pt ptLine;
pt ptCamera;
pt ptLights;

int buzzer = 12;
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

TheLionSleepsTonight song(12);

Speedometer mphGauge(SCREEN_ADDRESS);

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

#define SONAR_ECHO 9
#define SONAR_TRIGGER 10
#define SONAR_MOTOR 11

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

uint8_t base_speed = 70;

uint8_t base_right_speed = 70;
uint8_t base_left_speed = 65;

uint8_t right_speed = 70;
uint8_t left_speed = 65;

float leftRatio = 1.0f;
float rightRatio = 1.0f;

bool is_turning = false;

float startTime = 0;
#define BAUD_RATE 9600


void nonblockingDelay(unsigned long ms) {
  unsigned long startTime = millis();
  while (millis() - startTime < ms) {
    yield();
  }
}

void setMotors(float leftRatio, float rightRatio) {
    frontLeftLow.on();
    frontRightLow.on();
    leftRatio  = constrain(leftRatio, -1.0f, 1.0f);
    rightRatio = constrain(rightRatio, -1.0f, 1.0f);

    int leftPWM  = (int)(base_speed * fabs(leftRatio));
    int rightPWM = (int)(base_speed * fabs(rightRatio));

    int avgPWM = leftPWM * rightPWM;
    mphGauge.update(avgPWM);


    analogWrite(MotorPWM_A, leftPWM);
    analogWrite(MotorPWM_B, rightPWM);


    digitalWrite(INA1A, leftRatio >= 0 ? HIGH : LOW);
    digitalWrite(INA2A, leftRatio >= 0 ? LOW  : HIGH);
    digitalWrite(INA1B, rightRatio >= 0 ? HIGH : LOW);
    digitalWrite(INA2B, rightRatio >= 0 ? LOW  : HIGH);
}



static PT_THREAD(turnSignal(struct pt *pt1)) {


  PT_BEGIN(pt1);


  while (1) {
    Serial.println("Updating lights");
    frontRightTurn.update();
    frontLeftTurn.update();
    rearRightTurn.update();
    rearLeftTurn.update();

    frontLeftHigh.update();
    frontLeftLow.update();
    frontRightHigh.update();
    frontRightLow.update();

    rearRightBrake.update();
    rearLeftBrake.update();
    rearRightRev.update();
    rearLeftRev.update();

    PT_YIELD(pt1);
  }

  PT_END(pt1);
}

unsigned long prevTime = 0;



void leftTurn() {
  count_right = 0;
  frontLeftTurn.on();
  rearLeftTurn.on();
  
  is_turning = true;
  leftRatio  = -0.9f;
  rightRatio = 1.0f;
  nonblockingDelay(500);
  is_turning = false;
  
  

  frontLeftTurn.off();
  rearLeftTurn.off();
}

void reverse() {
  rearRightRev.on();
  rearLeftRev.on();
  delay(1000);
  rearRightRev.off();
  rearLeftRev.off();
}

void brake() {
  count_left = 0;
  count_right = 0;
  rearRightBrake.on();
  rearLeftBrake.on();
}

static PT_THREAD(lineFollow(struct pt *pt2)) {
    static int left, center, right;
    static int lastTurn = 0;  // -1 = left, +1 = right

    PT_BEGIN(pt2);

    while (1) {
        left   = digitalRead(LINE_SENSOR_LEFT);
        center = digitalRead(LINE_SENSOR_CENTER);
        right  = digitalRead(LINE_SENSOR_RIGHT);

        int lineStatus = (left << 2) | (center << 1) | right;

        switch (lineStatus) {
            case 0b001:
                frontRightTurn.on();
                rearRightTurn.on();
                frontLeftTurn.off();
                rearLeftTurn.off();
                lastTurn = +1;
                leftRatio  = 1.0f;   // left wheel forward
                rightRatio = -0.9f;  // right wheel backward
                break;
            case 0b011:
                frontRightTurn.on();
                rearRightTurn.on();
                frontLeftTurn.off();
                rearLeftTurn.off();
                lastTurn = +1;
                leftRatio  = 1.0f;   // left wheel forward
                rightRatio = -0.8f;  // right wheel backward
                break;

            // Line to the LEFT → turn left
            case 0b100:
                frontLeftTurn.on();
                rearLeftTurn.on();
                frontRightTurn.off();
                rearRightTurn.off();
                lastTurn = -1;
                leftRatio  = -0.9f;  // left wheel backward
                rightRatio = 1.0f;   // right wheel forward
                break;
            case 0b110:
                frontLeftTurn.on();
                rearLeftTurn.on();
                frontRightTurn.off();
                rearRightTurn.off();
                lastTurn = -1;
                leftRatio  = -0.8f;  // left wheel backward
                rightRatio = 1.0f;   // right wheel forward
                break;

            //Forward
            case 0b010:
                frontLeftTurn.off();
                rearLeftTurn.off();
                frontRightTurn.off();
                rearRightTurn.off();
                leftRatio  = 1.0f;
                rightRatio = 1.0f;
                break;

            //Redirection
            case 0b000:
                frontLeftTurn.off();
                rearLeftTurn.off();
                frontRightTurn.off();
                rearRightTurn.off();
                if (lastTurn > 0) {
                    leftRatio  = 1.0f;
                    rightRatio = -0.6f;
                } else if (lastTurn < 0) {
                    leftRatio  = -0.6f;
                    rightRatio = 1.0f;
                }
                break;

            default:
                break;
        }

        setMotors(leftRatio, rightRatio);
        PT_YIELD(pt2);
    }

    PT_END(pt2);
}
Servo s;

void setup() {
  mphGauge.init();
  PT_INIT(&ptMusic);
  PT_INIT(&ptLights);
  PT_INIT(&ptLine);
  
  song.begin();
  
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);

  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(LINE_SENSOR_LEFT, INPUT_PULLUP);
  pinMode(LINE_SENSOR_CENTER, INPUT_PULLUP);
  pinMode(LINE_SENSOR_RIGHT, INPUT_PULLUP);


  frontRightTurn.begin();
  frontLeftTurn.begin();
  rearRightTurn.begin();
  rearLeftTurn.begin();

  frontLeftHigh.begin();
  frontLeftLow.begin();
  frontRightHigh.begin();
  frontRightLow.begin();

  rearRightBrake.begin();
  rearLeftBrake.begin();
  rearRightRev.begin();
  rearLeftRev.begin();

  pinMode(LEFT_ENCODER_FRONT, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_FRONT, INPUT_PULLUP);

  Serial.begin(BAUD_RATE);
  Serial.println("Starting");

  Serial3.begin(BAUD_RATE);
}

void loop() {
  PT_SCHEDULE(song.play(&ptMusic));
  PT_SCHEDULE(lineFollow(&ptLine));
  PT_SCHEDULE(turnSignal(&ptLights));
  if (Serial3.available()) {
    String msg = "";
    while (Serial3.available()) {
      char c = Serial3.read();
      msg += c;
    }

    Serial.println(msg);
  }
  //setMotors(leftRatio, rightRatio);



  int distance;
  // echoLocator.getDistance(&(echoLocator._ptDistance), &distance);
  // Serial.println(distance);
  //

  /*
  for (int pos = 0; pos <= 180; pos += 5) {
    s.write(pos);
    delay(20);
  }
  for (int pos = 180; pos >= 0; pos -= 5) {
    s.write(pos);
    delay(20);
  }
  */
  /*
  int left = digitalRead(LINE_SENSOR_LEFT);
  int center = digitalRead(LINE_SENSOR_CENTER);
  int right = digitalRead(LINE_SENSOR_RIGHT);

  int lineStatus = (left << 2) | (center << 1) | right;
  Serial.print("Line Status: ");
  Serial.print(left);
  Serial.print(center);
  Serial.println(right);
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

      break;
  }*/   
  
}
