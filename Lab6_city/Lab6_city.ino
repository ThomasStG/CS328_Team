#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Pixy2.h>
#include <Servo.h>

#include "protothreads.h"
#include "Definitions.h"
#include "BlinkLight.h"
#include "EchoLocator.h"
#include "Speedometer.h"
#include "LightControl.h"

#include "Music.h"
#include "TheLionSleepsTonight.h"

EchoLocator echoLocator(SONAR_TRIGGER, SONAR_ECHO);
TheLionSleepsTonight song(BUZZER);
Speedometer mphGauge(SCREEN_ADDRESS);

struct pt ptMusic;
struct pt ptLine;
struct pt ptEcho;
struct pt ptEchoServo;

bool searchActive = false;
int searchResult = 0;

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

/**
 * Function to turn right by 90 degrees and turn on the turn signal
 */
PT_THREAD(turnRight(struct pt *pt)) {
  PT_BEGIN(pt);
  // reset count_right to correctly count turn distance
  count_right = 0;
  //Enable Blinking lights
  frontLeftTurn.on();
  rearLeftTurn.on();
  // Set left motor to stopa nd right motor to accelerate
  analogWrite(MotorPWM_A, 215);
  analogWrite(MotorPWM_B, 0);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);
  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);

  // Measure rotations and blink leds
  while (count_right < 100 * 3) {
    //Update blinking lights
    PT_SCHEDULE(turnSignal(&ptBlink));
    PT_YIELD(pt);
  }

  // Reset wheel speeds
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

  PT_END(pt);
}

/**
 * Function to turn left by 90 degrees and turn on the turn signal
 */
PT_THREAD(turnLeft(struct pt *pt)) {
  PT_BEGIN(pt);
  // reset count_right to correctly count turn distance
  count_right = 0;
  //Enable Blinking lights
  frontLeftTurn.on();
  rearLeftTurn.on();
  // Set left motor to stopa nd right motor to accelerate
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 215);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);
  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);

  // Measure rotations and blink leds
  while (count_right < 100 * 3) {
    //Update blinking lights
    PT_SCHEDULE(turnSignal(&ptBlink));
    PT_YIELD(pt);
  }

  // Reset wheel speeds
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

  PT_END(pt);
}

// Method: Forward
// Input: speed – value [0-255]
// Rotate the motor in a clockwise fashion
void Forward() {
  float avg_speed = (left_speed + right_speed) / 2.0f;
  mphGauge.update(avg_speed);

  frontLeftHigh.on();
  frontLeftLow.on();
  frontRightHigh.on();
  frontRightLow.on();


  analogWrite(MotorPWM_A, left_speed);
  analogWrite(MotorPWM_B, right_speed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

static PT_THREAD(lineSensor(struct pt *ptLine)) {
  
  PT_BEGIN(ptLine);
  int left;
  int center;
  int right;
  int lineStatus;
  
  while(1){
    delay(5);
    
    left = digitalRead(LINE_SENSOR_LEFT);
    center = digitalRead(LINE_SENSOR_CENTER);
    right = digitalRead(LINE_SENSOR_RIGHT);
    
    lineStatus = (left << 2) | (center << 1) | right;
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
    }
      
    PT_YIELD(ptLine);
  }
 
  PT_END(ptLine);
}

static PT_THREAD(lineFollow(struct pt *pt)) {
  static int left, right, center, lineStatus;

  PT_BEGIN(pt);

  while (1) {
    left = digitalRead(LINE_SENSOR_LEFT);
    center = digitalRead(LINE_SENSOR_CENTER);
    right = digitalRead(LINE_SENSOR_RIGHT);

    lineStatus = (left << 2) | (center << 1) | right;

    switch (lineStatus) {
      case 0b001:
        Serial.println("Right");
        left_speed = base_left_speed + 50;
        right_speed = base_right_speed;
        break;
      case 0b011:
        Serial.println("RIGHT");
        left_speed = base_left_speed + 50;
        right_speed = base_right_speed;
        break;
      case 0b100:
        Serial.println("Left");
        left_speed = base_left_speed;
        right_speed = base_right_speed + 50;
        break;
      case 0b110:
        Serial.println("LEFT");
        left_speed = base_left_speed;
        right_speed = base_right_speed + 50;
        break;
      case 0b000:
      case 0b010:
        Serial.println("FORWARD");
        left_speed = base_left_speed;
        right_speed = base_right_speed;
        break;
      case 0b101:
      case 0b111:
      default:
        Serial.println("STOP");
        left_speed = 0;
        right_speed = 0;
        brake();
        break;
    }
    Forward();
    PT_YIELD(pt);
  }

  PT_END(pt);
}

Servo servo;

void setup() {
  mphGauge.init();
  PT_INIT(&ptMusic);
  PT_INIT(&ptLine);
  PT_INIT(&ptEcho);
  PT_INIT(&ptEchoServo);

  song.begin();

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
  /*
  float testVal;
  for(int i = 0; i <= 255; i++){
    delay(5);
    testVal = i * 1.0f;
    mphGauge.update(testVal);
  }
  */
  servo.attach(4);
  servo.write(70);
  echoLocator.setMotor(&servo);
}

void loop() {
  // if (Serial3.available()) {
  //   String msg = "";
  //   while (Serial3.available()) {
  //     char c = Serial3.read();
  //     msg += c;
  //   }

  //   Serial.println(msg);
  // }
  
  PT_SCHEDULE(song.play(&ptMusic));

  static unsigned long lastSearch = 0;
  if (searchActive && millis() - lastSearch >= 100) {
      
      lastSearch = millis();
      searchActive = PT_SCHEDULE(echoLocator.search(&ptEcho, searchResult));
  }
  if (!searchActive) {
    PT_SCHEDULE(lineFollow(&ptLine));
  }

  long double = -1; 
  PT_SCHEDULE(echoLocator.getDistance(&ptEcho, &distance));
  if (distance > 0 && distance < 100) {
    searchActive = true;
  }
}
