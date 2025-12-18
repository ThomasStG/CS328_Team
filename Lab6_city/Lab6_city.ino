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
    frontLeftHigh.update();

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
  PT_SCHEDULE(lineFollow(&ptLine));
  PT_SCHEDULE(turnSignal(&ptLights));
  if (Serial3.available()) {
    String msg = "";
    while (Serial3.available()) {
      char c = Serial3.read();
      msg += c;
    }

  static unsigned long lastSearch = 0;
  if (searchActive && millis() - lastSearch >= 100) {
      
      lastSearch = millis();
      searchActive = PT_SCHEDULE(echoLocator.search(&ptEcho, searchResult));
  }
  if (!searchActive) {
    PT_SCHEDULE(lineFollow(&ptLine));
  }
  //setMotors(leftRatio, rightRatio);



  long double = -1; 
  PT_SCHEDULE(echoLocator.getDistance(&ptEcho, &distance));
  if (distance > 0 && distance < 100) {
    searchActive = true;
  }
}
