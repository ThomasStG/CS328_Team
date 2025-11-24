#include "protothreads.h"
#include "BlinkLight.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// setup the oled
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4  //left motor
#define MotorPWM_B 5  //right motor

// protothread for the turn signal
pt ptBlink;

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

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

#define PIXY_CAMERA 10
#define SONAR 11

#define BUTTON_ONE 10
#define BUTTON_TWO 12

const uint8_t countPerRotation = 180;
const uint8_t wheelCircumference = 223.84;  //mm
const uint32_t line_length = 600.14;        //TODO: measure the actual length

static volatile int16_t count_left = 0;
static volatile int16_t count_right = 0;

// 3.215 = (60sec/0.1sec)/(48gear ratio * 4pulses/rev)
float rotation = 3.125;
float RPM = 0;

// Define base speeds for left and right wheels for straight driving
uint8_t base_right_speed = 180;
uint8_t base_left_speed = 175;

uint8_t right_speed = 180;
uint8_t left_speed = 175;

float startTime = 0;

#define BAUD_RATE 9600

// Custom method to have a blocking whill convert to a non-blocking delay for project 2
void blockingDelay(unsigned long ms) {
  unsigned long startTime = millis();
  while (millis() - startTime < ms) {
    yield();
  }
}

/**
 * Setup Function. Runs at the start once.
 */
void setup() {

  //Initialize the OLED screen 
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.display();
  PT_INIT(&ptBlink);

  //Initialize the pinmodes for the motors

  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  //Initialize the pinmodes for the lights

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

  // Store start time to calculate total time with
  startTime = millis();
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
    //Update lights with the logic determining if they are on/off
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

/**
 * Function to turn left by 90 degrees and turn on the turn signal
 */
void leftTurn() {
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
    blockingDelay(10);
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
}

/**
 * Currently unused function to display reverse lights
 */
void reverse() {
  rearRightRev.on();
  rearLeftRev.on();
  blockingDelay(1000);
  rearRightRev.off();
  rearLeftRev.off();
}

/**
 * Function to turn on break lights
 */
void brake() {
  rearRightBrake.on();
  rearLeftBrake.on();
}


// Iterator to hold which iteration of the loop we are on
int iterator = 0;

/**
 * Function to go forward by 3 feet (line_length)
 *
 * - Sets motor speed to predefined values which cause straight movement.
 * - Counts rotations until desired distance is met.
 */
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
    blockingDelay(10);
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

/**
 * Main looping function to handle first four iterations then display time and brake lights
 */
void loop() {
  // prevent integer overflow by limiting counts to 180 (1 rotation)
  count_left = count_left % 180;
  count_right = count_right % 180;
  // If within the first four iterations move forward line distance and turn left
  if (iterator < 4) {
    //If iterator is lower than 4, call the forward 3ft and left turn functions
    Forward3ft();
    leftTurn();
    iterator++;

  } else if (iterator == 4) {
    // Display time and break
    brake();
    float endTime = millis();
    //Configure OLED display to display the total time in seconds 
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print((endTime - startTime) / 1000);
    display.print(" seconds");
    display.display();
    iterator++; //Increment iterator
  } else {
    brake(); //Turn on the brake lights
  }
}
