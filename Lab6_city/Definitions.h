#ifndef DEFINITIONS_H

#define DEFINITIONS_H
#include <protothreads.h>
#include <stdint.h>

#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#define BAUD_RATE 9600

#define MotorPWM_A 4 // left motor
#define MotorPWM_B 5 // right motor

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

#define BUZZER 12

#define LEFT_ENCODER_FRONT 2
#define LEFT_ENCODER_REAR 3

#define RIGHT_ENCODER_FRONT 18
#define RIGHT_ENCODER_REAR 19

#define SONAR_ECHO 9
#define SONAR_TRIGGER 11
#define SONAR_MOTOR 4

#define BUTTON_TWO 12

#define LINE_SENSOR_LEFT 8
#define LINE_SENSOR_CENTER 7
#define LINE_SENSOR_RIGHT 6

#define FORWARD_ANGLE 70
#define LEFT_ANGLE 30
#define RIGHT_ANGLE 120

extern float countPerRotation;
extern float wheelCircumference; // mm

extern volatile int16_t count_left;
extern volatile int16_t count_right;

extern uint8_t base_right_speed;
extern uint8_t base_left_speed;

extern uint8_t right_speed;
extern uint8_t left_speed;

extern struct pt ptMusic;
extern struct pt ptLine;
extern struct pt ptEcho;
extern struct pt ptEchoServo;
#endif
