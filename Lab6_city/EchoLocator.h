#ifndef ECHOLOCATOR_H
#define ECHOLOCATOR_H
#include "Definitions.h"
#include <Arduino.h>
#include <Servo.h>
#include <protothreads.h>

class EchoLocator {
public:
  EchoLocator(int trigPin = 11, int echoPin = 9, int motorPin = 10);

  PT_THREAD(getDistance(struct pt *pt, double *result));

  PT_THREAD(search(struct pt *pt, int *result));

  void search();
  int getDistanceBlocking();

  pt _ptDistance;
  int _trigPin;
  int _echoPin;

  int _motorPin;
  Servo _motor;

  void write(int angle) { _motor.write(angle); }

  void setMotor(Servo *motor) { _motor = *motor; }
};

#endif
