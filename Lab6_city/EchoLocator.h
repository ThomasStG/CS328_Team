#ifndef ECHOLOCATOR_H
#define ECHOLOCATOR_H
#include <Arduino.h>
#include <Servo.h>
#include <protothreads.h>

class EchoLocator {
public:
  EchoLocator(int trigPin = 9, int echoPin = 10, int motorPin = 11);
  PT_THREAD(getDistance(struct pt *pt, int *result));
  void search();
  int getDistanceBlocking();

  pt _ptDistance;
  int _trigPin;
  int _echoPin;

  int _motorPin;
  Servo _motor;
};

#endif
