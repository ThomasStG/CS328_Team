#include "EchoLocator.h"

// Constructor to initalize the class and necessary pins
EchoLocator::EchoLocator(int trigPin, int echoPin, int motorPin)
    : _trigPin(trigPin), _echoPin(echoPin), _motorPin(motorPin) {
  PT_INIT(&_ptDistance);

  pinMode(motorPin, OUTPUT);
  _motor.attach(motorPin);

  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

// Thread to get the distance from the ultrasonic sensor blocking
PT_THREAD(EchoLocator::getDistance(struct pt *pt, int *result)) {
  PT_BEGIN(pt);

  long duration;

  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(_trigPin, LOW);

  duration = pulseIn(_echoPin, HIGH, 2000);

  *result = duration * 0.034 / 2;

  PT_END(pt);
}

// Old code to Search with the ultrasonic sensor blocking
void EchoLocator::search() {
  int distanceMid, distanceRight, distanceLeft;
  _motor.write(90);
  delay(500);
  distanceMid = getDistanceBlocking();

  delay(500);
  _motor.write(135);
  delay(500);
  distanceRight = getDistanceBlocking();

  delay(500);
  _motor.write(45);
  delay(500);
  distanceLeft = getDistanceBlocking();

  if (distanceRight > distanceLeft && distanceRight > distanceMid) {
    Serial.println("Right");
  } else if (distanceLeft > distanceRight && distanceLeft > distanceMid) {
    Serial.println("Left");
  }
}
