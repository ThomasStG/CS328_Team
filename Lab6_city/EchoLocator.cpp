#include "EchoLocator.h"

EchoLocator::EchoLocator(int trigPin, int echoPin, int motorPin)
    : _trigPin(trigPin), _echoPin(echoPin), _motorPin(motorPin) {
  PT_INIT(&_ptDistance);

  pinMode(_echoPin, INPUT);
  pinMode(_trigPin, OUTPUT);
  digitalWrite(_trigPin, LOW);
}

PT_THREAD(EchoLocator::getDistance(struct pt *pt, double *result)) {
  static unsigned long tStart;
  static unsigned long pulseStart;

  PT_BEGIN(pt);

  // Trigger pulse
  digitalWrite(_trigPin, LOW);
  tStart = micros();
  PT_WAIT_UNTIL(pt, micros() - tStart >= 2);

  digitalWrite(_trigPin, HIGH);
  tStart = micros();
  PT_WAIT_UNTIL(pt, micros() - tStart >= 10);

  digitalWrite(_trigPin, LOW);

  // Wait for echo HIGH (timeout ~25ms)
  pulseStart = micros();
  PT_WAIT_UNTIL(pt,
                digitalRead(_echoPin) == HIGH || micros() - pulseStart > 25000);

  if (digitalRead(_echoPin) == LOW) {
    *result = -1; // no echo
    PT_EXIT(pt);
  }

  // Measure HIGH pulse width (timeout)
  pulseStart = micros();
  PT_WAIT_UNTIL(pt,
                digitalRead(_echoPin) == LOW || micros() - pulseStart > 25000);

  if (digitalRead(_echoPin) == HIGH) {
    *result = -1; // stuck HIGH
    PT_EXIT(pt);
  }

  unsigned long duration = micros() - pulseStart;
  *result = duration * 0.034 / 2;

  PT_END(pt);
}

PT_THREAD(EchoLocator::search(struct pt *pt, int *result)) {
  static unsigned long t;
  static int state;
  static int mid, right, left;

  PT_BEGIN(pt);
  state = 0;

  while (1) {
    switch (state) {

    case 0:
      _motor.write(FORWARD_ANGLE);
      t = millis();
      state = 1;
      break;

    case 1:
      if (millis() - t < 200)
        break;
      PT_INIT(&ptEchoServo);
      state = 2;
      break;

    case 2:
      if (PT_SCHEDULE(getDistance(&ptEchoServo, &mid)))
        break;
      _motor.write(RIGHT_ANGLE);
      t = millis();
      state = 3;
      break;

    case 3:
      if (millis() - t < 200)
        break;
      PT_INIT(&ptEchoServo);
      state = 4;
      break;

    case 4:
      if (PT_SCHEDULE(getDistance(&ptEchoServo, &right)))
        break;
      _motor.write(LEFT_ANGLE);
      t = millis();
      state = 5;
      break;

    case 5:
      if (millis() - t < 200)
        break;
      PT_INIT(&ptEchoServo);
      state = 6;
      break;

    case 6:
      if (PT_SCHEDULE(getDistance(&ptEchoServo, &left)))
        break;

      if (right > left && right > mid) {
        Serial.println("Right");
      } else if (left > right && left > mid) {
        Serial.println("Left");
      }
      PT_EXIT(pt);
    }

    PT_YIELD(pt);
  }

  PT_END(pt);
}

int EchoLocator::getDistanceBlocking() {
  long duration;

  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(_trigPin, LOW);

  duration = pulseIn(_echoPin, HIGH, 2000);

  return duration * 0.034 / 2;
}
