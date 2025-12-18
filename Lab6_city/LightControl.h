#include "Definitions.h"
#include <Arduino.h>
#include <protothreads.h>

BlinkLight frontLeftTurn(49, 100);
BlinkLight frontLeftHigh(47, 0);
BlinkLight frontLeftLow(45, 0);
BlinkLight frontRightTurn(43, 100);
BlinkLight frontRightHigh(41, 0);
BlinkLight frontRightLow(39, 0);
BlinkLight rearRightTurn(37, 100);
BlinkLight rearRightRev(35, 0);
BlinkLight rearRightBrake(33, 0);
BlinkLight rearLeftTurn(31, 100);
BlinkLight rearLeftRev(29, 0);
BlinkLight rearLeftBrake(27, 0);

static PT_THREAD(turnSignal(struct pt *pt)) {

  PT_BEGIN(pt);

  while (1) {
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
    delay(10);
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
