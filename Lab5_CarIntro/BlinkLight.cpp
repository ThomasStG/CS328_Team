#include "BlinkLight.h"

//Constructor to initalize the light using respective pin

BlinkLight::BlinkLight(uint8_t pin, unsigned long interval)
    : _pin(pin), _interval(interval), _state(false), _prevTime(0),
      _enabled(false) {}

void BlinkLight::begin() { pinMode(_pin, OUTPUT); } //begin assigns the pin to output uploaded functionality

void BlinkLight::setPin(uint8_t newPin) {
  _pin = newPin;
  pinMode(_pin, OUTPUT);
} //Switches pin FOR TESTING PURPOSES

void BlinkLight::update() {
  if (_enabled) { //Checks if the light is 
    if (_interval == 0) {
      digitalWrite(_pin, HIGH);
      return;
    } //If the interval is set to 0, then the LED light is enabled until off is called
    unsigned long now = millis();
    if (now - _prevTime >= _interval) {
      _state = !_state;
      digitalWrite(_pin, _state);
      _prevTime = now;
    } //compares RTOS milliseconds that have passed, and switches the state if the difference in the time of the previous state and the time of the current state
  }
}

void BlinkLight::on() { _enabled = true; } //Enables LED light

void BlinkLight::off() {
  _state = false;
  _enabled = false;
} //Disables LED Light
