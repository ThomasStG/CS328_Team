#include "BlinkLight.h"

BlinkLight::BlinkLight(uint8_t pin, unsigned long interval)
  : _pin(pin), _interval(interval), _state(false), _prevTime(0), _enabled(false) {}

void BlinkLight::begin() {
  pinMode(_pin, OUTPUT);
}

void BlinkLight::setPin(uint8_t newPin) {
  _pin = newPin;
  pinMode(_pin, OUTPUT);
}

void BlinkLight::update(){
  if(_enabled){
    if(_interval == 0){
      digitalWrite(_pin, HIGH);
      return;
    }
    unsigned long now = millis();
    if (now - _prevTime >= _interval){
      _state = !_state;
      digitalWrite(_pin,_state);
      _prevTime = now;
    }
  }
}

void BlinkLight::on(){
  _enabled = true;
}

void BlinkLight::off(){
  _state = false;
  _enabled = false;
}