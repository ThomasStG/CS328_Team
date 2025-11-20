#ifndef BLINKLIGHT_H
#define BLINKLIGHT_H

#include <Arduino.h>
//Header file that handles all of the led lights

class BlinkLight{
  public:
    BlinkLight(uint8_t pin, unsigned long interval);

    void begin();
    void setPin(uint8_t newPin);
    void update();
    void on();
    void off();
  private:
    uint8_t _pin;
    unsigned long _interval;
    unsigned long _prevTime;
    bool _state;
    bool _enabled;
};

#endif