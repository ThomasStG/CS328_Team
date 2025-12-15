#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Speedometer{
  public:
    Speedometer(uint8_t address);

    void init();
    void buildGauge();
    void update(float MPH);
    void clear();
  private:
    uint8_t _address;
    uint8_t _pointX;
    uint8_t _pointY;
    uint8_t _pointX1;
    uint8_t _pointY1;
    int _width;
    int _height;
    signed int _reset;
    Adafruit_SSD1306 *display;
};

#endif