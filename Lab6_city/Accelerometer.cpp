#include "Accelerometer.h"
#include <Wire.h>

Accelerometer::Accelerometer(uint8_t address)
  : _address(address), _pointX(0), _point(0), _width(128), _height(32), _reset(-1) {}

Accelerometer::begin(){Adafruit_SSD1306 display(_width, _height, &Wire, _reset);}

Accelerometer::buildGauge(){}

Accelerometer::update(uint8_t RPM){}

Accelerometer::clear(){}