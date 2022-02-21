#pragma once

#include <Arduino.h>

#include "sensor.h"

#include <Adafruit_PM25AQI.h>

class PMS_Sensor : public Sensor {
 public:
  PMS_Sensor(uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated) : Sensor(update_frequency, accuracy, precision, calibrated) {};

  void begin();
  void handle();

  uint16_t density_1_0() { _mark_read(); return _data.particles_10um; };
  uint16_t density_2_5() { _mark_read(); return _data.particles_25um; };
  uint16_t density_10_0() { _mark_read(); return _data.particles_100um; };

 private:
  Adafruit_PM25AQI _pms;

  PM25_AQI_Data _data;

  uint32_t _last_read_request = 0;
};
