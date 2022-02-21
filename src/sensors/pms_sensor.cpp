#include "hw.h"
#include "pms_sensor.h"

void PMS_Sensor::begin() {
  if(!_pms.begin_I2C()) {
    _status = SENSOR_NOT_FOUND;
    _present = false;

    return;
  }

  _present = true;
}

#define PMS_READ_DELAY 1000

void PMS_Sensor::handle() {
  if(!_present)
    return;

  if(millis() - _last_read_request < PMS_READ_DELAY)
    return;

  _pms.read(&_data);

  _last_read_request = millis();
}
