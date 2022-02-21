#include "sensors/ads1115_sensor.h"

/*
 * the ADS1115 datasheet suggests the following configurations:
 *
 * 4 channel ADC, commonly used on furballs to hook up MQ series gas sensors
 *
 */
void ADS1115_Sensor::begin() {
  _present = _ads1115.begin();
  if(!_present)
    _status = SENSOR_NOT_FOUND;
}

void ADS1115_Sensor::handle() {
  if(!_present)
    return;
  
  _channel0 = _ads1115.readADC_SingleEnded(0);
  _channel1 = _ads1115.readADC_SingleEnded(0);
  _channel2 = _ads1115.readADC_SingleEnded(0);
  _channel3 = _ads1115.readADC_SingleEnded(0);
}
