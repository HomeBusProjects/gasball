#include "multichannel-gas-sensor.h"

void Multichannel_Gas_Sensor::begin() {
  byte error;

  Wire.beginTransmission(0x08);
  error = Wire.endTransmission();
  Serial.print("i2c probe ");
  Serial.println(error);
  if(error == 0) {
    _present = true;
  } else {
    _present = false;
    return;
  }

  _gas.begin(Wire, 0x08);
  _gas.preheated();
;}

void Multichannel_Gas_Sensor::handle() {
  if(!_present)
    return;

  _voc = _gas.measure_VOC();
  _no2 = _gas.measure_NO2();
  _co = _gas.measure_CO();
  _c2h5oh = _gas.measure_C2H5OH();
}
