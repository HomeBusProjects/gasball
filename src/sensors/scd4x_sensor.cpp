#include "scd4x_sensor.h"

SCD4x _scd4x(SCD4x_SENSOR_SCD40);

void SCD4x_Sensor::begin() {
    if(!_scd4x.begin()) {
        Serial.println("Could not find a valid SCD4x sensor, check wiring!");
        _status = SENSOR_NOT_FOUND;

        _present = false;
        return;
    }

    _present = true;
    ;
}

void SCD4x_Sensor::handle() {
    if(!_present)
        return;

    _temperature = _scd4x.getTemperature();
    _humidity = _scd4x.getHumidity();
    _co2 = _scd4x.getCO2();
}
