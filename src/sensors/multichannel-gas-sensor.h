#pragma once

#include "sensor.h"

#include <Multichannel_Gas_GMXXX.h>
#include <Wire.h>

class Multichannel_Gas_Sensor : public Sensor {
  public:
    Multichannel_Gas_Sensor(uint16_t update_frequency, uint16_t accuracy, uint16_t precision, boolean calibrated)
        : Sensor(update_frequency, accuracy, precision, calibrated){};

    void begin();
    void handle();

    uint32_t co() {
        _mark_read();
        return _co;
    };
    uint32_t no2() {
        _mark_read();
        return _no2;
    };
    uint32_t c2h5oh() {
        _mark_read();
        return _c2h5oh;
    };
    uint32_t voc() {
        _mark_read();
        return _voc;
    };

  private:
    GAS_GMXXX<TwoWire> _gas;

    uint32_t _co;
    uint32_t _no2;
    uint32_t _c2h5oh;
    uint32_t _voc;
};
