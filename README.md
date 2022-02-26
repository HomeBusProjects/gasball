# Gasball

[![PlatformIO CI](https://github.com/HomeBusProjects/gasball/actions/workflows/main.yml/badge.svg)](https://github.com/HomeBusProjects/gasball/actions/workflows/main.yml)

Gasball is an open-source hardware project intended to help with the research and development of [Homebus](https://github.com/HomeBusProjects), an IoT auto-provisioning framework and data processing network built on top of MQTT. Note that Homebus is not even alpha quality software at this time and is still undergoing architectural changes and development.

Gasball is based on the [Furball](https://github.com/HomeBusProjects/furball), an homage to Dave Mills' ["Fuzzball"](https://en.wikipedia.org/wiki/Fuzzball_router), one of the first routers on the nascent Internet.

## Sensors

Gasball currently supports:
- BME680/BME688 air temperature/humidity/pressure/volatile organic compound sensor
- PMSA003I I2C particle counter
- SFA30 Sensirion formaldehyde sensor
- SCD41 Sensirion CO2 sensor
- DFRobot calibrated gas sensors (currently using CO, O3, NH3, NO2, H2S sensors)


## Software

Furball's software is built on the Arduino Core using [PlatformIO](https://platformio.org/) as its build system. I prefer PlatformIO because it's fast, has a good library manager, lets you build for arbitrary platforms without having to manually install support for them, and lets you use your editor of choice.

Create a `src/config.h` file based on `src/config.h-example` and configure Wifi and the MQTT broker.

Once you've installed PlatformIO you can build the Furball firmware by running:
```
platformio run
```

in its top level directory.

To flash the firmware, run:
```
platformio run -t upload
```

It currently supports OTA updates.

# License

Software and documentation is licensed under the [MIT license](https://romkey.mit-license.org/).

Circuits are licensed under [Creative Commons Attribution Share-Alike license](https://creativecommons.org/licenses/by-sa/4.0). 
