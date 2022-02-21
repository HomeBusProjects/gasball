#pragma once

/* expose some sensors that were previously private to furball.cpp
 * we need these for display.cpp and this is the most expedient way to do it
 * this needs some redesign at a later time
 */
#include "sensors/bme680_sensor.h"
#include "sensors/pms_sensor.h"
#include "sensors/scd4x_sensor.h"
#include "sensors/tsl2561_sensor.h"

extern BME680_Sensor bme680;
extern SCD4x_Sensor scd4x;
extern PMS_Sensor pmsa003;
extern TSL2561_Sensor tsl2561;

#define DDC_AIR_SENSOR "org.homebus.experimental.air-sensor"
#define DDC_AIR_QUALITY_SENSOR "org.homebus.experimental.air-quality-sensor"
#define DDC_SYSTEM "org.homebus.experimental.system"
#define DDC_DIAGNOSTIC "org.homebus.experimental.diagnostic"
#define DDC_AQI_SENSOR_O3 "org.homebus.experimental.aqi-o3"
#define DDC_AQI_SENSOR_PM25 "org.homebus.experimental.aqi-pm25"
#define DDC_MICS6814_SENSOR "org.homebus.experimental.mics6814-sensor"
#define DDC_CO_SENSOR "org.homebus.experimental.co-sensor"     // carbon monoxide
#define DDC_CO2_SENSOR "org.homebus.experimental.co2-sensor"   // carbon dioxide
#define DDC_H2CO_SENSOR "org.homebus.experimental.h2co-sensor" // formaldehyde
#define DDC_H2S_SENSOR "org.homebus.experimental.h2s-sensor"   // hydrogen sulfide
#define DDC_NH3_SENSOR "org.homebus.experimental.nh3-sensor"   // ammonia
#define DDC_NO2_SENSOR "org.homebus.experimental.no2-sensor"   // nitrogen dioxide
#define DDC_O2_SENSOR "org.homebus.experimental.o2-sensor"     // oxygen
#define DDC_O3_SENSOR "org.homebus.experimental.o3-sensor"     // ozone
#define DDC_VOC_SENSOR "org.homebus.experimental.voc-sensor"   // generic volatile organic compounds

void furball_setup();
void furball_loop();
void furball_stream();
