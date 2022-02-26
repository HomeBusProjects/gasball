#include "furball.h"

#include <Arduino.h>
#include <time.h>

#include <SoftwareSerial.h>
#include <Wire.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/homebus.h>
#include <multiball/wifi.h>

#ifdef USE_DIAGNOSTICS
#include <diagnostics.h>
#endif

#include "sensors/ads1115_sensor.h"
#include "sensors/bme680_sensor.h"
#include "sensors/multichannel-gas-sensor.h"
#include "sensors/pms_sensor.h"
#include "sensors/scd4x_sensor.h"

#include "sensors/tsl2561_sensor.h"

#include "sensors/uptime.h"

#include <DFRobot_MultiGasSensor.h>
#include <SensirionI2CSfa3x.h>

#include "display.h"
#include "storage.h"

#include "aqi.h"

Uptime uptime;

BME680_Sensor bme680(UPDATE_DELAY, 0, 0, false);
SCD4x_Sensor scd4x(UPDATE_DELAY, 0, 0, false);
PMS_Sensor pmsa003(UPDATE_DELAY, 0, 0, false);
ADS1115_Sensor ads1115(UPDATE_DELAY, 0, 0, false);
Multichannel_Gas_Sensor multichannel_gas(UPDATE_DELAY, 0, 0, false);

TSL2561_Sensor tsl2561(UPDATE_DELAY, 0, 0, false);

SensirionI2CSfa3x h2co_sensor;

#define O3_I2C_ADDRESS 0x68
#define H2S_I2C_ADDRESS 0x69
#define NH3_I2C_ADDRESS 0x6a
#define CO_I2C_ADDRESS 0x6b
#define NO2_I2C_ADDRESS 0x64

/* other devices
 * 0x12 PMSA003I particle sensor
 * 0x3c, 0x3d display
 * 0x5d formaldehyde sensor
 * 0x62 SCD41 CO2 sensor
 * 0x77 BME68x
 */

DFRobot_GAS_I2C dfrobot_h2s(&Wire, H2S_I2C_ADDRESS);
DFRobot_GAS_I2C dfrobot_nh3(&Wire, NH3_I2C_ADDRESS);
DFRobot_GAS_I2C dfrobot_o3(&Wire, O3_I2C_ADDRESS);
DFRobot_GAS_I2C dfrobot_co(&Wire, CO_I2C_ADDRESS);
DFRobot_GAS_I2C dfrobot_no2(&Wire1, NO2_I2C_ADDRESS);

void furball_setup() {
    Wire.begin(23, 22);

    storage_begin();

    ads1115.begin();
    if(ads1115.is_present())
        Serial.println("[ads1115]");
    else
        Serial.println("ADS1115 not found");

    bme680.begin();
    if(bme680.is_present())
        Serial.println("[bme680]");
    else
        Serial.println("BME680 not found");

    scd4x.begin();
    if(scd4x.is_present())
        Serial.println("[scd4x]");
    else
        Serial.println("SCD4x not found");

    multichannel_gas.begin();
    if(multichannel_gas.is_present())
        Serial.println("[multichannel gas]");
    else
        Serial.println("Multichannel gas sensor not found");

    // really need to rewrite to detect the presence of the PMSA003
    pmsa003.begin();
    if(pmsa003.is_present())
        Serial.println("[pma003]");
    else
        Serial.println("PMA003 not found");

    tsl2561.begin();
    if(tsl2561.is_present())
        Serial.println("[tsl2561]");
    else
        Serial.println("TSL2561 not found");

    h2co_sensor.begin(Wire);
    if(h2co_sensor.startContinuousMeasurement() == 0)
        Serial.println("[h2co]");
    else
        Serial.println("h2co sensor not found");

    if(display_begin())
        Serial.println("[display]");
    else
        Serial.println("Dsplay not found");

    String gas_type;

    dfrobot_h2s.begin();
    //  dfrobot_h2s.changeAcquireMode(dfrobot_h2s.PASSIVITY);
    dfrobot_h2s.setTempCompensation(dfrobot_h2s.ON);
    gas_type = dfrobot_h2s.queryGasType();
    if(gas_type == "H2S")
        Serial.println("[h2s sensor]");
    else
        Serial.printf("Expected H2S sensor, got %s\n", gas_type.c_str());

    dfrobot_nh3.begin();
    //  dfrobot_nh3.changeAcquireMode(dfrobot_nh3.PASSIVITY);
    dfrobot_nh3.setTempCompensation(dfrobot_nh3.ON);
    gas_type = dfrobot_nh3.queryGasType();
    if(gas_type == "NH3")
        Serial.println("[nh3 sensor]");
    else
        Serial.printf("Expected NH3 sensor, got %s\n", gas_type.c_str());

    dfrobot_o3.begin();
    //  dfrobot_o3.changeAcquireMode(dfrobot_o3.PASSIVITY);
    dfrobot_o3.setTempCompensation(dfrobot_o3.ON);
    gas_type = dfrobot_o3.queryGasType();
    if(gas_type == "O3")
        Serial.println("[o3 sensor]");
    else
        Serial.printf("Expected O3 sensor, got %s\n", gas_type.c_str());

    dfrobot_co.begin();
    //  dfrobot_co.changeAcquireMode(dfrobot_co.PASSIVITY);
    dfrobot_co.setTempCompensation(dfrobot_co.ON);
    gas_type = dfrobot_co.queryGasType();
    if(gas_type == "CO")
        Serial.println("[co sensor]");
    else
        Serial.printf("Expected CO sensor, got %s\n", gas_type.c_str());

    dfrobot_no2.begin();
    //  dfrobot_no2.changeAcquireMode(dfrobot_no2.PASSIVITY);
    dfrobot_no2.setTempCompensation(dfrobot_no2.ON);
    gas_type = dfrobot_no2.queryGasType();
    if(gas_type == "NO2")
        Serial.println("[no2 sensor]");
    else
        Serial.printf("Expected NO2 sensor, got %s\n", gas_type.c_str());

    homebus_set_provisioner(HOMEBUS_SERVER, HOMEBUS_AUTHENTICATION_TOKEN);

    static const char *publishes[] = {
        DDC_AIR_SENSOR,    DDC_AIR_QUALITY_SENSOR, DDC_CO2_SENSOR, DDC_SYSTEM,      DDC_DIAGNOSTIC, DDC_AQI_SENSOR_PM25,
        DDC_AQI_SENSOR_O3, DDC_CO_SENSOR,          DDC_NO2_SENSOR, DDC_H2CO_SENSOR, DDC_CO_SENSOR,  DDC_NH3_SENSOR,
        DDC_H2S_SENSOR,    DDC_O2_SENSOR,          DDC_O3_SENSOR,  DDC_VOC_SENSOR,  DDC_VOC_SENSOR, NULL};
    static const char *consumes[] = {NULL};
    static char mac_address[3 * 6];

    strncpy(mac_address, App.mac_address().c_str(), 3 * 6);

    // this is... wrong - needs to be sorted for proper Homebus use
    homebus_configure("Homebus", "Gasball Air Quality Monitor", mac_address, "", publishes, consumes);

    homebus_setup();
    Serial.println("[homebus]");
}

static bool furball_mics6814_update(char *buf, size_t buf_len) {
    if(!ads1115.is_present())
        return false;

    uint32_t co = 0;
    uint32_t nh3 = 0;
    uint32_t no2 = 0;

    for(int i = 0; i < 3; i++) {
        co += ads1115.channel0();
        nh3 += ads1115.channel1();
        no2 += ads1115.channel2();
        delay(1000);
    }

    co /= 3;
    nh3 /= 3;
    no2 /= 3;

    snprintf(buf, buf_len, "{ \"co\": %u, \"nh3\": %u, \"no2\": %u }", co, nh3, no2);

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_air_update(char *buf, size_t buf_len) {
    if(!bme680.is_present())
        return false;

    snprintf(buf, buf_len, "{ \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f }",
#ifdef TEMPERATURE_ADJUSTMENT
             bme680.temperature() + TEMPERATURE_ADJUSTMENT,
#else
             bme680.temperature(),
#endif
             bme680.humidity(), bme680.pressure());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_air_quality_update(char *buf, size_t buf_len) {
    if(!bme680.is_present() && !pmsa003.is_present())
        return false;

    uint16_t pm1 = pmsa003.density_1_0();
    uint16_t pm25 = pmsa003.density_2_5();
    uint16_t pm10 = pmsa003.density_10_0();

    if(pm1 > 10000 && uptime.uptime() < 60 * 1000)
        pm1 = 0;

    if(pm25 > 10000 && uptime.uptime() < 60 * 1000)
        pm25 = 0;

    if(pm10 > 10000 && uptime.uptime() < 60 * 1000)
        pm10 = 0;

    snprintf(buf, buf_len, "{ \"tvoc\": %0.2f, \"pm1\": %d, \"pm25\": %d, \"pm10\": %d }", bme680.gas_resistance(), pm1,
             pm25, pm10);

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_aqi_update(char *buf, size_t buf_len) {
    if(!pmsa003.is_present())
        return false;

    uint16_t pm25 = pmsa003.density_2_5();
    uint16_t pm10 = pmsa003.density_10_0();

    if(pm25 > 10000 && uptime.uptime() < 60 * 1000)
        pm25 = 0;

    if(pm10 > 10000 && uptime.uptime() < 60 * 1000)
        pm10 = 0;

    unsigned aqi_value = aqi(pm25, pm10);

    snprintf(buf, buf_len, "{ \"aqi\": %u, \"condition\": \"%s\", \"condition_index\": %d }", aqi_value,
             aqi_condition_name(aqi_value), aqi_index(aqi_value));

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_co2_update(char *buf, size_t buf_len) {
    if(!scd4x.is_present())
        return false;

    snprintf(buf, buf_len,
             "{  \"co2\": %.0f, \"--temperature\": %3.1f, \"--humidity\": "
             "%4.0f, \"--source\": \"SCD4x\" }",
             scd4x.co2(), scd4x.temperature(), scd4x.humidity());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_h2co_update(char *buf, size_t buf_len) {
    int16_t h2co_value;
    int16_t temperature;
    int16_t humidity;

    uint16_t result = h2co_sensor.readMeasuredValues(h2co_value, humidity, temperature);

    if(result != 0)
        return false;

    snprintf(buf, buf_len,
             "{  \"h2co\": %0.1f, \"--temperature\": %0.1f, \"--humidity\": "
             "%0.0f, \"source\": \"Sensirion SFA30\" }",
             h2co_value / 5.0, temperature / 200.0, humidity / 100.0);

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

/* multichannel gas sensor
 */
#if 0
static bool furball_co_update(char* buf, size_t buf_len) {
  if(!multichannel_gas.is_present())
    return false;

  snprintf(buf,
	   buf_len,
	   "{  \"co\": %u }",
	   multichannel_gas.co());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}
#endif

static bool furball_no2_update(char *buf, size_t buf_len) {
    float temp = dfrobot_no2.readTempC();

    if(temp < -270)
        return false;

    snprintf(buf, buf_len,
             "{  \"no2\": %0.4f, \"--temperature\": %0.1f, \"--source\": "
             "\"DFRobot NO2\" }",
             dfrobot_no2.readGasConcentrationPPM(), temp);

    Serial.println(dfrobot_no2.readGasConcentrationPPM());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_o3_update(char *buf, size_t buf_len) {
    float temp = dfrobot_o3.readTempC();

    if(temp < -270)
        return false;

    snprintf(buf, buf_len,
             "{  \"o3\": %0.4f, \"--temperature\": %0.1f, \"--source\": "
             "\"DFRobot O3\" }",
             dfrobot_o3.readGasConcentrationPPM(), temp);

    Serial.println(dfrobot_o3.readGasConcentrationPPM());
    Serial.print("  sensor voltage ");
    Serial.println(dfrobot_o3.getSensorVoltage());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_co_update(char *buf, size_t buf_len) {
    float temp = dfrobot_co.readTempC();

    if(temp < -270)
        return false;

    snprintf(buf, buf_len,
             "{  \"co\": %0.4f, \"--temperature\": %0.1f, \"--source\": "
             "\"DFRobot CO\" }",
             dfrobot_co.readGasConcentrationPPM(), temp);

    Serial.println(dfrobot_co.readGasConcentrationPPM());
    Serial.print("  sensor voltage ");
    Serial.println(dfrobot_co.getSensorVoltage());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_h2s_update(char *buf, size_t buf_len) {
    float temp = dfrobot_h2s.readTempC();

    if(temp < -270)
        return false;

    snprintf(buf, buf_len,
             "{  \"h2s\": %0.4f, \"--temperature\": %0.1f, \"--source\": "
             "\"DFRobot H2S\" }",
             dfrobot_h2s.readGasConcentrationPPM(), temp);

    Serial.println(dfrobot_h2s.readGasConcentrationPPM());
    Serial.print("  sensor voltage ");
    Serial.println(dfrobot_h2s.getSensorVoltage());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_nh3_update(char *buf, size_t buf_len) {
    float temp = dfrobot_nh3.readTempC();

    if(temp < -270)
        return false;

    snprintf(buf, buf_len,
             "{  \"nh3\": %0.4f, \"--temperature\": %0.1f, \"--source\": "
             "\"DFRobot NH3\" }",
             dfrobot_nh3.readGasConcentrationPPM(), temp);

    Serial.println(dfrobot_nh3.readGasConcentrationPPM());
    Serial.print("  sensor voltage ");
    Serial.println(dfrobot_nh3.getSensorVoltage());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

#if 0
static bool furball_c2h5oh_update(char* buf, size_t buf_len) {
  if(!multichannel_gas.is_present())
    return false;

  snprintf(buf,
	   buf_len,
	   "{  \"coc2h5oh\": %u }",
	   multichannel_gas.c2h5oh());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}
#endif

static bool furball_voc_update(char *buf, size_t buf_len) {
    if(!multichannel_gas.is_present())
        return false;

    snprintf(buf, buf_len, "{  \"voc\": %u }", multichannel_gas.voc());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

/*
 * we do this once at startup, and not again unless our IP address changes
 */
static bool furball_system_update(char *buf, size_t buf_len) {
    static IPAddress oldIP = IPAddress(0, 0, 0, 0);
    static String mac_address = WiFi.macAddress();
    IPAddress local = WiFi.localIP();

    if(oldIP == local)
        return false;

    snprintf(buf, buf_len,
             "{ \"name\": \"%s\", \"platform\": \"%s\", \"build\": \"%s\", "
             "\"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" }",
             App.hostname().c_str(), "furball", App.build_info().c_str(), local[0], local[1], local[2], local[3],
             mac_address.c_str());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

static bool furball_diagnostic_update(char *buf, size_t buf_len) {
    snprintf(buf, buf_len,
             "{ \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": "
             "%d, \"wifi_failures\": %d }",
             ESP.getFreeHeap(), uptime.uptime() / 1000, WiFi.RSSI(), App.boot_count(), App.wifi_failures());

#ifdef VERBOSE
    Serial.println(buf);
#endif

    return true;
}

void furball_loop() {
    static unsigned long next_loop = 0;

    display_loop();

    if(next_loop > millis())
        return;

    Serial.println("do some handlin'");

    next_loop = millis() + UPDATE_DELAY;

    Serial.println("bme");
    bme680.handle();
    Serial.println("scd");
    scd4x.handle();
    Serial.println("pms");
    pmsa003.handle();
    Serial.println("ads");
    ads1115.handle();
    Serial.println("mgs");
    multichannel_gas.handle();

#define BUFFER_LENGTH 700
    char buffer[BUFFER_LENGTH + 1];
    char now_str[32];
    time_t now;

    time(&now);
    snprintf(now_str, 32, "%lu", now);

    if(furball_mics6814_update(buffer, BUFFER_LENGTH)) {
        homebus_publish_to(DDC_MICS6814_SENSOR, buffer);

        storage_writeln(now_str);
        storage_writeln(DDC_MICS6814_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_air_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.air-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.air-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_AIR_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_air_quality_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.air-quality-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.air-quality-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_AIR_QUALITY_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_aqi_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.aqi-pm25", buffer);
#else
        homebus_publish_to("org.homebus.experimental.aqi-pm25", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_AQI_SENSOR_PM25);
        storage_writeln(buffer);
    }

    if(furball_co2_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.co2-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.co2-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_CO2_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_h2co_update(buffer, BUFFER_LENGTH)) {
        homebus_publish_to("org.homebus.experimental.h2co-sensor", buffer);

        storage_writeln(now_str);
        storage_writeln(DDC_H2CO_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_no2_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.no2-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.no2-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_NO2_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_h2s_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.h2s-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.h2s-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_H2S_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_nh3_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.nh3-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.nh3-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_NH3_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_co_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.co-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.co-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_CO_SENSOR);
        storage_writeln(buffer);
    }

    if(furball_o3_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.o3-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.o3-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_O3_SENSOR);
        storage_writeln(buffer);
    }

#if 0
  if(furball_c2h5oh_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
    homebus_publish_to("org.homebus.experimental.c2h5oh-sensor", buffer);
#else
    homebus_publish_to("org.homebus.experimental.c2h5oh-sensor", buffer);
#endif

    storage_writeln(now_str);
    storage_writeln(DDC_C2H5OH_SENSOR);
    storage_writeln(buffer);
  }
#endif

    if(furball_voc_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.voc-sensor", buffer);
#else
        homebus_publish_to("org.homebus.experimental.voc-sensor", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_VOC_SENSOR);
        storage_writeln(buffer);
    }

#if 0
  if(furball_sound_update(buffer, BUFFER_LENGTH)) {
    homebus_publish_to("org.homebus.experimental.sound-sensor", buffer);
  }

  if(furball_presence_update(buffer, BUFFER_LENGTH)) {
    homebus_publish_to("org.homebus.experimental.presence-sensor", buffer);
  }
#endif

    if(furball_system_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.system", buffer);
#else
        homebus_publish_to("org.homebus.experimental.system", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_SYSTEM);
        storage_writeln(buffer);
    }

    if(furball_diagnostic_update(buffer, BUFFER_LENGTH)) {
#ifdef MQTT_OVERRIDE_TOPIC_AIR_SENSOR
        homebus_publish_to("org.homebus.experimental.diagnostic", buffer);
#else
        homebus_publish_to("org.homebus.experimental.diagnostic", buffer);
#endif

        storage_writeln(now_str);
        storage_writeln(DDC_DIAGNOSTIC);
        storage_writeln(buffer);
    }

    Serial.println("flush");
    storage_flush();
}

void homebus_mqtt_callback(const char *ddc, const char *payload) {
}

/*
 * this callback is used to stream sensor data for diagnostics
 */
#ifdef USE_DIAGNOSTICS
void furball_stream() {
    static uint8_t count = 0;

    if(count == 0) {
        Serial.println("TEMP PRES HUMD TVOC TEMP HUMD CO2   1.0  2.5 10.0");
    }

    if(++count == 10) {
        count = 0;
    }

    bme680.handle();
    scd4x.handle();
    pmsa003.handle();

    Serial.printf("%03.1f %4.0f %4.0f %4.0f %4.0f %03.1f %4.0f %4d %4d %4d\n", bme680.temperature(), bme680.pressure(),
                  bme680.humidity(), bme680.gas_resistance(), scd4x.temperature(), scd4x.humidity(), scd4x.co2(),
                  pmsa003.density_1_0(), pmsa003.density_2_5(), pmsa003.density_10_0());

    if(0) {
        Serial.println("[system]");
        Serial.printf("  Uptime %.2f seconds\n", uptime.uptime() / 1000.0);
        Serial.printf("  Free heap %u bytes\n", ESP.getFreeHeap());
        Serial.printf("  Wifi RSSI %d\n", WiFi.RSSI());
    }
}
#endif
