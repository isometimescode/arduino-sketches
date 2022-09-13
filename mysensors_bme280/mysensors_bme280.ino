/*
 * Read temperature, relative humidity, and barometric pressure from a BME280 sensor and report the data.
 *
 * Copyright: (c) 2021, Toni Wells (@isometimescode)
 * GNU General Public License v3.0+ (see https://www.gnu.org/licenses/gpl-3.0.txt)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 *******************************
 * Referenced libraries and examples:
 *
 * MySensors Arduino library
 *   GNU General Public License v2+
 *   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 *   Copyright (C) 2013-2019 Sensnology AB
 *   Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 *   Documentation: http://www.mysensors.org
 *   Support Forum: http://forum.mysensors.org
 *
 * SparkFun BME280 library
 *   MIT License
 *   Copyright (c) 2015 SparkFun Electronics
 *   https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
 *
 *******************************
 */

// #define MY_DEBUG
// #define MY_DEBUG_VERBOSE_RFM69
// #define LOCAL_DEBUG

//-------
// Arduino Pro Mini + RFM69HW radio
//-------
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW

#define MY_RFM69_FREQUENCY RFM69_915MHZ

//-------
// MySensors Node config
//-------
#define MY_NODE_ID         11
#define MY_PARENT_ID       0
#define MY_ENCRYPTION_SIMPLE_PASSWD "1234567812345678"

//-------
// Sensor logic config
//-------
#define SLEEP_TIME       300000 // Sleep time between reads (in milliseconds) = 5 min
#define TEMP_CHILD_ID         0
#define TEMP_DIFF           0.1 // only report if new value is different

#define HUM_CHILD_ID          1
#define HUM_DIFF            0.5 // only report if new value is different

#define BARO_CHILD_ID         2
#define BARO_DIFF          0.01 // only report if new value is different
#define ONE_INHG        3386.39 // 1 inch Hg = 3386.39 Pascal

#define BME_ADDR           0x76
#define USE_FAHRENHEIT        1

//-------
// Battery config
//-------
#define FULL_BATTERY 3 // 3V for 2xAA alkaline

//-------
// Required external libraries
//
// MySensors v2.4.0-alpha (development) (https://github.com/mysensors/MySensors)
// SparkFun BME280 v2.0.9 (https://www.arduino.cc/reference/en/libraries/sparkfun-bme280/)
//-------
#include <Wire.h>
#include <MySensors.h>
#include <SparkFunBME280.h>

// redefine to change pressure to inHg
struct BME280_LastMeasurements
{
  public:
	float temperature;
	float pressureInHg;
	float humidity;
};

BME280 bme;
BME280_LastMeasurements oldM;
BME280_SensorMeasurements newM;

MyMessage temperatureMsg(TEMP_CHILD_ID, V_TEMP);
MyMessage humidityMsg(HUM_CHILD_ID, V_HUM);
MyMessage pressureMsg(BARO_CHILD_ID, V_PRESSURE);

uint8_t oldBatteryPcnt = 0;

void setup()
{
  unsigned status;
  Wire.begin();
  bme.setI2CAddress(BME_ADDR);
  if(bme.beginI2C() == false) {
#ifdef LOCAL_DEBUG
    Serial.println("No sensor found; checking wiring");
#endif
    while(1); // can't do anything without a sensor
  }

  // same as Bosch datasheet section 3.5.1:
  // low power consumption but increased noise
  bme.setMode(MODE_SLEEP);
  bme.setFilter(0); // off
  bme.setTempOverSample(1);
  bme.setPressureOverSample(1);
  bme.setHumidityOverSample(1);
}

void presentation()  {
  sendSketchInfo("Outdoor Weather Sensor", "1.0");
  present(TEMP_CHILD_ID, S_TEMP);
  present(HUM_CHILD_ID, S_HUM);
  present(BARO_CHILD_ID, S_BARO);
}

void loop()
{
  // 1. Grab sensor values
  //
  bme.setMode(MODE_FORCED);

#ifdef LOCAL_DEBUG
  Serial.println("Begin measuring");
#endif
  while(bme.isMeasuring() == false) ; // sensor is waking up
  while(bme.isMeasuring() == true) ;  // actually taking a measurement
#ifdef LOCAL_DEBUG
  Serial.println("End measuring");
#endif

   // retrieve all register data at the same time for efficiency
   // see Bosch datasheet section 4 for burst reading details
  bme.readAllMeasurements(&newM, USE_FAHRENHEIT);

  oldM.temperature = sendSensorValue(
    oldM.temperature, newM.temperature, TEMP_DIFF, temperatureMsg);

  oldM.humidity = sendSensorValue(
    oldM.humidity, newM.humidity, HUM_DIFF, humidityMsg);

  oldM.pressureInHg = sendSensorValue(
    oldM.pressureInHg, pressureToInHg(newM.pressure), BARO_DIFF, pressureMsg);

  // 2. Check battery percentage for changes
  //
  checkBattery();

  // 3. All done, wait until next interval
  //
#ifdef LOCAL_DEBUG
  Serial.print(SLEEP_TIME/1000/60);
  Serial.println(" minute sleep");
#endif
  sleep(SLEEP_TIME);
#ifdef LOCAL_DEBUG
  Serial.println("Awake!");
#endif
}

float sendSensorValue(float oldValue, float newValue, float diffThreshold, MyMessage &msg) {
#ifdef LOCAL_DEBUG
  Serial.print(oldValue, 1); Serial.print(" vs "); Serial.println(newValue, 1);
#endif

  if (abs(newValue - oldValue) < diffThreshold) {
#ifdef LOCAL_DEBUG
    Serial.print(abs(newValue - oldValue));
    Serial.print(" < ");
    Serial.print(diffThreshold);
    Serial.println("; skip sending");
#endif
    return oldValue;
  }

#ifdef LOCAL_DEBUG
    Serial.print(abs(newValue - oldValue));
    Serial.print(" > ");
    Serial.print(diffThreshold);
    Serial.print("; sending: ");
    Serial.println(newValue, 1);
#endif

  send(msg.set(newValue, 2));
  return newValue;
}

float pressureToInHg(float value) {
  // default value is Pa
  // silly US uses inHg
  return value / ONE_INHG;
}

// https://www.mysensors.org/build/battery#measuring-and-reporting-battery-level
void checkBattery() {
	// get the battery voltage
	long batteryMillivolts = hwCPUVoltage();
	uint8_t batteryPcnt = batteryMillivolts / FULL_BATTERY / 1000.0 * 100 + 0.5;
  batteryPcnt = min(batteryPcnt, 100);

#ifdef LOCAL_DEBUG
	Serial.print("Battery voltage: ");
	Serial.print(batteryMillivolts / 1000.0);
	Serial.println("V");
	Serial.print("Battery percent: ");
	Serial.print(batteryPcnt);
	Serial.println(" %");
#endif

	if (oldBatteryPcnt != batteryPcnt) {
		sendBatteryLevel(batteryPcnt);
		oldBatteryPcnt = batteryPcnt;
	}
}
