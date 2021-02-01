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
#define LOCAL_DEBUG
#ifdef LOCAL_DEBUG
  #define Printf(...) {char printBuf[24]; sprintf(printBuf, __VA_ARGS__) ; Serial.print(printBuf);}
#else
  #define Printf(...);
#endif


//-------
// Arduino Pro Mini + RFM69HCW radio
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
#define SLEEP_TIME       300000 // Sleep time between reads (in milliseconds)
#define TEMP_CHILD_ID         0
#define HUM_CHILD_ID          1
#define BARO_CHILD_ID         2

#define BME_ADDR           0x76
#define USE_FAHRENHEIT        1
#define DIFF_THRESHOLD      0.1 // only report if new value is different

//-------
// Battery config
//-------
// TODO

//-------
// Required libraries
//
// MySensors (https://www.arduino.cc/reference/en/libraries/mysensors/)
// SparkFun BME280 (https://www.arduino.cc/reference/en/libraries/sparkfun-bme280/)
//-------
#include <MySensors.h>
#include <SparkFunBME280.h>

BME280 bme;
BME280_SensorMeasurements oldM;
BME280_SensorMeasurements newM;

MyMessage temperatureMsg(TEMP_CHILD_ID, V_TEMP);
MyMessage humidityMsg(HUM_CHILD_ID, V_HUM);
MyMessage pressureMsg(BARO_CHILD_ID, V_PRESSURE);

uint8_t oldVbatPct = 0;

void setup()
{
  unsigned status;
  bme.setI2CAddress(BME_ADDR);
  if(bme.beginI2C() == false) {
    Printf("sensor not found\n");
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

  while(bme.isMeasuring() == false) ; // sensor is waking up
  while(bme.isMeasuring() == true) ;  // actually taking a measurement

   // retrieve all register data at the same time for efficiency
   // see Bosch datasheet section 4 for burst reading details
  bme.readAllMeasurements(&newM, USE_FAHRENHEIT);

  oldM.temperature = sendSensorValue(oldM.temperature, newM.temperature, temperatureMsg);
  oldM.humidity = sendSensorValue(oldM.humidity, newM.humidity, humidityMsg);
  oldM.pressure = sendSensorValue(oldM.pressure, newM.pressure, pressureMsg);

  // 2. Check battery percentage for changes
  //
  // checkBattery();

  // 3. All done, wait until next interval
  //
  sleep(SLEEP_TIME);
}

float sendSensorValue(float oldValue, float newValue, MyMessage &msg) {
#ifdef LOCAL_DEBUG
  Serial.print(oldValue, 1); Serial.print(" vs "); Serial.println(newValue, 1);
#endif

  if (abs(newValue - oldValue) < DIFF_THRESHOLD) {
    Printf("threshold not met\n");
  } else {
    send(msg.set(newValue, 1));
  }
  return newValue;
}

void checkBattery() {
  uint8_t vbatPct = 0; // TODO

  Printf("---VBAT: %u%%\n", vbatPct);

  if (oldVbatPct != vbatPct) {
    sendBatteryLevel(vbatPct);
    oldVbatPct = vbatPct;
  }
}
