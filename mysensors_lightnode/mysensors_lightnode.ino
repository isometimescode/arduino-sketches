/*
 * Check for changes in light and report as a door sensor to central gateway node.
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
 * Battery-detection methods modified from Adafruit, Inc
 *   GNU General Public License v2.1+
 *   https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Hardware/adc_vbat/adc_vbat.ino
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
 * OneTime-BH1750 library
 *   Apache License v2
 *   https://github.com/JVKran/OneTime-BH1750
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
// Adafruit Feather 328p + RFM69 wing config
//-------
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW

#define MY_RFM69_FREQUENCY RFM69_915MHZ
#define MY_RFM69_IRQ_PIN   3
#define MY_RFM69_CS_PIN    4
#define MY_RFM69_RST_PIN   2

//-------
// MySensors Node config
//-------
#define MY_NODE_ID         10
#define MY_PARENT_ID       0
#define MY_ENCRYPTION_SIMPLE_PASSWD "1234567812345678"

//-------
// Sensor logic config
//-------
#define SLEEP_TIME         8000 // Sleep time between reads (in milliseconds)
#define CHILD_ID           0
#define LUX_DIFF           10 // only report if this much change in lux happened

//-------
// Battery config
//-------
#define VBATPIN            A6
#define VBAT_MV_PER_LSB    (3.22265625F)   // 3.3V ADC range and 10-bit ADC resolution = 3300mV/1024
#define VBAT_DIVIDER_COMP  (2.0F)          // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

#define BATT_DIFF          2 // only report if this much change in battery pct

//-------
// Required libraries
//
// MySensors (https://www.arduino.cc/reference/en/libraries/mysensors/)
// OneTime-BH1750 (https://www.arduino.cc/reference/en/libraries/onetime-bh1750/)
//-------
#include <MySensors.h>
#include <BH1750.h>

BH1750 lightSensor;

MyMessage msg(CHILD_ID, V_TRIPPED);
uint16_t lastlux = 0;
uint8_t doorstate = 0;
uint8_t oldVbatPct = 0;

void setup()
{
  lightSensor.begin();
}

void presentation()  {
  sendSketchInfo("Delivery Box Sensor", "1.0");
  present(CHILD_ID, S_DOOR);
}

void loop()
{
  // 1. Check light sensor for changes
  //
  uint16_t currlux = lightSensor.getLightIntensity();

  if (currlux != lastlux) {
    uint8_t newstate = doorstate;
    int16_t diff = currlux - lastlux;

    if (diff >= LUX_DIFF) {
      // new lux is brighter, i.e. box opened
      newstate = 1;
    } else if (abs(diff) >= LUX_DIFF) {
      // new lux is less than before (i.e. box closed)
      newstate = 0;
    }

    Printf("---LUX: old(%u) vs new(%u)\n", lastlux, currlux);
    Printf("---DOOR: old(%u) vs new(%u)\n", doorstate, newstate);

    if (newstate != doorstate) {
      send(msg.set(newstate));
    }

    lastlux = currlux;
    doorstate = newstate;
  }

  // 2. Check battery percentage for changes
  //
  uint8_t vbatPct = getBattPercent();
  Printf("---VBAT: %u%%\n", vbatPct);

  if (abs(oldVbatPct - vbatPct) >= BATT_DIFF) {
    sendBatteryLevel(vbatPct);
    oldVbatPct = vbatPct;
  }

  // 3. All done, wait until next interval
  sleep(SLEEP_TIME);
}

uint8_t mvToPercent(float mvolts) {
  // LIPO discharge is not linear, so this handles converting mvolts
  // to a percent charge grouped in several ranges from dead to fully charged

  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  mvolts -= 3600;
  uint8_t pct = 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
  return (pct >= 100) ? 100 : pct;
}

uint8_t getBattPercent() {
  // https://learn.adafruit.com/adafruit-feather-328p-atmega328-atmega328p/power-management#measuring-battery-2981135-7
  //
  // two 100k resistors and ADC ref of 3.3V
  // ( ((100e3+100e3)/100e3)*3.3) / 1024 ) = Voltage
  //
  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3300mV and resolution is 10-bit (0..1023)
  float raw = analogRead(VBATPIN);
  float vbatmv = raw * REAL_VBAT_MV_PER_LSB;

  Printf("---VBAT: %umv\n", vbatmv);

  return round(mvToPercent(vbatmv));
}
