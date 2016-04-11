/**
 * Heated.h
 * Designed for module Hotend and Bed
 *
 * Copyright (C) 2016 Alberto Cotronei MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../../base.h"
#include "thermistortables.h"
#include <stdint.h>

#ifndef HEATED_H
  #define HEATED_H

  static void* heater_ttbl_map[13] = {(void*)temptable_1, (void*)temptable_2, (void*)temptable_3, (void*)temptable_4
    #if NUM_TEMPS_USERTHERMISTOR0 > 0
      ,(void*)temptable_5
    #else
      ,0
    #endif
    #if NUM_TEMPS_USERTHERMISTOR1 > 0
      ,(void*)temptable_6
    #else
      ,0
    #endif
    #if NUM_TEMPS_USERTHERMISTOR2 > 0
      ,((void*)temptable_7
    #else
      ,0
    #endif
    ,(void*)temptable_8
    ,(void*)temptable_9
    ,(void*)temptable_10
    ,(void*)temptable_11
    ,(void*)temptable_12
    ,(void*)temptable_13
  };

  static uint8_t heater_ttbllen_map[13] PROGMEM = {
    COUNT(temptable_1), COUNT(temptable_2), COUNT(temptable_3), COUNT(temptable_4),
    NUM_TEMPS_USERTHERMISTOR0, NUM_TEMPS_USERTHERMISTOR1, NUM_TEMPS_USERTHERMISTOR2,
    COUNT(temptable_8), COUNT(temptable_9), COUNT(temptable_10), COUNT(temptable_11),
    COUNT(temptable_12), COUNT(temptable_13)
  };

  class Heated {
    public:
      uint8_t id;
      int16_t sensorType;
      uint8_t heater_pin;
      uint8_t sensor_pin;
      int8_t fan_pin;
      int16_t currentTemperature_raw;
      int16_t targetTemperature_raw;
      float currentTemperatureC;
      float targetTemperatureC;
      float Kp;
      float Ki;
      float Kd;
      float Kc;
      float hotend_offset[3];
      float ad595_offset;
      float ad595_gain;
      int16_t auto_fan_temperature;
      int16_t minttempC;
      int16_t maxttempC;
      unsigned char soft_pwm;

      int getHeaterPower() { return this->soft_pwm; }
      bool isHeating() { return this->targetTemperatureC > this->currentTemperatureC; }
      bool isCooling() { return this->targetTemperatureC < this->currentTemperatureC; }
      float analog2temp() {
        int16_t type  = this->sensorType;
        int16_t raw   = this->currentTemperature_raw;
        switch(type) {
          case 0:
            return 25; break;
          case 1:
          case 2:
          case 3:
          case 4:
          case 5:
          case 6:
          case 7:
          case 8:
          case 9:
          case 10:
          case 11:
          case 12:
          {
            type--;
            if (heater_ttbl_map[type] != NULL) {
              float celsius = 0;
              uint8_t i;
              short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map[type]);

              for (i = 1; i < heater_ttbllen_map[type]; i++) {
                if (PGM_RD_W((*tt)[i][0]) > raw) {
                  celsius = PGM_RD_W((*tt)[i - 1][1]) +
                            (raw - PGM_RD_W((*tt)[i - 1][0])) *
                            (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                            (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
                  break;
                }
              }

              // Overflow: Set to last value in the table
              if (i == heater_ttbllen_map[type]) celsius = PGM_RD_W((*tt)[i - 1][1]);

              return TEMP_INT_TO_FLOAT(celsius);
            }
          }
          break;
          case 100:
            #ifdef __SAM3X8E__
              return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR) * this->ad595_gain) + this->ad595_offset;
            #else
              return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * this->ad595_gain) + this->ad595_offset;
            #endif
          break;
          case 101:
            return (float)raw / 4.0;
          break;
          case 998:
            return DUMMY_THERMISTOR_998_VALUE; break;
          case 999:
            return DUMMY_THERMISTOR_999_VALUE; break;
        }
      }
  };

  extern Heated Hotends[];
  extern Heated Beds[];

  // public functions
  void Heated_init();   // initialize the heating
  void manage_heater(); // it is critical that this is called periodically.
  void Update_heater();
  void disable_all_heaters();
  void updatePID();
  void PID_autotune(float temp, int hotend, int ncycles);
  void setExtruderAutoFanState(int pin, bool state);
  void checkExtruderAutoFans();
  void autotempShutdown();

  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);

  #if ENABLED(FILAMENT_SENSOR)
    // For converting raw Filament Width to milimeters
    float analog2widthFil();

    // For converting raw Filament Width to an extrusion ratio
    int widthFil_to_size_ratio();
  #endif

  #if HAS(POWER_CONSUMPTION_SENSOR)
    // For converting raw Power Consumption to watt
    float analog2voltage();
    float analog2current();
    float analog2power();
    float raw_analog2voltage();
    float analog2error(float current);
    float analog2efficiency(float watt);
  #endif

  #if ENABLED(BABYSTEPPING)
    extern volatile int babystepsTodo[3];
  #endif

  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    void start_watching_heater(int h = 0);
  #endif

  FORCE_INLINE void setTargetCelsius(const float& celsius, uint8_t heater) {
    Hotends[heater].targetTemperatureC = celsius;
    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      start_watching_heater(heater);
    #endif
  }

  #define HOTEND_ROUTINES(NR) \
    FORCE_INLINE void setTargetHotend##NR(const float c) { setTargetCelsius(c, NR); } \
  HOTEND_ROUTINES(0);
  #if HOTENDS > 1
    HOTEND_ROUTINES(1);
  #else
    #define setTargetHotend1(c) do{}while(0)
  #endif
  #if HOTENDS > 2
    HOTEND_ROUTINES(2);
  #else
    #define setTargetHotend2(c) do{}while(0)
  #endif
  #if HOTENDS > 3
    HOTEND_ROUTINES(3);
  #else
    #define setTargetHotend3(c) do{}while(0)
  #endif

#endif // HEATED_H
