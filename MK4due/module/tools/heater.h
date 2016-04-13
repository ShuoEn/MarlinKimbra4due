/**
 * Heater.h
 * Designed for Heater Tools Hotend, Bed and Chamber
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

#ifndef HEATER_H
  #define HEATER_H

  static void* heater_ttbl_map[14] = { 0,
    (void*)temptable_1, (void*)temptable_2, (void*)temptable_3, (void*)temptable_4,
    (void*)temptable_5, (void*)temptable_6, (void*)temptable_7, (void*)temptable_8,
    (void*)temptable_9, (void*)temptable_10
    #if NUM_TEMPS_USERTHERMISTOR0 > 0
      ,(void*)temptable_11
    #else
      ,0
    #endif
    #if NUM_TEMPS_USERTHERMISTOR1 > 0
      ,(void*)temptable_12
    #else
      ,0
    #endif
    #if NUM_TEMPS_USERTHERMISTOR2 > 0
      ,(void*)temptable_13
    #else
      ,0
    #endif
  };

  static uint8_t heater_ttbllen_map[14] PROGMEM = { 0,
    COUNT(temptable_1), COUNT(temptable_2), COUNT(temptable_3), COUNT(temptable_4),
    COUNT(temptable_5), COUNT(temptable_6), COUNT(temptable_7), COUNT(temptable_8),
    COUNT(temptable_9), COUNT(temptable_10),
    NUM_TEMPS_USERTHERMISTOR0, NUM_TEMPS_USERTHERMISTOR1, NUM_TEMPS_USERTHERMISTOR2
  };

  class Heater {
    public:
      uint8_t id;
      uint8_t sensorType;
      int8_t sensor_pin;
      int8_t heater_pin;
      int8_t fan_pin;
      int16_t currentTemperature_raw;
      int16_t targetTemperature_raw;
      int16_t minttempC;
      int16_t maxttempC;
      uint8_t pid_max;
      float currentTemperatureC;
      float targetTemperatureC;
      bool use_pid;
      float Kp;
      float Ki;
      float Kd;
      float Kc;
      float hotend_offset[3];
      float ad595_offset;
      float ad595_gain;
      bool has_fan;
      int16_t auto_fan_temperature;
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

              return celsius;
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
        }
      }
    private:
  };

  extern Heater Heaters[];
  //uint8_t Hotends(HEATER_HOTENDS);
  //uint8_t Beds(HEATER_BEDS);

  // public functions
  void Heated_init();   // initialize the heating
  void manage_heater(); // it is critical that this is called periodically.
  void Update_heater();
  void disable_all_heaters();
  void updatePID();
  void PID_autotune(float temp, int heater, int ncycles);
  void setHeaterAutoFanState(int pin, bool state);
  void checkHeaterAutoFans();
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
    Heaters[heater].targetTemperatureC = celsius;
    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      start_watching_heater(heater);
    #endif
  }
#endif // HEATED_H
