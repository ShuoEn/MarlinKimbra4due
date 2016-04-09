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
#include <stdint.h>

#ifndef HEATED_H
  #define HEATED_H

  #define OVERSAMPLENR 16
  #define PID_dT (((OVERSAMPLENR + 2) * 14.0)/ TEMP_FREQUENCY)

  #define NUMTEMPS_1 28 // Epcos B57560G0107F000
  const short temptable_1[NUMTEMPS_1][2] PROGMEM = {
    {0,4000},{92,2400},{105,2320},{121,2240},{140,2160},{162,2080},{189,2000},{222,1920},{261,1840},{308,1760},
    {365,1680},{434,1600},{519,1520},{621,1440},{744,1360},{891,1280},{1067,1200},{1272,1120},
    {1771,960},{2357,800},{2943,640},{3429,480},{3760,320},{3869,240},{3912,200},{3948,160},{4077,-160},{4094,-440}
  };
  #define NUMTEMPS_2 21
  const short temptable_2[NUMTEMPS_2][2] PROGMEM = {
    {1*4, 848*8},{54*4, 275*8}, {107*4, 228*8}, {160*4, 202*8},{213*4, 185*8}, {266*4, 171*8}, {319*4, 160*8}, {372*4, 150*8},
    {425*4, 141*8}, {478*4, 133*8},{531*4, 125*8},{584*4, 118*8},{637*4, 110*8},{690*4, 103*8},{743*4, 95*8},{796*4, 86*8},
    {849*4, 77*8},{902*4, 65*8},{955*4, 49*8},{1008*4, 17*8},{1020*4, 0*8}
  };
  #define NUMTEMPS_3 28
  const short temptable_3[NUMTEMPS_3][2] PROGMEM = {
    {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},
    {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},
    {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}
  };
  #define NUMTEMPS_4 20
  const short temptable_4[NUMTEMPS_4][2] PROGMEM = {
    {1*4, 430*8},{54*4, 137*8},{107*4, 107*8},{160*4, 91*8},{213*4, 80*8},{266*4, 71*8},{319*4, 64*8},{372*4, 57*8},{425*4, 51*8},
    {478*4, 46*8},{531*4, 41*8},{584*4, 35*8},{637*4, 30*8},{690*4, 25*8},{743*4, 20*8},{796*4, 14*8},{849*4, 7*8},{902*4, 0*8},
    {955*4, -11*8},{1008*4, -35*8}
  };
  #define NUMTEMPS_8 34 // ATC 104GT
  const short temptable_8[NUMTEMPS_8][2] PROGMEM = {
    {0,8000},{69,2400},{79,2320},{92,2240},{107,2160},{125,2080},{146,2000},{172,1920},{204,1840},{222,1760},{291,1680},{350,1600},
    {422,1520},{511,1440},{621,1360},{755,1280},{918,1200},{1114,1120},{1344,1040},{1608,960},{1902,880},{2216,800},{2539,720},
    {2851,640},{3137,560},{3385,480},{3588,400},{3746,320},{3863,240},{3945,160},{4002,80},{4038,0},{4061,-80},{4075,-160}
  };
  #define NUMTEMPS_9 67 // 100k Honeywell 135-104LAG-J01
  const short temptable_9[NUMTEMPS_9][2] PROGMEM = {
    {1*4, 941*8},{19*4, 362*8},{37*4, 299*8}, //top rating 300C
    {55*4, 266*8},{73*4, 245*8},{91*4, 229*8},{109*4, 216*8},{127*4, 206*8},{145*4, 197*8},{163*4, 190*8},{181*4, 183*8},{199*4, 177*8},
    {217*4, 171*8},{235*4, 166*8},{253*4, 162*8},{271*4, 157*8},{289*4, 153*8},{307*4, 149*8},{325*4, 146*8},{343*4, 142*8},{361*4, 139*8},
    {379*4, 135*8},{397*4, 132*8},{415*4, 129*8},{433*4, 126*8},{451*4, 123*8},{469*4, 121*8},{487*4, 118*8},{505*4, 115*8},{523*4, 112*8},
    {541*4, 110*8},{559*4, 107*8},{577*4, 105*8},{595*4, 102*8},{613*4, 99*8},{631*4, 97*8},{649*4, 94*8},{667*4, 92*8},{685*4, 89*8},
    {703*4, 86*8},{721*4, 84*8},{739*4, 81*8},{757*4, 78*8},{775*4, 75*8},{793*4, 72*8},{811*4, 69*8},{829*4, 66*8},{847*4, 62*8},
    {865*4, 59*8},{883*4, 55*8},{901*4, 51*8},{919*4, 46*8},{937*4, 41*8},
    {955*4, 35*8},{973*4, 27*8},{991*4, 17*8},{1009*4, 1*8},{1023*4, 0}  //to allow internal 0 degrees C
  };
  #define NUMTEMPS_10 20 // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
  const short temptable_10[NUMTEMPS_10][2] PROGMEM = {
    {1*4, 704*8},{54*4, 216*8},{107*4, 175*8},{160*4, 152*8},{213*4, 137*8},{266*4, 125*8},{319*4, 115*8},{372*4, 106*8},{425*4, 99*8},
    {478*4, 91*8},{531*4, 85*8},{584*4, 78*8},{637*4, 71*8},{690*4, 65*8},{743*4, 58*8},{796*4, 50*8},{849*4, 42*8},{902*4, 31*8},
    {955*4, 17*8},{1008*4, 0}
  };
  #define NUMTEMPS_11 31 // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
  const short temptable_11[NUMTEMPS_11][2] PROGMEM = {
    {1*4, 936*8},{36*4, 300*8},{71*4, 246*8},{106*4, 218*8},{141*4, 199*8},{176*4, 185*8},{211*4, 173*8},{246*4, 163*8},{281*4, 155*8},
    {316*4, 147*8},{351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},
    {631*4, 97*8},{666*4, 92*8},{701*4, 87*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},
    {946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0}
  };
  #define NUMTEMPS_12 31 // 100k RS thermistor 198-961 (4.7k pullup)
  const short temptable_12[NUMTEMPS_12][2] PROGMEM = {
    {1*4, 929*8},{36*4, 299*8},{71*4, 246*8},{106*4, 217*8},{141*4, 198*8},{176*4, 184*8},{211*4, 173*8},{246*4, 163*8},{281*4, 154*8},{316*4, 147*8},
    {351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},{631*4, 97*8},{666*4, 91*8},
    {701*4, 86*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},{946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0*8}
  };
  #define NUMTEMPS_13 19
  const short temptable_13[NUMTEMPS_13][2] PROGMEM = {
    {0,0},{908,8},{942,10*8},{982,20*8},{1015,8*30},{1048,8*40},{1080,8*50},{1113,8*60},{1146,8*70},{1178,8*80},{1211,8*90},{1276,8*110},{1318,8*120},
    {1670,8*230},{2455,8*500},{3445,8*900},{3666,8*1000},{3871,8*1100},{4095,8*2000}
  };
  #if NUM_TEMPS_USERTHERMISTOR0 > 0
    const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0;
  #endif
  #if NUM_TEMPS_USERTHERMISTOR1 > 0
    const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1;
  #endif
  #if NUM_TEMPS_USERTHERMISTOR2 > 0
    const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2;
  #endif

  const short * const temptables[13] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0]
    #if NUM_TEMPS_USERTHERMISTOR0 > 0
      ,(short int *)&temptable_5[0][0]
    #else
      ,0
    #endif
    #if NUM_TEMPS_USERTHERMISTOR1 > 0
      ,(short int *)&temptable_6[0][0]
    #else
      ,0
    #endif
    #if NUM_TEMPS_USERTHERMISTOR2 > 0
      ,(short int *)&temptable_7[0][0]
    #else
      ,0
    #endif
    ,(short int *)&temptable_8[0][0]
    ,(short int *)&temptable_9[0][0]
    ,(short int *)&temptable_10[0][0]
    ,(short int *)&temptable_11[0][0]
    ,(short int *)&temptable_12[0][0]
    ,(short int *)&temptable_13[0][0]
  };

  const uint8_t temptables_num[13] PROGMEM = {
    NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,0,0,0,NUMTEMPS_8,
    NUMTEMPS_9,NUMTEMPS_10,NUMTEMPS_11,NUMTEMPS_12,NUMTEMPS_13
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
            return DUMMY_THERMISTOR_998_VALUE; break;
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
            uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
            uint8_t i = 2;
            const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
            int16_t oldraw = pgm_read_word(&temptable[0]);
            int16_t oldtemp = pgm_read_word(&temptable[1]);
            int16_t newraw, newtemp = 0;
            while(i < num) {
              newraw = pgm_read_word(&temptable[i++]);
              newtemp = pgm_read_word(&temptable[i++]);
              if (newraw > raw)
                return TEMP_INT_TO_FLOAT(oldtemp + (float)(raw - oldraw)*(float)(newtemp - oldtemp) / (newraw - oldraw));
              oldtemp = newtemp;
              oldraw = newraw;
            }
            // Overflow: Set to last value in the table
            return TEMP_INT_TO_FLOAT(newtemp);
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
