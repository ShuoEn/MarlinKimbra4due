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
#include "heated.h"

//===========================================================================
//================================== macros =================================
//===========================================================================

#if ENABLED(K1) // Defined in Configuration_base.h in the PID settings
  #define K2 (1.0 - K1)
#endif

//===========================================================================
//================================== Object =================================
//===========================================================================
#if HOTENDS == 0
  Heated Hotends[1];
#else
  Heated Hotends[HOTENDS] = {
  #if HOTENDS > 0
    {
      0, TEMP_SENSOR_0, HOTEND_0_PIN, TEMP_0_PIN, HOTEND_0_AUTO_FAN_PIN,
      0, 0, 0, 0, 0, 0, 0, 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, HOTEND_0_AUTO_FAN_TEMPERATURE,
      HOTEND_0_MINTEMP, HOTEND_0_MAXTEMP, 0
    }
  #endif
  #if HOTENDS > 1
    ,{
      1, TEMP_SENSOR_1, HOTEND_1_PIN, TEMP_1_PIN, HOTEND_1_AUTO_FAN_PIN,
      0, 0, 0, 0, 0, 0, 0, 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, HOTEND_1_AUTO_FAN_TEMPERATURE,
      HOTEND_1_MINTEMP, HOTEND_0_MAXTEMP, 0
    }
  #endif
  #if HOTENDS > 2
    ,{
      2, TEMP_SENSOR_2, HOTEND_2_PIN, TEMP_2_PIN, HOTEND_2_AUTO_FAN_PIN,
      0, 0, 0, 0, 0, 0, 0, 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, HOTEND_2_AUTO_FAN_TEMPERATURE,
      HOTEND_2_MINTEMP, HOTEND_0_MAXTEMP, 0
    }
  #endif
  #if HOTENDS > 3
    ,{
      3, TEMP_SENSOR_3, HOTEND_3_PIN, TEMP_3_PIN, HOTEND_3_AUTO_FAN_PIN,
      0, 0, 0, 0, 0, 0, 0, 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, HOTEND_3_AUTO_FAN_TEMPERATURE,
      HOTEND_3_MINTEMP, HOTEND_0_MAXTEMP, 0
    }
  #endif
  };
#endif

#if BEDS == 0
  Heated Beds[1];
#else
  Heated Beds[BEDS] {
    #if BEDS > 0
    {
      0, TEMP_SENSOR_BED_0, BED_0_PIN, TEMP_BED_0_PIN, -1,
      0, 0, 0, 0, DEFAULT_bedKp, ((DEFAULT_bedKi) * (PID_dT)), ((DEFAULT_bedKd) / (PID_dT)), 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, 0,
      BED_0_MINTEMP, BED_0_MAXTEMP, 0
    }
    #endif
    #if BEDS > 1
    {
      1, TEMP_SENSOR_BED_1, BED_1_PIN, TEMP_BED_1_PIN, -1,
      0, 0, 0, 0, DEFAULT_bedKp, ((DEFAULT_bedKi) * (PID_dT)), ((DEFAULT_bedKd) / (PID_dT)), 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, 0,
      BED_1_MINTEMP, BED_1_MAXTEMP, 0
    }
    #endif
    #if BEDS > 2
    {
      2, TEMP_SENSOR_BED_2, BED_2_PIN, TEMP_BED_2_PIN, -1,
      0, 0, 0, 0, DEFAULT_bedKp, ((DEFAULT_bedKi) * (PID_dT)), ((DEFAULT_bedKd) / (PID_dT)), 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, 0,
      BED_2_MINTEMP, BED_2_MAXTEMP, 0
    }
    #endif
    #if BEDS > 3
    {
      3, TEMP_SENSOR_BED_3, BED_3_PIN, TEMP_BED_3_PIN, -1,
      0, 0, 0, 0, DEFAULT_bedKp, ((DEFAULT_bedKi) * (PID_dT)), ((DEFAULT_bedKd) / (PID_dT)), 100,
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN, 0,
      BED_3_MINTEMP, BED_3_MAXTEMP, 0
    }
    #endif
  };
#endif

//===========================================================================
//============================= public variables ============================
//===========================================================================

#if ENABLED(FAN_SOFT_PWM)
  unsigned char fanSpeedSoftPwm = 0;
  #if HAS(AUTO_FAN)
    unsigned char fanSpeedSoftPwm_auto = EXTRUDER_AUTO_FAN_MIN_SPEED;
  #endif
  #if HAS(CONTROLLERFAN)
    unsigned char fanSpeedSoftPwm_controller = CONTROLLERFAN_MIN_SPEED;
  #endif
#endif

#if ENABLED(BABYSTEPPING)
  volatile int babystepsTodo[3] = { 0 };
#endif

#if ENABLED(FILAMENT_SENSOR)
  int current_raw_filwidth = 0;  //Holds measured filament diameter - one extruder only
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED)
  enum TRState { TRReset, TRInactive, TRFirstHeating, TRStable, TRRunaway };
  void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float targetTemperatureC, int heater_id, int period_seconds, int hysteresis_degc);
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    static TRState thermal_runaway_state_machine[4] = { TRReset, TRReset, TRReset, TRReset };
    static millis_t thermal_runaway_timer[4]; // = {0,0,0,0};
  #endif
  #if ENABLED(THERMAL_PROTECTION_BED) && TEMP_SENSOR_BED != 0
    static TRState thermal_runaway_bed_state_machine = TRReset;
    static millis_t thermal_runaway_bed_timer;
  #endif
#endif
#if HAS(POWER_CONSUMPTION_SENSOR)
  int current_raw_powconsumption = 0;  //Holds measured power consumption
  static unsigned long raw_powconsumption_value = 0;
#endif

//===========================================================================
//============================ private variables ============================
//===========================================================================
static volatile bool temp_meas_ready = false;

//static cannot be external:
static float temp_iState[HOTENDS] = { 0 };
static float temp_dState[HOTENDS] = { 0 };
static float pTerm[HOTENDS];
static float iTerm[HOTENDS];
static float dTerm[HOTENDS];
#if ENABLED(PID_ADD_EXTRUSION_RATE)
  static float cTerm[HOTENDS];
  static long last_position[EXTRUDERS];
  static long lpq[LPQ_MAX_LEN];
  static int lpq_ptr = 0;
#endif
//int output;
static float pid_error[HOTENDS];
static float temp_iState_min[HOTENDS];
static float temp_iState_max[HOTENDS];
static bool pid_reset[HOTENDS];

#if ENABLED(PIDTEMPBED)
  //static cannot be external:
  static float temp_iState_bed = { 0 };
  static float temp_dState_bed = { 0 };
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
  static millis_t  next_bed_check_ms;
#endif //PIDTEMPBED

#if ENABLED(FAN_SOFT_PWM)
  static unsigned char soft_pwm_fan;
  #if HAS(AUTO_FAN)
    static unsigned char soft_pwm_fan_auto;
  #endif
  #if HAS(CONTROLLERFAN)
    static unsigned char soft_pwm_fan_controller = 0;
  #endif
#endif
#if HAS(AUTO_FAN)
  static millis_t next_auto_fan_check_ms;
#endif

static void updateTemperaturesFromRawValues();

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  int watch_target_temp[HOTENDS] = { 0 };
  millis_t watch_heater_next_ms[HOTENDS] = { 0 };
#endif

#if DISABLED(SOFT_PWM_SCALE)
  #define SOFT_PWM_SCALE 0
#endif

#if ENABLED(FILAMENT_SENSOR)
  static int meas_shift_index;  // used to point to a delayed sample in buffer for filament width sensor
#endif

#if ENABLED(HOTEND_0_USES_MAX6675)
  static int16_t read_max6675();
#endif

//===========================================================================
//================================ Functions ================================
//===========================================================================

void autotempShutdown() {
  #if ENABLED(AUTOTEMP)
    if (autotemp_enabled) {
      autotemp_enabled = false;
      if (Hotends[active_extruder].targetTemperatureC > autotemp_min)
        setTargetCelsius(0, active_extruder);
    }
  #endif
}

void PID_autotune(float temp, int hotend, int ncycles) {
  float input = 0.0;
  int cycles = 0;
  bool heating = true;

  millis_t temp_ms = millis(), t1 = temp_ms, t2 = temp_ms;
  long t_high = 0, t_low = 0;

  long bias = 0, d = 0;
  float Ku = 0, Tu = 0;
  float Kp_temp = 0, Ki_temp = 0, Kd_temp = 0;
  float max = 0, min = 10000;

  #if HAS(AUTO_FAN)
    millis_t next_auto_fan_check_ms = temp_ms + 2500;
  #endif

  if (hotend >= HOTENDS
    #if HASNT(TEMP_BED)
       || hotend < 0
    #endif
  ) {
    ECHO_LM(ER, SERIAL_PID_BAD_EXTRUDER_NUM);
    return;
  }

  ECHO_LM(DB, SERIAL_PID_AUTOTUNE_START);
  if (hotend < 0) {
    ECHO_SM(DB, "BED");
  }
  else {
      ECHO_SMV(DB, "Hotend: ", hotend);
  }
  ECHO_MV(" Temp: ", temp);
  ECHO_EMV(" Cycles: ", ncycles);

  disable_all_heaters(); // switch off all heaters.

  if (hotend < 0)
    Beds[0].soft_pwm = bias = d = MAX_BED_POWER / 2;
  else
    Hotends[hotend].soft_pwm = bias = d = PID_MAX / 2;

  // PID Tuning loop
  for (;;) {

    millis_t ms = millis();

    if (temp_meas_ready) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = (hotend < 0) ? Beds[0].currentTemperatureC : Hotends[hotend].currentTemperatureC;

      max = max(max, input);
      min = min(min, input);

      #if HAS(AUTO_FAN)
        if (ms > next_auto_fan_check_ms) {
          checkExtruderAutoFans();
          next_auto_fan_check_ms = ms + 2500;
        }
      #endif

      if (heating && input > temp) {
        if (ms > t2 + 5000) {
          heating = false;
          if (hotend < 0)
            Beds[0].soft_pwm = (bias - d) >> 1;
          else
            Hotends[hotend].soft_pwm = (bias - d) >> 1;
          t1 = ms;
          t_high = t1 - t2;
          max = temp;
        }
      }

      if (!heating && input < temp) {
        if (ms > t1 + 5000) {
          heating = true;
          t2 = ms;
          t_low = t2 - t1;
          if (cycles > 0) {
            long max_pow = hotend < 0 ? MAX_BED_POWER : PID_MAX;
            bias += (d * (t_high - t_low)) / (t_low + t_high);
            bias = constrain(bias, 20, max_pow - 20);
            d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

            ECHO_MV(SERIAL_BIAS, bias);
            ECHO_MV(SERIAL_D, d);
            ECHO_MV(SERIAL_T_MIN, min);
            ECHO_MV(SERIAL_T_MAX, max);
            if (cycles > 2) {
              Ku = (4.0 * d) / (3.14159265 * (max - min) / 2.0);
              Tu = ((float)(t_low + t_high) / 1000.0);
              ECHO_MV(SERIAL_KU, Ku);
              ECHO_EMV(SERIAL_TU, Tu);
              Kp_temp = 0.6 * Ku;
              Ki_temp = 2 * Kp_temp / Tu;
              Kd_temp = Kp_temp * Tu / 8;
              
              ECHO_EM(SERIAL_CLASSIC_PID);
              ECHO_MV(SERIAL_KP, Kp_temp);
              ECHO_MV(SERIAL_KI, Ki_temp);
              ECHO_EMV(SERIAL_KD, Kd_temp);
            }
            else {
              ECHO_E;
            }
          }
          if (hotend < 0)
            Beds[0].soft_pwm = (bias + d) >> 1;
          else
            Hotends[hotend].soft_pwm = (bias + d) >> 1;
          cycles++;
          min = temp;
        }
      }
    }
    if (input > temp + MAX_OVERSHOOT_PID_AUTOTUNE) {
      ECHO_LM(ER, SERIAL_PID_TEMP_TOO_HIGH);
      return;
    }

    // Every 2 seconds...
    if (ms > temp_ms + 2000) {
      #if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HOTEND_0_USES_MAX6675)
        print_heaterstates();
        ECHO_E;
      #endif

      temp_ms = ms;
    } // every 2 seconds

    // Over 2 minutes?
    if (((ms - t1) + (ms - t2)) > (10L*60L*1000L*2L)) {
      ECHO_LM(ER, SERIAL_PID_TIMEOUT);
      return;
    }
    if (cycles > ncycles) {
      ECHO_LM(DB, SERIAL_PID_AUTOTUNE_FINISHED);
      #if ENABLED(PIDTEMP)
        if (hotend >= 0) {
          Hotends[hotend].Kp = Kp_temp;
          Hotends[hotend].Ki = scalePID_i(Ki_temp);
          Hotends[hotend].Kd = scalePID_d(Kd_temp);
          updatePID();

          ECHO_SMV(DB, SERIAL_KP, Hotends[hotend].Kp);
          ECHO_MV(SERIAL_KI, unscalePID_i(Hotends[hotend].Ki));
          ECHO_EMV(SERIAL_KD, unscalePID_d(Hotends[hotend].Kd));
        }
        else {
          ECHO_LMV(DB, "#define DEFAULT_bedKp ", Kp_temp);
          ECHO_LMV(DB, "#define DEFAULT_bedKi ", unscalePID_i(Ki_temp));
          ECHO_LMV(DB, "#define DEFAULT_bedKd ", unscalePID_d(Kd_temp));
        }
      #endif
      return;
    }
    lcd_update();
  }
}

// Apply the scale factors to the PID values
float scalePID_i(float i)   { return i * PID_dT; }
float unscalePID_i(float i) { return i / PID_dT; }
float scalePID_d(float d)   { return d / PID_dT; }
float unscalePID_d(float d) { return d * PID_dT; }
  

#if HAS(AUTO_FAN)
void setExtruderAutoFanState(int pin, bool state) {
  unsigned char newFanSpeed = (state != 0) ? HOTEND_AUTO_FAN_SPEED : HOTEND_AUTO_FAN_MIN_SPEED;
  // this idiom allows both digital and PWM fan outputs (see M42 handling).
  #if ENABLED(FAN_SOFT_PWM)
    fanSpeedSoftPwm_auto = newFanSpeed;
  #else
    digitalWrite(pin, newFanSpeed);
    analogWrite(pin, newFanSpeed);
  #endif
}

void checkExtruderAutoFans() {
  uint8_t fanState = 0;

  // which fan pins need to be turned on?
  #if HAS(AUTO_FAN_0)
    if (Hotends[0].currentTemperatureC > Hotends[0].auto_fan_temperature)
      fanState |= 1;
  #endif
  #if HAS(AUTO_FAN_1)
    if (Hotends[1].currentTemperatureC > Hotends[1].auto_fan_temperature) {
      if (Hotends[1].fan_pin == Hotends[0].fan_pin)
        fanState |= 1;
      else
        fanState |= 2;
    }
  #endif
  #if HAS(AUTO_FAN_2)
    if (Hotends[2].currentTemperatureC > Hotends[2].auto_fan_temperature) {
      if (Hotends[2].fan_pin == Hotends[0].fan_pin)
        fanState |= 1;
      else if (Hotends[2].fan_pin == Hotends[1].fan_pin)
        fanState |= 2;
      else
        fanState |= 4;
    }
  #endif
  #if HAS(AUTO_FAN_3)
    if (Hotends[3].currentTemperatureC > Hotends[3].auto_fan_temperature) {
      if (Hotends[3].fan_pin == Hotends[0].fan_pin) 
        fanState |= 1;
      else if (Hotends[3].fan_pin == Hotends[1].fan_pin)
        fanState |= 2;
      else if (Hotends[3].fan_pin == Hotends[2].fan_pin)
        fanState |= 4;
      else
        fanState |= 8;
    }
  #endif

  // update extruder auto fan states
  #if HAS(AUTO_FAN_0)
    setExtruderAutoFanState(Hotends[0].fan_pin, (fanState & 1) != 0);
  #endif
  #if HAS(AUTO_FAN_1)
    if (Hotends[1].fan_pin != Hotends[0].fan_pin)
      setExtruderAutoFanState(Hotends[1].fan_pin, (fanState & 2) != 0);
  #endif
  #if HAS(AUTO_FAN_2)
    if (Hotends[2].fan_pin != Hotends[0].fan_pin
        && Hotends[2].fan_pin != Hotends[1].fan_pin)
      setExtruderAutoFanState(Hotends[2].fan_pin, (fanState & 4) != 0);
  #endif
  #if HAS(AUTO_FAN_3)
    if (Hotends[3].fan_pin != Hotends[0].fan_pin
        && Hotends[3].fan_pin != Hotends[1].fan_pin
        && Hotends[3].fan_pin != Hotends[2].fan_pin)
      setExtruderAutoFanState(Hotends[3].fan_pin, (fanState & 8) != 0);
  #endif
}

#endif // HAS(AUTO_FAN)

//
// Temperature Error Handlers
//
inline void _temp_error(int h, const char* serial_msg, const char* lcd_msg, bool Hotend = true) {
  static bool killed = false;
  if (IsRunning()) {
    ECHO_ST(ER, serial_msg);

    if (Hotend)
      ECHO_M(SERIAL_STOPPED_HOTEND);
    else
      ECHO_M(SERIAL_STOPPED_BED);

    ECHO_EV((int)h);

    #if ENABLED(ULTRA_LCD)
      lcd_setalertstatuspgm(lcd_msg);
    #endif
  }
  #if DISABLED(BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE)
    if (!killed) {
      Running = false;
      killed = true;
      kill(lcd_msg);
    }
    else
      disable_all_heaters(); // paranoia
  #endif
}

void max_temp_error(uint8_t h, bool Hotend = true) {
  if (Hotend)
    _temp_error(h, PSTR(SERIAL_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
  else
    _temp_error(h, PSTR(SERIAL_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP_BED), false);
}
void min_temp_error(uint8_t h, bool Hotend = true) {
  if (Hotend)
    _temp_error(h, PSTR(SERIAL_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
  else
    _temp_error(h, PSTR(SERIAL_T_MINTEMP), PSTR(MSG_ERR_MINTEMP_BED), false);
}


float get_pid_output(uint8_t h) {
  #if ENABLED(PIDTEMP)
    float pid_output;
    #if ENABLED(PID_OPENLOOP)
      pid_output = constrain(Hotends[h].targetTemperatureC, 0, PID_MAX);
    #else
      pid_error[h] = Hotends[h].targetTemperatureC - Hotends[h].currentTemperatureC;
      dTerm[h] = K2 * Hotends[h].Kd * (Hotends[h].currentTemperatureC - temp_dState[h]) + K1 * dTerm[h];
      temp_dState[h] = Hotends[h].currentTemperatureC;
      if (pid_error[h] > PID_FUNCTIONAL_RANGE) {
        pid_output = BANG_MAX;
        pid_reset[h] = true;
      }
      else if (pid_error[h] < -PID_FUNCTIONAL_RANGE || Hotends[h].targetTemperatureC == 0) {
        pid_output = 0;
        pid_reset[h] = true;
      }
      else {
        if (pid_reset[h]) {
          temp_iState[h] = 0.0;
          pid_reset[h] = false;
        }
        pTerm[h] = Hotends[h].Kp * pid_error[h];
        temp_iState[h] += pid_error[h];
        temp_iState[h] = constrain(temp_iState[h], temp_iState_min[h], temp_iState_max[h]);
        iTerm[h] = Hotends[h].Ki * temp_iState[h];

        pid_output = pTerm[h] + iTerm[h] - dTerm[h];

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          cTerm[h] = 0;
          #if ENABLED(SINGLENOZZLE)
            long e_position = st_get_position(E_AXIS);
            if (e_position > last_position[active_extruder]) {
              lpq[lpq_ptr++] = e_position - last_position[active_extruder];
              last_position[active_extruder] = e_position;
            }
            else {
              lpq[lpq_ptr++] = 0;
            }
            if (lpq_ptr >= lpq_len) lpq_ptr = 0;
            cTerm[0] = (lpq[lpq_ptr] / axis_steps_per_unit[E_AXIS + active_extruder]) * Hotends[0].Kc;
            pid_output += cTerm[0] / 100.0;
          #else  
            if (h == active_extruder) {
              long e_position = st_get_position(E_AXIS);
              if (e_position > last_position[h]) {
                lpq[lpq_ptr++] = e_position - last_position[h];
                last_position[h] = e_position;
              }
              else {
                lpq[lpq_ptr++] = 0;
              }
              if (lpq_ptr >= lpq_len) lpq_ptr = 0;
              cTerm[h] = (lpq[lpq_ptr] / axis_steps_per_unit[E_AXIS + active_extruder]) * Hotends[h].Kc;
              pid_output += cTerm[h] / 100.0;
            }
          #endif // SINGLENOZZLE
        #endif // PID_ADD_EXTRUSION_RATE

        if (pid_output > PID_MAX) {
          if (pid_error[h] > 0) temp_iState[h] -= pid_error[h]; // conditional un-integration
          pid_output = PID_MAX;
        }
        else if (pid_output < 0) {
          if (pid_error[h] < 0) temp_iState[h] -= pid_error[h]; // conditional un-integration
          pid_output = 0;
        }
      }
    #endif // PID_OPENLOOP

    #if ENABLED(PID_DEBUG)
      ECHO_SMV(DB, SERIAL_PID_DEBUG, h);
      ECHO_MV(SERIAL_PID_DEBUG_INPUT, Hotends[h].currentTemperatureC);
      ECHO_MV(SERIAL_PID_DEBUG_OUTPUT, pid_output);
      ECHO_MV(SERIAL_PID_DEBUG_PTERM, pTerm[h]);
      ECHO_MV(SERIAL_PID_DEBUG_ITERM, iTerm[h]);
      ECHO_MV(SERIAL_PID_DEBUG_DTERM, dTerm[h]);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        ECHO_MV(SERIAL_PID_DEBUG_CTERM, cTerm[h]);
      #endif
      ECHO_E;
    #endif // PID_DEBUG

  #else /* PID off */
    pid_output = (Hotends[h].currentTemperatureC < Hotends[h].targetTemperatureC) ? PID_MAX : 0;
  #endif

  return pid_output;
}

#if ENABLED(PIDTEMPBED)
  float get_pid_output_bed(uint8_t h) {
    float pid_output;
    #if ENABLED(PID_OPENLOOP)
      pid_output = constrain(Beds[h].targetTemperatureC, 0, MAX_BED_POWER);
    #else
      pid_error_bed = Beds[h].targetTemperatureC - Beds[h].currentTemperatureC;
      pTerm_bed = Beds[h].Kp * pid_error_bed;
      temp_iState_bed += pid_error_bed;
      temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
      iTerm_bed = Beds[h].Ki * temp_iState_bed;

      dTerm_bed = K2 * Beds[h].Kd * (Beds[h].currentTemperatureC - temp_dState_bed) + K1 * dTerm_bed;
      temp_dState_bed = Beds[h].currentTemperatureC;

      pid_output = pTerm_bed + iTerm_bed - dTerm_bed;
      if (pid_output > MAX_BED_POWER) {
        if (pid_error_bed > 0) temp_iState_bed -= pid_error_bed; // conditional un-integration
        pid_output = MAX_BED_POWER;
      }
      else if (pid_output < 0) {
        if (pid_error_bed < 0) temp_iState_bed -= pid_error_bed; // conditional un-integration
        pid_output = 0;
      }
    #endif // PID_OPENLOOP

    #if ENABLED(PID_BED_DEBUG)
      ECHO_SM(DB ," PID_BED_DEBUG ");
      ECHO_MV(": Input ", Beds[h].currentTemperatureC);
      ECHO_MV(" Output ", pid_output);
      ECHO_MV(" pTerm ", pTerm_bed);
      ECHO_MV(" iTerm ", iTerm_bed);
      ECHO_EMV(" dTerm ", dTerm_bed);
    #endif //PID_BED_DEBUG

    return pid_output;
  }
#endif

/**
 * Manage heating activities for extruder hot-ends and a heated bed
 *  - Acquire updated temperature readings
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void manage_heater() {

  if (!temp_meas_ready) return;

  updateTemperaturesFromRawValues();

  #if ENABLED(HOTEND_0_USES_MAX6675)
    float ct = Hotends[0].currentTemperatureC;
    if (ct > min(HOTEND_0_MAXTEMP, 1023)) max_temp_error(0);
    if (ct < max(HOTEND_0_MINTEMP, 0.01)) min_temp_error(0);
  #endif

  #if ENABLED(THERMAL_PROTECTION_HOTENDS) || DISABLED(PIDTEMPBED) || HAS(AUTO_FAN)
    millis_t ms = millis();
  #endif

  // Loop through all hotends
  for (uint8_t h = 0; h < HOTENDS; h++) {

    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      thermal_runaway_protection(&thermal_runaway_state_machine[h], &thermal_runaway_timer[h], Hotends[h].currentTemperatureC, Hotends[h].targetTemperatureC, h, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
    #endif

    float pid_output = get_pid_output(h);

    // Check if temperature is within the correct range
    Hotends[h].soft_pwm = Hotends[h].currentTemperatureC > Hotends[h].minttempC && Hotends[h].currentTemperatureC < Hotends[h].maxttempC ? (int)pid_output >> 1 : 0;

    // Check if the temperature is failing to increase
    #if ENABLED(THERMAL_PROTECTION_HOTENDS)

      // Is it time to check this extruder's heater?
      if (watch_heater_next_ms[h] && ms > watch_heater_next_ms[h]) {
        // Has it failed to increase enough?
        if (Hotends[h].currentTemperatureC < watch_target_temp[h]) {
          // Stop!
          _temp_error(h, PSTR(SERIAL_T_HEATING_FAILED), PSTR(MSG_HEATING_FAILED_LCD));
        }
        else {
          // Start again if the target is still far off
          start_watching_heater(h);
        }
      }

    #endif // THERMAL_PROTECTION_HOTENDS

  } // Hotends Loop

  #if HAS(AUTO_FAN)
    if (ms > next_auto_fan_check_ms) { // only need to check fan state very infrequently
      checkExtruderAutoFans();
      next_auto_fan_check_ms = ms + 2500;
    }
  #endif

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_SENSOR)
    if (filament_sensor) {
      meas_shift_index = delay_index1 - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed

      // Get the delayed info and add 100 to reconstitute to a percent of
      // the nominal filament diameter then square it to get an area
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);
      float vm = pow((measurement_delay[meas_shift_index] + 100.0) / 100.0, 2);
      NOLESS(vm, 0.01);
      volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vm;
    }
  #endif //FILAMENT_SENSOR

  #if DISABLED(PIDTEMPBED)
    if (ms < next_bed_check_ms) return;
    next_bed_check_ms = ms + BED_CHECK_INTERVAL;
  #endif

  #if BEDS != 0
    // Loop through all beds
    for (uint8_t h = 0; h < BEDS; h++) {

      #if ENABLED(THERMAL_PROTECTION_BED)
        thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, Beds[h].currentTemperatureC, Beds[h].targetTemperatureC, -1, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
      #endif

      #if ENABLED(PIDTEMPBED)
        float pid_output = get_pid_output_bed(h);

        Beds[h].soft_pwm = Beds[h].currentTemperatureC > Beds[h].minttempC && Beds[h].currentTemperatureC < Beds[h].maxttempC ? (int)pid_output >> 1 : 0;

      #elif ENABLED(BED_LIMIT_SWITCHING)
        // Check if temperature is within the correct band
        if (Beds[h].currentTemperatureC > Beds[h].minttempC && Beds[h].currentTemperatureC < Beds[h].maxttempC) {
          if (Beds[h].currentTemperatureC >= Beds[h].targetTemperatureC + BED_HYSTERESIS)
            Beds[h].soft_pwm = 0;
          else if (Beds[h].currentTemperatureC <= Beds[h].targetTemperatureC - BED_HYSTERESIS)
            Beds[h].soft_pwm = MAX_BED_POWER >> 1;
        }
        else {
          Beds[h].soft_pwm = 0;
          WRITE_HEATER(Beds[h].heater_pin, LOW);
        }
      #else // BED_LIMIT_SWITCHING
        // Check if temperature is within the correct range
        if (Beds[h].currentTemperatureC > Beds[h].minttempC && Beds[h].currentTemperatureC < Beds[h].maxttempC) {
          Beds[h].soft_pwm = Beds[h].currentTemperatureC < Beds[h].targetTemperatureC ? MAX_BED_POWER >> 1 : 0;
        }
        else {
          Beds[h].soft_pwm = 0;
          WRITE_HEATER(Beds[h].heater_pin, LOW);
        }
      #endif
    }
  #endif // BEDS != 0
}

void updatePID() {
  for (int h = 0; h < HOTENDS; h++) {
    temp_iState_max[h] = PID_INTEGRAL_DRIVE_MAX / Hotends[h].Ki;
  }
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    for (int e = 0; e < EXTRUDERS; e++) last_position[e] = 0;
  #endif
  #if ENABLED(PIDTEMPBED)
    temp_iState_max_bed = PID_BED_INTEGRAL_DRIVE_MAX / bedKi;
  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues() {
  static millis_t last_update = millis();
  millis_t temp_last_update = millis();
  millis_t from_last_update = temp_last_update - last_update;
  #if ENABLED(HOTEND_0_USES_MAX6675)
    Hotends[0].currentTemperature_raw = read_max6675();
  #endif

  for (uint8_t h = 0; h < HOTENDS; h++) {
    Hotends[h].currentTemperatureC = Hotends[h].analog2temp();
  }

  for (uint8_t h = 0; h < BEDS; h++) {
    Beds[h].currentTemperatureC = Beds[h].analog2temp();
  }

  #if HAS(FILAMENT_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif
  #if HAS(POWER_CONSUMPTION_SENSOR)
    static float watt_overflow = 0.0;
    power_consumption_meas = analog2power();
    /*ECHO_MV("raw:", raw_analog2voltage(), 5);
    ECHO_MV(" - V:", analog2voltage(), 5);
    ECHO_MV(" - I:", analog2current(), 5);
    ECHO_EMV(" - P:", analog2power(), 5);*/
    watt_overflow += (power_consumption_meas * from_last_update) / 3600000.0;
    if (watt_overflow >= 1.0) {
      power_consumption_hour++;
      watt_overflow--;
    }
  #endif

  // Update printer usage
  static unsigned int second_overflow = 0;
  second_overflow += from_last_update;
  if (second_overflow >= 1000) {
    printer_usage_seconds++;
    second_overflow -= 1000;
  }
  last_update = temp_last_update;
  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();
  #endif

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}

#if ENABLED(FILAMENT_SENSOR)
  // Convert raw Filament Width to millimeters
  float analog2widthFil() {
    return current_raw_filwidth / 16383.0 * 5.0;
    // return current_raw_filwidth;
  }

  // Convert raw Filament Width to a ratio
  int widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  //assume sensor cut out
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }

#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  // Convert raw Power Consumption to watt
  float raw_analog2voltage() {
    return (5.0 * current_raw_powconsumption) / (1023.0 * OVERSAMPLENR);
  }

  float analog2voltage() {
    float power_zero_raw = (POWER_ZERO * 1023.0 * OVERSAMPLENR) / 5.0;
    float rel_raw_power = (current_raw_powconsumption < power_zero_raw) ? (2 * power_zero_raw - current_raw_powconsumption) : (current_raw_powconsumption);
    return ((5.0 * rel_raw_power) / (1023.0 * OVERSAMPLENR)) - POWER_ZERO;
  }
  float analog2current() {
    float temp = analog2voltage() / POWER_SENSITIVITY;
    temp = (((100 - POWER_ERROR) / 100) * temp) - POWER_OFFSET;
    return temp > 0 ? temp : 0;
  }
  float analog2power() {
    return (analog2current() * POWER_VOLTAGE * 100) /  POWER_EFFICIENCY;
  }

  float analog2error(float current) {
    float temp1 = (analog2voltage() / POWER_SENSITIVITY - POWER_OFFSET) * POWER_VOLTAGE;
    if(temp1 <= 0) return 0.0;
    float temp2 = (current) * POWER_VOLTAGE;
    if(temp2 <= 0) return 0.0;
    return ((temp2/temp1) - 1) * 100;
  }
  float analog2efficiency(float watt) {
    return (analog2current() * POWER_VOLTAGE * 100) / watt;
  }
#endif

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Heated_init() {
  #if MB(RUMBA) && ((TEMP_SENSOR_0==-1)||(TEMP_SENSOR_1==-1)||(TEMP_SENSOR_2==-1)||(TEMP_SENSOR_BED==-1))
    // disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
    MCUCR = _BV(JTD);
    MCUCR = _BV(JTD);
  #endif

  #if ENABLED(PIDTEMP)
    // Finish init of mult hotends arrays
    for (uint8_t h = 0; h < HOTENDS; h++) {
    // populate with the first value
      temp_iState_min[h] = 0.0;
      temp_iState_max[h] = PID_INTEGRAL_DRIVE_MAX / Hotends[h].Ki;
    }
  #endif //PIDTEMP

  #if ENABLED(PIDTEMPBED)
    for (uint8_t h = 0; h < BEDS; h++) {
      temp_iState_min_bed = 0.0;
      temp_iState_max_bed = PID_BED_INTEGRAL_DRIVE_MAX / Beds[0].Ki;
    }
  #endif // PIDTEMPBED

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    for (uint8_t e = 0; e < EXTRUDERS; e++) last_position[e] = 0;
  #endif

  for (uint8_t h = 0; h < HOTENDS; h++)
    SET_OUTPUT(Hotends[h].heater_pin);

  for (uint8_t h = 0; h < BEDS; h++)
    SET_OUTPUT(Beds[h].heater_pin);

  #if HAS(FAN)
    SET_OUTPUT(FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #if ENABLED(FAN_SOFT_PWM)
      soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif

  #if ENABLED(HOTEND_0_USES_MAX6675)

    #if DISABLED(SDSUPPORT)
      WRITE(SCK_PIN, 0);
      SET_OUTPUT(SCK_PIN);
      WRITE(MOSI_PIN, 1);
      SET_OUTPUT(MOSI_PIN);
      WRITE(MISO_PIN, 1);
      SET_INPUT(MISO_PIN);
    #endif

    OUT_WRITE(MAX6675_SS, HIGH);

  #endif // HOTEND_0_USES_MAX6675

  // Set analog inputs
  // Setup channels
  ADC->ADC_MR |= ADC_MR_FREERUN_ON |
               ADC_MR_LOWRES_BITS_12;

  for (uint8_t h = 0; h < HOTENDS; h++)
    startAdcConversion(pinToAdcChannel(Hotends[h].sensor_pin));

  for (uint8_t h = 0; h < BEDS; h++)
    startAdcConversion(pinToAdcChannel(Beds[h].sensor_pin));

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt

  HAL_temp_timer_start(TEMP_TIMER_NUM);
  HAL_timer_enable_interrupt (TEMP_TIMER_NUM);

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);
}

#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  /**
   * Start Heating Sanity Check for hotends that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M104, M109)
   */
  void start_watching_heater(int h) {
    if (Hotends[h].currentTemperatureC < Hotends[h].targetTemperatureC - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp[h] = Hotends[h].currentTemperatureC + WATCH_TEMP_INCREASE;
      watch_heater_next_ms[h] = millis() + WATCH_TEMP_PERIOD * 1000UL;
    }
    else
      watch_heater_next_ms[h] = 0;
  }
#endif

#if ENABLED(THERMAL_PROTECTION_HOTENDS) || ENABLED(THERMAL_PROTECTION_BED)

  void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float targetTemperatureC, int heater_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[HOTENDS + 1] = { 0.0 };

    /*
        ECHO_SM(DB, "Thermal Thermal Runaway Running. Heater ID: ");
        if (heater_id < 0) ECHO_M("bed"); else ECHO_V(heater_id);
        ECHO_MV(" ;  State:", *state);
        ECHO_MV(" ;  Timer:", *timer);
        ECHO_MV(" ;  Temperature:", temperature);
        ECHO_EMV(" ;  Target Temp:", targetTemperatureC);
    */

    int heater_index = heater_id >= 0 ? heater_id : HOTENDS;

    // If the target temperature changes, restart
    if (tr_target_temperature[heater_index] != targetTemperatureC)
      *state = TRReset;

    switch (*state) {
      case TRReset:
        *timer = 0;
        *state = TRInactive;
      // Inactive state waits for a target temperature to be set
      case TRInactive:
        if (targetTemperatureC > 0) {
          tr_target_temperature[heater_index] = targetTemperatureC;
          *state = TRFirstHeating;
        }
        break;
      // When first heating, wait for the temperature to be reached then go to Stable state
      case TRFirstHeating:
        if (temperature >= tr_target_temperature[heater_index]) *state = TRStable;
        break;
      // While the temperature is stable watch for a bad temperature
      case TRStable:
        // If the temperature is over the target (-hysteresis) restart the timer
        if (temperature >= tr_target_temperature[heater_index] - hysteresis_degc)
          *timer = millis();
        // If the timer goes too long without a reset, trigger shutdown
        else if (millis() > *timer + period_seconds * 1000UL)
          *state = TRRunaway;
        break;
      case TRRunaway:
        _temp_error(heater_id, PSTR(SERIAL_T_THERMAL_RUNAWAY), PSTR(MSG_THERMAL_RUNAWAY));
    }
  }

#endif // THERMAL_PROTECTION_HOTENDS || THERMAL_PROTECTION_BED

void disable_all_heaters() {
  for (uint8_t h = 0; h < HOTENDS; h++) {
    setTargetCelsius(0, h);
    Hotends[h].soft_pwm = 0;
    WRITE_HEATER(Hotends[h].heater_pin, LOW);
  }

  for (uint8_t h = 0; h < BEDS; h++) {
    Beds[h].targetTemperatureC = 0;
    Beds[h].soft_pwm = 0;
    WRITE_HEATER(Beds[h].heater_pin, LOW);
  }
}

#if ENABLED(HOTEND_0_USES_MAX6675)
  static millis_t last_max6675_read = 0;
  static int16_t max6675_temp = 0;

  int16_t read_max6675() {

    if (HAL::timeInMilliseconds() - last_max6675_read > 230) {
      HAL::spiBegin();
      HAL::spiInit(2);
      HAL::digitalWrite(MAX6675_SS, 0);  // enable TT_MAX6675
      HAL::delayMicroseconds(1);    // ensure 100ns delay - a bit extra is fine
      max6675_temp = HAL::spiReceive();
      max6675_temp <<= 8;
      max6675_temp |= HAL::spiReceive();
      HAL::digitalWrite(MAX6675_SS, 1);  // disable TT_MAX6675
      last_max6675_read = HAL::timeInMilliseconds();
    }
    return max6675_temp & 4 ? 2000 : max6675_temp >> 3; // thermocouple open?
  }
#endif // HOTEND_0_USES_MAX6675

/**
 * Stages in the ISR loop
 */
enum TempState {
  MeasureTemp_0,
  MeasureTemp_1,
  MeasureTemp_2,
  MeasureTemp_3,
  MeasureTemp_BED_0,
  MeasureTemp_BED_1,
  MeasureTemp_BED_2,
  MeasureTemp_BED_3,
  Prepare_FILWIDTH,
  Measure_FILWIDTH,
  Prepare_POWCONSUMPTION,
  Measure_POWCONSUMPTION,
  StartupDelay // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

//
// Timer 0 is shared with millies
//
HEATED_TIMER_ISR {
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static uint32_t raw_temp_value[HOTENDS];
  static uint32_t raw_temp_bed_value[BEDS];
  static TempState temp_state = StartupDelay;
  static unsigned char pwm_count = _BV(SOFT_PWM_SCALE);
  static int max_temp[HOTENDS];
  static int min_temp[HOTENDS];
  static int max_temp_bed[BEDS];
  static int min_temp_bed[BEDS];
  static int temp_read = 0;

  static unsigned char median_counter = 0;
  static unsigned long raw_median_temp[HOTENDS][MEDIAN_COUNT];
  static unsigned long raw_median_temp_bed[BEDS][MEDIAN_COUNT];
  static bool first_start = true;

  // Static members for each heater
  #if ENABLED(SLOW_PWM_HEATERS)
    static unsigned char slow_pwm_count = 0;
    static unsigned char soft_pwm[HOTENDS];
    static unsigned char state_heater[HOTENDS];
    static unsigned char state_timer_heater[HOTENDS];
    static unsigned char soft_pwm_bed[BEDS];
    static unsigned char state_heater_bed[BEDS];
    static unsigned char state_timer_heater_bed[BEDS];
  #else
    static unsigned char soft_pwm[HOTENDS];
    static unsigned char soft_pwm_bed[BEDS];
  #endif

  #if HAS(FILAMENT_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  // Initialize some variables only at start!
  if (first_start) {
	  for (uint8_t h = 0; h < HOTENDS; h++) {
      max_temp[h] = 0;
      min_temp[h] = 123000;
		  for (int j = 0; j < MEDIAN_COUNT; j++) raw_median_temp[h][j] = 3600 * OVERSAMPLENR;
	  }
    for (uint8_t h = 0; h < BEDS; h++) {
      max_temp_bed[h] = 0;
      min_temp_bed[h] = 123000;
		  for (int j = 0; j < MEDIAN_COUNT; j++) raw_median_temp_bed[h][j] = 3600 * OVERSAMPLENR;
	  }
	  first_start = false;
	  ECHO_EM("First start for temperature finished.");
  }

  HAL_timer_isr_status (TEMP_TIMER_COUNTER, TEMP_TIMER_CHANNEL);

  #if DISABLED(SLOW_PWM_HEATERS)
    /**
     * standard PWM modulation
     */
    if (pwm_count == 0) {
      for (uint8_t h = 0; h < HOTENDS; h++) {
        soft_pwm[h] = Hotends[h].soft_pwm;
        WRITE_HEATER(Hotends[h].heater_pin, soft_pwm[h] > 0 ? 1 : 0);
      }

      for (uint8_t h = 0; h < BEDS; h++) {
        soft_pwm_bed[h] = Beds[h].soft_pwm;
        WRITE_HEATER(Beds[h].heater_pin, soft_pwm_bed[h] > 0 ? 1 : 0);
      }

      #if ENABLED(FAN_SOFT_PWM)
        soft_pwm_fan = fanSpeedSoftPwm / 2;
        #if HAS(CONTROLLERFAN)
          soft_pwm_fan_controller = fanSpeedSoftPwm_controller / 2;
          WRITE_FAN(CONTROLLERFAN_PIN, soft_pwm_fan_controller > 0 ? 1 : 0);
        #endif
        WRITE_FAN(soft_pwm_fan > 0 ? 1 : 0);
        #if HAS(AUTO_FAN)
          soft_pwm_fan_auto = fanSpeedSoftPwm_auto / 2;
        #endif
        #if HAS(AUTO_FAN_0)
          WRITE_FAN(Hotends[0].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN_1)
          WRITE_FAN(Hotends[1].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN_2)
          WRITE_FAN(Hotends[2].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN_3)
          WRITE_FAN(Hotends[3].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
      #endif
    }

    for (uint8_t h = 0; h < HOTENDS; h++)
      if (soft_pwm[h] < pwm_count) WRITE_HEATER(Hotends[h].heater_pin, 0);

    for (uint8_t h = 0; h < BEDS; h++)
      if (soft_pwm_bed[h] < pwm_count) WRITE_HEATER(Beds[h].heater_pin, 0);

    #if ENABLED(FAN_SOFT_PWM)
      if (soft_pwm_fan < pwm_count) WRITE_FAN(0);
      #if HAS(CONTROLLERFAN)
        if (soft_pwm_fan_controller < pwm_count) WRITE_FAN(CONTROLLERFAN_PIN, 0);
      #endif
      #if HAS(AUTO_FAN)
        if (soft_pwm_fan_auto < pwm_count) {
          #if HAS(AUTO_FAN_0)
            WRITE_FAN(Hotends[0].fan_pin, 0);
          #endif
          #if HAS(AUTO_FAN_1)
            WRITE_FAN(Hotends[1].fan_pin, 0);
          #endif
          #if HAS(AUTO_FAN_2)
            WRITE_FAN(Hotends[2].fan_pin, 0);
          #endif
          #if HAS(AUTO_FAN_3)
            WRITE_FAN(Hotends[3].fan_pin, 0);
          #endif
        }
      #endif
    #endif

    pwm_count += _BV(SOFT_PWM_SCALE);
    pwm_count &= 0x7f;

  #else // SLOW_PWM_HEATERS

    /*
     * SLOW PWM HEATERS
     *
     * for heaters drived by relay
     */
    #if DISABLED(MIN_STATE_TIME)
      #define MIN_STATE_TIME 16 // MIN_STATE_TIME * 65.5 = time in milliseconds
    #endif

    if (slow_pwm_count == 0) {
      for (uint8_t h = 0; h < HOTENDS; h++) {
        soft_pwm[h] = Hotends[h].soft_pwm;
        if (soft_pwm[h] > 0) {
          if (state_timer_heater[h] == 0) {
            if (state_heater[h] == 0) state_timer_heater[h] = MIN_STATE_TIME;
            state_heater[h] = 1;
            WRITE_HEATER(Hotends[h].heater_pin, 1);
          }
        }
        else {
          if (state_timer_heater[h] == 0) {
            if (state_heater[h] == 1) state_timer_heater[h] = MIN_STATE_TIME;
            state_heater[h] = 0;
            WRITE_HEATER(Hotends[h].heater_pin, 0);
          }
        }
      }

      for (uint8_t h = 0; h < BEDS; h++) {
        soft_pwm_bed[h] = Beds[h].soft_pwm;
        if (soft_pwm_bed[h] > 0) {
          if (state_timer_heater_bed[h] == 0) {
            if (state_heater_bed[h] == 0) state_timer_heater_bed[h] = MIN_STATE_TIME;
            state_heater_bed[h] = 1;
            WRITE_HEATER(Beds[h].heater_pin, 1);
          }
        }
        else {
          if (state_timer_heater_bed[h] == 0) {
            if (state_heater_bed[h] == 1) state_timer_heater_bed[h] = MIN_STATE_TIME;
            state_heater_bed[h] = 0;
            WRITE_HEATER(Beds[h].heater_pin, 0);
          }
        }
      }
    } // slow_pwm_count == 0

    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (soft_pwm[h] < slow_pwm_count) {
        if (state_timer_heater[h] == 0) {
          if (state_heater[h] == 1) state_timer_heater[h] = MIN_STATE_TIME;
          state_heater[h] = 0;
          WRITE_HEATER(Hotends[h].heater_pin, 0);
        }
      }
    }
    
    for (uint8_t h = 0; h < BEDS; h++) {
      if (soft_pwm_BED < slow_pwm_count) {
        if (state_timer_heater_bed[h] == 0) {
          if (state_heater_bed[h] == 1) state_timer_heater_bed[h] = MIN_STATE_TIME;
          state_heater_bed[h] = 0;
          WRITE_HEATER(Beds[h].heater_pin, 0);
        }
      }
    }

    #if ENABLED(FAN_SOFT_PWM)
      if (pwm_count == 0) {
        soft_pwm_fan = fanSpeedSoftPwm / 2;
        WRITE_FAN(soft_pwm_fan > 0 ? 1 : 0);
        #if HAS(CONTROLLERFAN)
          soft_pwm_fan_controller = fanSpeedSoftPwm_controller / 2;
          WRITE_FAN(CONTROLLERFAN_PIN, soft_pwm_fan_controller > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN)
          soft_pwm_fan_auto = fanSpeedSoftPwm_auto / 2;
        #endif
        #if HAS(AUTO_FAN_0)
          WRITE_FAN(, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN_1)
          WRITE_FAN(Hotends[1].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN_2)
          WRITE_FAN(Hotends[2].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
        #if HAS(AUTO_FAN_3)
          WRITE_FAN(Hotends[3].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
        #endif
      }
      if (soft_pwm_fan < pwm_count) WRITE_FAN(0);
      #if HAS(CONTROLLERFAN)
        if (soft_pwm_fan_controller < pwm_count) WRITE_FAN(CONTROLLERFAN_PIN, 0);
      #endif
      #if HAS(AUTO_FAN)
        if (soft_pwm_fan_auto < pwm_count) {
          #if HAS(AUTO_FAN_0)
            WRITE_FAN(Hotends[0].fan_pin, 0);
          #endif
          #if HAS(AUTO_FAN_1)
            WRITE_FAN(Hotends[1].fan_pin, 0);
          #endif
          #if HAS(AUTO_FAN_2)
            WRITE_FAN(Hotends[2].fan_pin, 0);
          #endif
          #if HAS(AUTO_FAN_3)
            WRITE_FAN(Hotends[3].fan_pin, 0);
          #endif
        }
      #endif
    #endif // FAN_SOFT_PWM

    pwm_count += _BV(SOFT_PWM_SCALE);
    pwm_count &= 0x7f;

    // increment slow_pwm_count only every 64 pwm_count circa 65.5ms
    if ((pwm_count % 64) == 0) {
      slow_pwm_count++;
      slow_pwm_count &= 0x7f;

      for (uint8_t h = 0; h < HOTENDS; h++)
        if (state_timer_heater[h] > 0) state_timer_heater[h]--;

      for (uint8_t h = 0; h < BEDS; h++)
        if (state_timer_heater_bed[h] > 0) state_timer_heater_bed[h]--;
    } // (pwm_count % 64) == 0

  #endif // SLOW_PWM_HEATERS

  #define READ_TEMP(temp_id) temp_read = getAdcFreerun(pinToAdcChannel(Hotends[temp_id].sensor_pin)); \
    raw_temp_value[temp_id] += temp_read; \
    max_temp[temp_id] = max(max_temp[temp_id], temp_read); \
    min_temp[temp_id] = min(min_temp[temp_id], temp_read)

  #define READ_BED_TEMP(temp_id) temp_read = getAdcFreerun(pinToAdcChannel(Beds[temp_id].sensor_pin)); \
    raw_temp_bed_value[temp_id] += temp_read; \
    max_temp_bed[temp_id] = max(max_temp_bed[temp_id], temp_read); \
    min_temp_bed[temp_id] = min(min_temp_bed[temp_id], temp_read)

  // Prepare or measure a sensor, each one every 14th frame
  switch(temp_state) {
    case MeasureTemp_0:
      #if HAS(TEMP_0)
        READ_TEMP(0);
      #endif
      temp_state = MeasureTemp_1;
      break;
    case MeasureTemp_1:
      #if HAS(TEMP_1)
        READ_TEMP(1);
      #endif
      temp_state = MeasureTemp_2;
      break;
    case MeasureTemp_2:
      #if HAS(TEMP_2)
        READ_TEMP(2);
      #endif
      temp_state = MeasureTemp_3;
      break;
    case MeasureTemp_3:
      #if HAS(TEMP_3)
        READ_TEMP(3);
      #endif
      temp_state = MeasureTemp_BED_0;
      break;

    case MeasureTemp_BED_0:
      #if HAS(TEMP_BED_0)
        READ_BED_TEMP(0);
      #endif
      temp_state = MeasureTemp_BED_1;
      break;
    case MeasureTemp_BED_1:
      #if HAS(TEMP_BED_1)
        READ_BED_TEMP(1);
      #endif
      temp_state = MeasureTemp_BED_2;
      break;
    case MeasureTemp_BED_2:
      #if HAS(TEMP_BED_2)
        READ_BED_TEMP(2);
      #endif
      temp_state = MeasureTemp_BED_3;
      break;
    case MeasureTemp_BED_3:
      #if HAS(TEMP_BED_3)
        READ_BED_TEMP(3);
      #endif
      temp_state = Prepare_FILWIDTH;
      break;
    case Prepare_FILWIDTH:
      #if HAS(FILAMENT_SENSOR)
      // nothing todo for Due
      #endif
      lcd_buttons_update();
      temp_state = Measure_FILWIDTH;
      break;
    case Measure_FILWIDTH:
      #if HAS(FILAMENT_SENSOR)
        // raw_filwidth_value += ADC;  //remove to use an IIR filter approach
        if (ADC > 102) { //check that ADC is reading a voltage > 0.5 volts, otherwise don't take in the data.
          raw_filwidth_value -= (raw_filwidth_value >> 7); //multiply raw_filwidth_value by 127/128
          raw_filwidth_value += ((unsigned long)ADC << 7); //add new ADC reading
        }
      #endif
      temp_state = Prepare_POWCONSUMPTION;
      break;

    case Prepare_POWCONSUMPTION:
      #if HAS(POWER_CONSUMPTION_SENSOR)
      // nothing todo for Due
      #endif
      lcd_buttons_update();
      temp_state = Measure_POWCONSUMPTION;
      break;
    case Measure_POWCONSUMPTION:
      #if HAS(POWER_CONSUMPTION_SENSOR)
        raw_powconsumption_value = analogRead(POWER_CONSUMPTION_PIN);
      #endif
      temp_state = MeasureTemp_0;
      temp_count++;
      break;

    case StartupDelay:
      temp_state = MeasureTemp_0;
      break;

    // default:
    //  ECHO_LM(ER, MSG_TEMP_READ_ERROR);
    //  break;
  } // switch(temp_state)

  #define SET_CURRENT_TEMP_RAW(temp_id) raw_median_temp[temp_id][median_counter] = (raw_temp_value[temp_id] - (min_temp[temp_id] + max_temp[temp_id])); \
    sum = 0; \
    for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp[temp_id][i]; \
    Hotends[temp_id].currentTemperature_raw = (sum / MEDIAN_COUNT + 4) >> 2
      
  #define SET_CURRENT_BED_RAW(temp_id) raw_median_temp_bed[temp_id][median_counter] = (raw_temp_bed_value[temp_id] - (min_temp_bed[temp_id] + max_temp_bed[temp_id])); \
    sum = 0; \
    for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp_bed[temp_id][i]; \
    Beds[temp_id].currentTemperature_raw = (sum / MEDIAN_COUNT + 4) >> 2

  if(temp_count >= OVERSAMPLENR + 2) { // 14 * 16 * 1/(16000000/64/256)  = 164ms.
    if (!temp_meas_ready) { //Only update the raw values if they have been read. Else we could be updating them during reading.
      unsigned long sum = 0;
      #ifndef HOTEND_0_USES_MAX6675
        SET_CURRENT_TEMP_RAW(0);
      #endif
      #if HOTENDS > 1
        SET_CURRENT_TEMP_RAW(1);
        #if HOTENDS > 2
          SET_CURRENT_TEMP_RAW(2);
          #if HOTENDS > 3
            SET_CURRENT_TEMP_RAW(3);
          #endif
        #endif
      #endif
      #if BEDS > 0
        SET_CURRENT_BED_RAW(0);
        #if BEDS > 1
          SET_CURRENT_BED_RAW(1);
          #if BEDS > 2
            SET_CURRENT_BED_RAW(2);
            #if BEDS > 3
              SET_CURRENT_BED_RAW(3);
            #endif
          #endif
        #endif
      #endif

      //  Reset min/max-holder
      for (uint8_t h = 0; h < HOTENDS; h++) {
        max_temp[h] = 0;
        min_temp[h] = 123000;
      }
      for (uint8_t h = 0; h < BEDS; h++) {
        max_temp_bed[h] = 0;
        min_temp_bed[h] = 123000;
      }
    } //!temp_meas_ready

    // Filament Sensor - can be read any time since IIR filtering is used
    #if HAS(FILAMENT_SENSOR)
      current_raw_filwidth = raw_filwidth_value >> 10;  // Divide to get to 0-16384 range since we used 1/128 IIR filter approach
    #endif

    median_counter++;
    if (median_counter >= MEDIAN_COUNT) median_counter = 0;

    temp_meas_ready = true;
    temp_count = 0;
    for (uint8_t h = 0; h < HOTENDS; h++) raw_temp_value[h] = 0;
    for (uint8_t h = 0; h < BEDS; h++) raw_temp_bed_value[h] = 0;

    #if HAS(POWER_CONSUMPTION_SENSOR)
      raw_powconsumption_value = 0;
    #endif

    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (Hotends[h].currentTemperatureC < Hotends[h].minttempC ) min_temp_error(h);
      if (Hotends[h].currentTemperatureC > Hotends[h].maxttempC ) max_temp_error(h);
    }

    for (uint8_t h = 0; h < BEDS; h++) {
      if (Beds[h].currentTemperatureC < Beds[h].minttempC ) min_temp_error(h);
      if (Beds[h].currentTemperatureC > Beds[h].maxttempC ) max_temp_error(h);
    }
  } // temp_count >= OVERSAMPLENR

  #if ENABLED(BABYSTEPPING)
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
      int curTodo = babystepsTodo[axis]; //get rid of volatile for performance

      if (curTodo > 0) {
        babystep(axis,/*fwd*/true);
        babystepsTodo[axis]--; //fewer to do next time
      }
      else if (curTodo < 0) {
        babystep(axis,/*fwd*/false);
        babystepsTodo[axis]++; //fewer to do next time
      }
    }
  #endif //BABYSTEPPING
}
