/**
 * Heater.cpp
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
#include "heater.h"

//===========================================================================
//================================== macros =================================
//===========================================================================

#if ENABLED(K1) // Defined in Configuration_base.h in the PID settings
  #define K2 (1.0 - K1)
#endif

//===========================================================================
//================================== Object =================================
//===========================================================================
#if NUM_HEATER == 0
  Heater Heaters[1];
#else
  Heater Heaters[NUM_HEATER] = {
  #if NUM_HEATER > 0
    {
      0, TEMP_SENSOR_0, TEMP_0_PIN, HEATER_0_PIN, HEATER_0_AUTO_FAN_PIN,
      0, 0, HEATER_0_MINTEMP, HEATER_0_MAXTEMP, 0, 0,   // Temperature
      HEATER_0_PIDTEMP, 0, 0, 0, 100, HEATER_0_PID_MAX, // PID
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN,
      true, HEATER_0_AUTO_FAN_TEMPERATURE, 0
    }
  #endif
  #if NUM_HEATER > 1
    {
      0, TEMP_SENSOR_1, TEMP_1_PIN, HEATER_1_PIN, HEATER_1_AUTO_FAN_PIN,
      0, 0, HEATER_1_MINTEMP, HEATER_1_MAXTEMP, 0, 0,   // Temperature
      HEATER_1_PIDTEMP, 0, 0, 0, 100, HEATER_1_PID_MAX, // PID
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN,
      HAS_AUTO_FAN_1, HEATER_1_AUTO_FAN_TEMPERATURE, 0
    }
  #endif
  #if NUM_HEATER > 2
    {
      0, TEMP_SENSOR_2, TEMP_2_PIN, HEATER_2_PIN, HEATER_2_AUTO_FAN_PIN,
      0, 0, HEATER_2_MINTEMP, HEATER_2_MAXTEMP, 0, 0,   // Temperature
      HEATER_2_PIDTEMP, 0, 0, 0, 100, HEATER_2_PID_MAX, // PID
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN,
      HAS_AUTO_FAN_2, HEATER_2_AUTO_FAN_TEMPERATURE, 0
    }
  #endif
  #if NUM_HEATER > 3
    {
      0, TEMP_SENSOR_3, TEMP_3_PIN, HEATER_3_PIN, HEATER_3_AUTO_FAN_PIN,
      0, 0, HEATER_3_MINTEMP, HEATER_3_MAXTEMP, 0, 0,   // Temperature
      HEATER_3_PIDTEMP, 0, 0, 0, 100, HEATER_3_PID_MAX, // PID
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN,
      HAS_AUTO_FAN_3, HEATER_3_AUTO_FAN_TEMPERATURE, 0
    }
  #endif
  #if NUM_HEATER > 4
    {
      0, TEMP_SENSOR_4, TEMP_4_PIN, HEATER_4_PIN, HEATER_4_AUTO_FAN_PIN,
      0, 0, HEATER_4_MINTEMP, HEATER_4_MAXTEMP, 0, 0,   // Temperature
      HEATER_4_PIDTEMP, 0, 0, 0, 100, HEATER_4_PID_MAX, // PID
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN,
      HAS_AUTO_FAN_4, HEATER_4_AUTO_FAN_TEMPERATURE, 0
    }
  #endif
  #if NUM_HEATER > 5
    {
      0, TEMP_SENSOR_5, TEMP_5_PIN, HEATER_5_PIN, HEATER_5_AUTO_FAN_PIN,
      0, 0, HEATER_5_MINTEMP, HEATER_5_MAXTEMP, 0, 0,   // Temperature
      HEATER_5_PIDTEMP, 0, 0, 0, 100, HEATER_5_PID_MAX, // PID
      {0}, TEMP_SENSOR_AD595_OFFSET, TEMP_SENSOR_AD595_GAIN,
      HAS_AUTO_FAN_5, HEATER_5_AUTO_FAN_TEMPERATURE, 0
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
    unsigned char fanSpeedSoftPwm_auto = HEATER_AUTO_FAN_MIN_SPEED;
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
static float temp_iState[NUM_HEATER] = { 0 };
static float temp_dState[NUM_HEATER] = { 0 };
static float pTerm[NUM_HEATER];
static float iTerm[NUM_HEATER];
static float dTerm[NUM_HEATER];
#if ENABLED(PID_ADD_EXTRUSION_RATE)
  static float cTerm[NUM_HEATER];
  static long last_position[EXTRUDERS];
  static long lpq[LPQ_MAX_LEN];
  static int lpq_ptr = 0;
#endif
//int output;
static float pid_error[NUM_HEATER];
static float temp_iState_min[NUM_HEATER];
static float temp_iState_max[NUM_HEATER];
static bool pid_reset[NUM_HEATER];

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
  int watch_target_temp[NUM_HEATER] = { 0 };
  millis_t watch_heater_next_ms[NUM_HEATER] = { 0 };
#endif

#if DISABLED(SOFT_PWM_SCALE)
  #define SOFT_PWM_SCALE 0
#endif

#if ENABLED(FILAMENT_SENSOR)
  static int meas_shift_index;  // used to point to a delayed sample in buffer for filament width sensor
#endif

#if ENABLED(HEATER_0_USES_MAX6675)
  static int16_t read_max6675();
#endif

//===========================================================================
//================================ Functions ================================
//===========================================================================

void autotempShutdown() {
  #if ENABLED(AUTOTEMP)
    if (autotemp_enabled) {
      autotemp_enabled = false;
      if (Heaters[active_extruder].targetTemperatureC > autotemp_min)
        setTargetCelsius(0, active_extruder);
    }
  #endif
}

void PID_autotune(float temp, int heater, int ncycles) {
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

  if (heater >= NUM_HEATER) {
    ECHO_LM(ER, SERIAL_PID_BAD_HEATER_NUM);
    return;
  }

  ECHO_LM(DB, SERIAL_PID_AUTOTUNE_START);
  ECHO_SMV(DB, "Heater: ", heater);
  ECHO_MV(" Temp: ", temp);
  ECHO_EMV(" Cycles: ", ncycles);

  disable_all_heaters(); // switch off all heaters.

  Heaters[heater].soft_pwm = bias = d = Heaters[heater].pid_max / 2;

  // PID Tuning loop
  for (;;) {

    millis_t ms = millis();

    if (temp_meas_ready) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = Heaters[heater].currentTemperatureC;

      max = max(max, input);
      min = min(min, input);

      #if HAS(AUTO_FAN)
        if (ms > next_auto_fan_check_ms) {
          checkHeaterAutoFans();
          next_auto_fan_check_ms = ms + 2500;
        }
      #endif

      if (heating && input > temp) {
        if (ms > t2 + 5000) {
          heating = false;
          Heaters[heater].soft_pwm = (bias - d) >> 1;
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
            long max_pow = Heaters[heater].pid_max;
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

          Heaters[heater].soft_pwm = (bias + d) >> 1;
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
      print_heaterstates();
      ECHO_E;

      temp_ms = ms;
    } // every 2 seconds

    // Over 2 minutes?
    if (((ms - t1) + (ms - t2)) > (10L*60L*1000L*2L)) {
      ECHO_LM(ER, SERIAL_PID_TIMEOUT);
      return;
    }
    if (cycles > ncycles) {
      ECHO_LM(DB, SERIAL_PID_AUTOTUNE_FINISHED);
      if (heater >= 0) {
        Heaters[heater].Kp = Kp_temp;
        Heaters[heater].Ki = scalePID_i(Ki_temp);
        Heaters[heater].Kd = scalePID_d(Kd_temp);
        updatePID();

        ECHO_SMV(DB, SERIAL_KP, Heaters[heater].Kp);
        ECHO_MV(SERIAL_KI, unscalePID_i(Heaters[heater].Ki));
        ECHO_EMV(SERIAL_KD, unscalePID_d(Heaters[heater].Kd));
      }
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
  void setHeaterAutoFanState(int pin, bool state) {
    unsigned char newFanSpeed = (state != 0) ? HEATER_AUTO_FAN_SPEED : HEATER_AUTO_FAN_MIN_SPEED;
    // this idiom allows both digital and PWM fan outputs (see M42 handling).
    #if ENABLED(FAN_SOFT_PWM)
      fanSpeedSoftPwm_auto = newFanSpeed;
    #else
      digitalWrite(pin, newFanSpeed);
      analogWrite(pin, newFanSpeed);
    #endif
  }

  void checkHeaterAutoFans() {
    uint8_t fanState = 0;

    // which fan pins need to be turned on?
    for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
      if (Heaters[heater].has_fan) {
        if (Heaters[heater].currentTemperatureC > Heaters[heater].auto_fan_temperature)
          fanState |= (int)pow(2, heater);
        setHeaterAutoFanState(Heaters[heater].fan_pin, (fanState & (int)pow(2, heater)) != 0);
      }
    }
  }
#endif // HAS(AUTO_FAN)

//
// Temperature Error Handlers
//
inline void _temp_error(uint8_t h, const char* serial_msg, const char* lcd_msg) {
  static bool killed = false;
  if (IsRunning()) {
    ECHO_ST(ER, serial_msg);
    ECHO_M(SERIAL_STOPPED_HEATER);
    ECHO_EV(h);

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

void max_temp_error(uint8_t h) {
  _temp_error(h, PSTR(SERIAL_T_MAXTEMP), PSTR(MSG_ERR_MAXTEMP));
}
void min_temp_error(uint8_t h) {
  _temp_error(h, PSTR(SERIAL_T_MINTEMP), PSTR(MSG_ERR_MINTEMP));
}

float get_pid_output(uint8_t heater) {
  float pid_output;

  if (Heaters[heater].use_pid) {
    #if ENABLED(PID_OPENLOOP)
      pid_output = constrain(Heaters[heater].targetTemperatureC, 0, Heaters[heater].pid_max);
    #else
      pid_error[heater] = Heaters[heater].targetTemperatureC - Heaters[heater].currentTemperatureC;
      dTerm[heater] = K2 * Heaters[heater].Kd * (Heaters[heater].currentTemperatureC - temp_dState[heater]) + K1 * dTerm[heater];
      temp_dState[heater] = Heaters[heater].currentTemperatureC;
      if (pid_error[heater] > PID_FUNCTIONAL_RANGE) {
        pid_output = BANG_MAX;
        pid_reset[heater] = true;
      }
      else if (pid_error[heater] < -PID_FUNCTIONAL_RANGE || Heaters[heater].targetTemperatureC == 0) {
        pid_output = 0;
        pid_reset[heater] = true;
      }
      else {
        if (pid_reset[heater]) {
          temp_iState[heater] = 0.0;
          pid_reset[heater] = false;
        }
        pTerm[heater] = Heaters[heater].Kp * pid_error[heater];
        temp_iState[heater] += pid_error[heater];
        temp_iState[heater] = constrain(temp_iState[heater], temp_iState_min[heater], temp_iState_max[heater]);
        iTerm[heater] = Heaters[heater].Ki * temp_iState[heater];

        pid_output = pTerm[heater] + iTerm[heater] - dTerm[heater];

        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          cTerm[heater] = 0;
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
            cTerm[0] = (lpq[lpq_ptr] / axis_steps_per_unit[E_AXIS + active_extruder]) * Heaters[0].Kc;
            pid_output += cTerm[0] / 100.0;
          #else  
            if (heater == active_extruder) {
              long e_position = st_get_position(E_AXIS);
              if (e_position > last_position[heater]) {
                lpq[lpq_ptr++] = e_position - last_position[heater];
                last_position[heater] = e_position;
              }
              else {
                lpq[lpq_ptr++] = 0;
              }
              if (lpq_ptr >= lpq_len) lpq_ptr = 0;
              cTerm[heater] = (lpq[lpq_ptr] / axis_steps_per_unit[E_AXIS + active_extruder]) * Heaters[heater].Kc;
              pid_output += cTerm[heater] / 100.0;
            }
          #endif // SINGLENOZZLE
        #endif // PID_ADD_EXTRUSION_RATE

        if (pid_output > Heaters[heater].pid_max) {
          if (pid_error[heater] > 0) temp_iState[heater] -= pid_error[heater]; // conditional un-integration
          pid_output = Heaters[heater].pid_max;
        }
        else if (pid_output < 0) {
          if (pid_error[heater] < 0) temp_iState[heater] -= pid_error[heater]; // conditional un-integration
          pid_output = 0;
        }
      }
    #endif // PID_OPENLOOP

    #if ENABLED(PID_DEBUG)
      ECHO_SMV(DB, SERIAL_PID_DEBUG, h);
      ECHO_MV(SERIAL_PID_DEBUG_INPUT, Heaters[heater].currentTemperatureC);
      ECHO_MV(SERIAL_PID_DEBUG_OUTPUT, pid_output);
      ECHO_MV(SERIAL_PID_DEBUG_PTERM, pTerm[heater]);
      ECHO_MV(SERIAL_PID_DEBUG_ITERM, iTerm[heater]);
      ECHO_MV(SERIAL_PID_DEBUG_DTERM, dTerm[heater]);
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        ECHO_MV(SERIAL_PID_DEBUG_CTERM, cTerm[heater]);
      #endif
      ECHO_E;
    #endif // PID_DEBUG
  }
  else { /* PID off */
    pid_output = (Heaters[heater].currentTemperatureC < Heaters[heater].targetTemperatureC) ? Heaters[heater].pid_max : 0;
  }

  return pid_output;
}

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

  #if ENABLED(HEATER_0_USES_MAX6675)
    float ct = Heaters[0].currentTemperatureC;
    if (ct > min(HEATER_0_MAXTEMP, 1023)) max_temp_error(0);
    if (ct < max(HEATER_0_MINTEMP, 0.01)) min_temp_error(0);
  #endif

  #if ENABLED(THERMAL_PROTECTION_HOTENDS) || DISABLED(PIDTEMPBED) || HAS(AUTO_FAN)
    millis_t ms = millis();
  #endif

  // Loop through all heaters
  for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {

    #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      thermal_runaway_protection(&thermal_runaway_state_machine[heater], &thermal_runaway_timer[heater], Heaters[heater].currentTemperatureC, Heaters[heater].targetTemperatureC, h, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
    #endif

    float pid_output = get_pid_output(heater);

    // Check if temperature is within the correct range
    Heaters[heater].soft_pwm = Heaters[heater].currentTemperatureC > Heaters[heater].minttempC && Heaters[heater].currentTemperatureC < Heaters[heater].maxttempC ? (int)pid_output >> 1 : 0;

    // Check if the temperature is failing to increase
    #if ENABLED(THERMAL_PROTECTION_HEATERS)
      // Is it time to check this extruder's heater?
      if (watch_heater_next_ms[heater] && ms > watch_heater_next_ms[heater]) {
        // Has it failed to increase enough?
        if (Heaters[heater].currentTemperatureC < watch_target_temp[heater]) {
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
      checkHeaterAutoFans();
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
}

void updatePID() {
  for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
    temp_iState_max[heater] = PID_INTEGRAL_DRIVE_MAX / Heaters[heater].Ki;
  }
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    for (uint8_t e = 0; e < EXTRUDERS; e++) last_position[e] = 0;
  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues() {
  static millis_t last_update = millis();
  millis_t temp_last_update = millis();
  millis_t from_last_update = temp_last_update - last_update;
  #if ENABLED(HEATER_0_USES_MAX6675)
    Heaters[0].currentTemperature_raw = read_max6675();
  #endif

  for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
    Heaters[heater].currentTemperatureC = Heaters[heater].analog2temp();
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

  // Finish init of mult heaters arrays
  for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
    // populate with the first value
    temp_iState_min[heater] = 0.0;
    temp_iState_max[heater] = PID_INTEGRAL_DRIVE_MAX / Heaters[heater].Ki;
    SET_OUTPUT(Heaters[heater].heater_pin);
  }

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    for (uint8_t e = 0; e < EXTRUDERS; e++) last_position[e] = 0;
  #endif

  #if HAS(FAN)
    SET_OUTPUT(FAN_PIN);
    #if ENABLED(FAST_PWM_FAN)
      setPwmFrequency(FAN_PIN, 1); // No prescaling. Pwm frequency = F_CPU/256/8
    #endif
    #if ENABLED(FAN_SOFT_PWM)
      soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif

  #if ENABLED(HEATER_0_USES_MAX6675)

    #if DISABLED(SDSUPPORT)
      WRITE(SCK_PIN, 0);
      SET_OUTPUT(SCK_PIN);
      WRITE(MOSI_PIN, 1);
      SET_OUTPUT(MOSI_PIN);
      WRITE(MISO_PIN, 1);
      SET_INPUT(MISO_PIN);
    #endif

    OUT_WRITE(MAX6675_SS, HIGH);

  #endif // HEATER_0_USES_MAX6675

  // Set analog inputs
  // Setup channels
  ADC->ADC_MR |= ADC_MR_FREERUN_ON |
               ADC_MR_LOWRES_BITS_12;

  for (uint8_t heater = 0; heater < NUM_HEATER; heater++)
    startAdcConversion(pinToAdcChannel(Heaters[heater].sensor_pin));

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt

  HAL_temp_timer_start(TEMP_TIMER_NUM);
  HAL_timer_enable_interrupt (TEMP_TIMER_NUM);

  // Wait for temperature measurement to settle
  HAL::delayMilliseconds(250);
}

#if ENABLED(THERMAL_PROTECTION_HEATERS)
  /**
   * Start Heating Sanity Check for heaters that are below
   * their target temperature by a configurable margin.
   * This is called when the temperature is set. (M104, M109)
   */
  void start_watching_heater(uint8_t heater) {
    if (Heaters[heater].currentTemperatureC < Heaters[heater].targetTemperatureC - (WATCH_TEMP_INCREASE + TEMP_HYSTERESIS + 1)) {
      watch_target_temp[heater] = Heaters[heater].currentTemperatureC + WATCH_TEMP_INCREASE;
      watch_heater_next_ms[heater] = millis() + WATCH_TEMP_PERIOD * 1000UL;
    }
    else
      watch_heater_next_ms[heater] = 0;
  }

  void thermal_runaway_protection(TRState* state, millis_t* timer, float temperature, float targetTemperatureC, int heater_id, int period_seconds, int hysteresis_degc) {

    static float tr_target_temperature[NUM_HEATER + 1] = { 0.0 };

    /*
        ECHO_SM(DB, "Thermal Thermal Runaway Running. Heater ID: ");
        if (heater_id < 0) ECHO_M("bed"); else ECHO_V(heater_id);
        ECHO_MV(" ;  State:", *state);
        ECHO_MV(" ;  Timer:", *timer);
        ECHO_MV(" ;  Temperature:", temperature);
        ECHO_EMV(" ;  Target Temp:", targetTemperatureC);
    */

    int heater_index = heater_id >= 0 ? heater_id : NUM_HEATER;

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
  for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
    setTargetCelsius(0, heater);
    Heaters[heater].soft_pwm = 0;
    WRITE_HEATER(Heaters[heater].heater_pin, LOW);
  }
}

#if ENABLED(HEATER_0_USES_MAX6675)
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
#endif // HEATER_0_USES_MAX6675

/**
 * Stages in the ISR loop
 */
enum TempState {
  MeasureTemp_0,
  MeasureTemp_1,
  MeasureTemp_2,
  MeasureTemp_3,
  MeasureTemp_4,
  MeasureTemp_5,
  Prepare_FILWIDTH,
  Measure_FILWIDTH,
  Prepare_POWCONSUMPTION,
  Measure_POWCONSUMPTION,
  StartupDelay // Startup, delay initial temp reading a tiny bit so the hardware can settle
};


void Update_heater() {
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static uint32_t raw_temp_value[NUM_HEATER];
  static TempState temp_state = StartupDelay;
  static unsigned char pwm_count = _BV(SOFT_PWM_SCALE);
  static int max_temp[NUM_HEATER];
  static int min_temp[NUM_HEATER];
  static int temp_read = 0;

  static unsigned char median_counter = 0;
  static unsigned long raw_median_temp[NUM_HEATER][MEDIAN_COUNT];
  static bool first_start = true;

  // Static members for each heater
  #if ENABLED(SLOW_PWM_HEATERS)
    static unsigned char slow_pwm_count = 0;
    static unsigned char soft_pwm[NUM_HEATER];
    static unsigned char state_heater[NUM_HEATER];
    static unsigned char state_timer_heater[NUM_HEATER];
  #else
    static unsigned char soft_pwm[NUM_HEATER];
  #endif

  #if HAS(FILAMENT_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  // Initialize some variables only at start!
  if (first_start) {
	  for (uint8_t heater = 0; heater < HOTENDS; heater++) {
      max_temp[heater] = 0;
      min_temp[heater] = 123000;
		  for (int j = 0; j < MEDIAN_COUNT; j++) raw_median_temp[heater][j] = 3600 * OVERSAMPLENR;
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
      for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
        soft_pwm[heater] = Heaters[heater].soft_pwm;
        WRITE_HEATER(Heaters[heater].heater_pin, soft_pwm[heater] > 0 ? 1 : 0);
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
          for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
            if (Heaters[heater].has_fan)
              WRITE_FAN(Heaters[heater].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
          }
        #endif
      #endif
    }

    for (uint8_t heater = 0; heater < NUM_HEATER; heater++)
      if (soft_pwm[heater] < pwm_count) WRITE_HEATER(Heaters[heater].heater_pin, 0);

    #if ENABLED(FAN_SOFT_PWM)
      if (soft_pwm_fan < pwm_count) WRITE_FAN(0);
      #if HAS(CONTROLLERFAN)
        if (soft_pwm_fan_controller < pwm_count) WRITE_FAN(CONTROLLERFAN_PIN, 0);
      #endif
      #if HAS(AUTO_FAN)
        if (soft_pwm_fan_auto < pwm_count) {
          for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
            if (Heaters[heater].has_fan)
              WRITE_FAN(Heaters[heater].fan_pin, 0);
          }
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
      for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
        soft_pwm[heater] = Heaters[heater].soft_pwm;
        if (soft_pwm[heater] > 0) {
          if (state_timer_heater[heater] == 0) {
            if (state_heater[heater] == 0) state_timer_heater[heater] = MIN_STATE_TIME;
            state_heater[heater] = 1;
            WRITE_HEATER(Heaters[heater].heater_pin, 1);
          }
        }
        else {
          if (state_timer_heater[heater] == 0) {
            if (state_heater[heater] == 1) state_timer_heater[heater] = MIN_STATE_TIME;
            state_heater[heater] = 0;
            WRITE_HEATER(Heaters[heater].heater_pin, 0);
          }
        }
      }
    } // slow_pwm_count == 0

    for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
      if (soft_pwm[heater] < slow_pwm_count) {
        if (state_timer_heater[heater] == 0) {
          if (state_heater[heater] == 1) state_timer_heater[heater] = MIN_STATE_TIME;
          state_heater[heater] = 0;
          WRITE_HEATER(Heaters[heater].heater_pin, 0);
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
          for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
            if (Heaters[heater].has_fan)
              WRITE_FAN(Heaters[heater].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
          }
        #endif
      }

      if (soft_pwm_fan < pwm_count) WRITE_FAN(0);
      #if HAS(CONTROLLERFAN)
        if (soft_pwm_fan_controller < pwm_count) WRITE_FAN(CONTROLLERFAN_PIN, 0);
      #endif
      #if HAS(AUTO_FAN)
        if (soft_pwm_fan_auto < pwm_count) {
          for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
            if (Heaters[heater].has_fan)
              WRITE_FAN(Heaters[heater].fan_pin, soft_pwm_fan_auto > 0 ? 1 : 0);
          }
        }
      #endif
    #endif // FAN_SOFT_PWM

    pwm_count += _BV(SOFT_PWM_SCALE);
    pwm_count &= 0x7f;

    // increment slow_pwm_count only every 64 pwm_count circa 65.5ms
    if ((pwm_count % 64) == 0) {
      slow_pwm_count++;
      slow_pwm_count &= 0x7f;

      for (uint8_t heater = 0; heater < NUM_HEATER; heater++)
        if (state_timer_heater[heater] > 0) state_timer_heater[heater]--;
    } // (pwm_count % 64) == 0

  #endif // SLOW_PWM_HEATERS

  #define READ_TEMP(temp_id) temp_read = getAdcFreerun(pinToAdcChannel(Heaters[temp_id].sensor_pin)); \
    raw_temp_value[temp_id] += temp_read; \
    max_temp[temp_id] = max(max_temp[temp_id], temp_read); \
    min_temp[temp_id] = min(min_temp[temp_id], temp_read)

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
      temp_state = MeasureTemp_4;
      break;
    case MeasureTemp_4:
      #if HAS(TEMP_4)
        READ_TEMP(4);
      #endif
      temp_state = MeasureTemp_5;
      break;
    case MeasureTemp_5:
      #if HAS(TEMP_5)
        READ_TEMP(5);
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
    Heaters[temp_id].currentTemperature_raw = (sum / MEDIAN_COUNT + 4) >> 2
      
  #define SET_CURRENT_BED_RAW(temp_id) raw_median_temp_bed[temp_id][median_counter] = (raw_temp_bed_value[temp_id] - (min_temp_bed[temp_id] + max_temp_bed[temp_id])); \
    sum = 0; \
    for(int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp_bed[temp_id][i]; \
    Beds[temp_id].currentTemperature_raw = (sum / MEDIAN_COUNT + 4) >> 2

  if (temp_count >= OVERSAMPLENR + 2) { // 14 * 16 * 1/(16000000/64/256)  = 164ms.
    if (!temp_meas_ready) { //Only update the raw values if they have been read. Else we could be updating them during reading.
      unsigned long sum = 0;
      for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
        raw_median_temp[heater][median_counter] = (raw_temp_value[heater] - (min_temp[heater] + max_temp[heater]));
        sum = 0;
        for (int i = 0; i < MEDIAN_COUNT; i++) sum += raw_median_temp[heater][i];
        Heaters[heater].currentTemperature_raw = (sum / MEDIAN_COUNT + 4) >> 2;
      }

      //  Reset min/max-holder
      for (uint8_t heater = 0; heater < NUM_HEATER; heater++) {
        max_temp[heater] = 0;
        min_temp[heater] = 123000;
      }
    } //!temp_meas_ready

    updateTemperaturesFromRawValues();

    // Filament Sensor - can be read any time since IIR filtering is used
    #if HAS(FILAMENT_SENSOR)
      current_raw_filwidth = raw_filwidth_value >> 10;  // Divide to get to 0-16384 range since we used 1/128 IIR filter approach
    #endif

    median_counter++;
    if (median_counter >= MEDIAN_COUNT) median_counter = 0;

    temp_meas_ready = true;
    temp_count = 0;
    for (uint8_t heater = 0; heater < NUM_HEATER; heater++) raw_temp_value[heater] = 0;

    #if HAS(POWER_CONSUMPTION_SENSOR)
      raw_powconsumption_value = 0;
    #endif
/*
    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (Heaters[heater].currentTemperatureC < Heaters[heater].minttempC ) min_temp_error(h);
      if (Heaters[heater].currentTemperatureC > Heaters[heater].maxttempC ) max_temp_error(h);
    }*/
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
