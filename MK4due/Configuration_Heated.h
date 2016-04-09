#ifndef CONFIGURATION_HEATED_H
  #define CONFIGURATION_HEATED_H

/*
 * This configuration file contains all function for Heated.
 *
 * - Number Hotend
 * - Number Bed
 * - Automatic temperature
 * - Wattage report
 * - Temperature status LEDs
 * - PID Settings - HOTEND
 * - PID Settings - BED
 * - Inverted PINS
 * - Thermal runaway protection
 * - Fan configuration
 * - Mediancount (ONLY FOR DUE)
 */


/***********************************************************************
 ************************** Hotends number *****************************
 ***********************************************************************/
// This defines the number of Hotends (0 - 4)
#define HOTENDS 1


/***********************************************************************
 **************************** Beds number ******************************
 ***********************************************************************/
// This defines the number of Beds (0 - 4)
#define BEDS 0


/*****************************************************************************************************
 ************************************** Thermistor type **********************************************
 *****************************************************************************************************
 *                                                                                                   *
 * 4.7kohm PULLUP!                                                                                   *
 * This is a normal value, if you use a 1k pullup thermistor see below                               *
 * Please choose the one that matches your setup and set to TEMP_SENSOR_.                            *
 *                                                                                                   *
 * Temperature sensor settings (4.7kohm PULLUP):                                                     *
 *  -2 is thermocouple with MAX6675 (only for sensor 0)                                              *
 *  -1 is thermocouple with AD595 or AD597                                                           *
 *   0 is not used                                                                                   *
 *   1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)                                 *
 *   2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)                                        *
 *   3 is Mendel-parts thermistor (4.7k pullup)                                                      *
 *   4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!     *
 *   5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)              *
 *   6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup) *
 *   7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)                                     *
 *  71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)                                     *
 *   8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)                                        *
 *   9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)                                          *
 *  10 is 100k RS thermistor 198-961 (4.7k pullup)                                                   *
 *  11 is 100k beta 3950 1% thermistor (4.7k pullup)                                                 *
 *  12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)       *
 *  13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"          *
 *  20 is the PT100 circuit found in the Ultimainboard V2.x                                          *
 *  40 is the 10k Carel NTC015WH01 or ELIWELL SN8T6A1502 (4.7k pullup)                               *
 *  60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950                                    *
 *                                                                                                   *
 * 1kohm PULLUP!                                                                                     *
 * This is not normal, you would have to have changed out your 4.7k for 1k                           *
 * (but gives greater accuracy and more stable PID)                                                  *
 * Please choose the one that matches your setup.                                                    *
 *                                                                                                   *
 * Temperature sensor settings (1kohm PULLUP):                                                       *
 *  51 is 100k thermistor - EPCOS (1k pullup)                                                        *
 *  52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)                                          *
 *  55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)                *
 *                                                                                                   *
 *  1047 is Pt1000 with 4k7 pullup                                                                   *
 *  1010 is Pt1000 with 1k pullup (non standard)                                                     *
 *  147 is Pt100 with 4k7 pullup                                                                     *
 *  110 is Pt100 with 1k pullup (non standard)                                                       *
 *  998 and 999 are Dummy Tables. ALWAYS read 25°C or DUMMY_THERMISTOR_998_VALUE temperature        *
 *                                                                                                   *
 *****************************************************************************************************/
#define TEMP_SENSOR_0     1
#define TEMP_SENSOR_1     0
#define TEMP_SENSOR_2     0
#define TEMP_SENSOR_3     0
#define TEMP_SENSOR_BED_0 0
#define TEMP_SENSOR_BED_1 0
#define TEMP_SENSOR_BED_2 0
#define TEMP_SENSOR_BED_3 0

//These 2 defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

// Use it for Testing or Development purposes. NEVER for production machine.
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 25

//Show Temperature ADC value
//The M105 command return, besides traditional information, the ADC value read from temperature sensors.
//#define SHOW_TEMP_ADC_VALUES
/*****************************************************************************************************/


/***********************************************************************
 ************************* Temperature limits ***************************
 ***********************************************************************/
// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HOTEND_0_MAXTEMP 275 // (degC)
#define HOTEND_1_MAXTEMP 275 // (degC)
#define HOTEND_2_MAXTEMP 275 // (degC)
#define HOTEND_3_MAXTEMP 275 // (degC)
#define BED_0_MAXTEMP    150 // (degC)
#define BED_1_MAXTEMP    150 // (degC)
#define BED_2_MAXTEMP    150 // (degC)
#define BED_3_MAXTEMP    150 // (degC)

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HOTEND_0_MINTEMP 5 // (degC)
#define HOTEND_1_MINTEMP 5 // (degC)
#define HOTEND_2_MINTEMP 5 // (degC)
#define HOTEND_3_MINTEMP 5 // (degC)
#define BED_0_MINTEMP    5 // (degC)
#define BED_1_MINTEMP    5 // (degC)
#define BED_2_MINTEMP    5 // (degC)
#define BED_3_MINTEMP    5 // (degC)

//Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 190
#define PLA_PREHEAT_HPB_TEMP     60
#define PLA_PREHEAT_FAN_SPEED   255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP    100
#define ABS_PREHEAT_FAN_SPEED   255   // Insert Value between 0 and 255

#define GUM_PREHEAT_HOTEND_TEMP 230
#define GUM_PREHEAT_HPB_TEMP     60
#define GUM_PREHEAT_FAN_SPEED   255   // Insert Value between 0 and 255
/*****************************************************************************************************/

/*****************************************************************************************
 ******************************** Automatic temperature **********************************
 *****************************************************************************************
 *                                                                                       *
 * The hotend target temperature is calculated by all the buffered lines of gcode.       *
 * The maximum buffered steps/sec of the extruder motor is called "se".                  *
 * Start autotemp mode with M109 S<mintemp> B<maxtemp> F<factor>                         *
 * The target temperature is set to mintemp+factor*se[steps/sec] and is limited by       *
 * mintemp and maxtemp. Turn this off by excuting M109 without F*                        *
 * Also, if the temperature is set to a value below mintemp, it will not be changed      *
 * by autotemp.                                                                          *
 * On an Ultimaker, some initial testing worked with M109 S215 B260 F1                   *
 * in the start.gcode                                                                    *
 *                                                                                       *
 *****************************************************************************************/
#define AUTOTEMP
#define AUTOTEMP_OLDWEIGHT 0.98
/*****************************************************************************************/


/***********************************************************************
 ************************* Wattage report ******************************
 ***********************************************************************
 *                                                                     *
 * If you want the M105 heater power reported in watts,                *
 * define the BED_WATTS, and (shared for all hotend) HOTEND_WATTS      *
 *                                                                     *
 ***********************************************************************/
//#define HOTEND_WATTS (12.0*12.0/6.7)  //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)     // P=I^2/R
/***********************************************************************/



/***********************************************************************
 ********************* Temperature status LEDs *************************
 ***********************************************************************
 *                                                                     *
 * Temperature status LEDs that display the hotend and bed             *
 * temperature.                                                        *
 * Otherwise the RED led is on. There is 1C hysteresis.                *
 *                                                                     *
 ***********************************************************************/
//#define TEMP_STAT_LEDS
/***********************************************************************/
 
 
/***********************************************************************
 ********************** PID Settings - HOTEND **************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 *                                                                     *
 ***********************************************************************/
#define BANG_MAX 255       // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX   // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define K1 0.95            // Smoothing factor within the PID
#define MAX_OVERSHOOT_PID_AUTOTUNE 20   // Max valor for overshoot autotune

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
//#define PID_DEBUG        // Sends debug data to the serial port.
//#define PID_OPENLOOP 1   // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
//#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
// If the temperature difference between the target temperature and the actual temperature
// is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#define PID_FUNCTIONAL_RANGE 10         // degC
#define PID_INTEGRAL_DRIVE_MAX PID_MAX  // Limit for the integral term
// this adds an experimental additional term to the heating power, proportional to the extrusion speed.
// if Kc is chosen well, the additional required power due to increased melting should be compensated.
//#define PID_ADD_EXTRUSION_RATE
#define LPQ_MAX_LEN 50

//           HotEnd{HE0,HE1,HE2,HE3}
#define DEFAULT_Kp {40, 40, 40, 40}     // Kp for H0, H1, H2, H3
#define DEFAULT_Ki {07, 07, 07, 07}     // Ki for H0, H1, H2, H3
#define DEFAULT_Kd {60, 60, 60, 60}     // Kd for H0, H1, H2, H3
#define DEFAULT_Kc {100, 100, 100, 100} // heating power = Kc * (e_speed)
/***********************************************************************/


/***********************************************************************
 ************************ PID Settings - BED ***************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPBED.                            *
 * If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis            *
 *                                                                     *
 ***********************************************************************/
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
//#define PIDTEMPBED

//#define BED_LIMIT_SWITCHING
#define BED_HYSTERESIS 2 //only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS (works only if BED_LIMIT_SWITCHING is enabled)
#define BED_CHECK_INTERVAL 5000 //ms between checks in bang-bang control

// This sets the max power delivered to the bed.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

#define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER // limit for the integral term
// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_bedKp 10.00
#define DEFAULT_bedKi .023
#define DEFAULT_bedKd 305.4

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.

//#define PID_BED_DEBUG // Sends debug data to the serial port.
/***********************************************************************/


/********************************************************************************
 ************************ Inverted Heater or Bed PINS ***************************
 ********************************************************************************
 *                                                                              *
 * For inverted logical Heater or Bed pins                                      *
 *                                                                              *
 ********************************************************************************/
//#define INVERTED_HEATER_PINS
//#define INVERTED_BED_PINS


/********************************************************************************
 ************************ Thermal runaway protection ****************************
 ********************************************************************************
 *                                                                              *
 * This protects your printer from damage and fire if a thermistor              *
 * falls out or temperature sensors fail in any way.                            *
 *                                                                              *
 * The issue: If a thermistor falls out or a temperature sensor fails,          *
 * Marlin can no longer sense the actual temperature. Since a                   *
 * disconnected thermistor reads as a low temperature, the firmware             *
 * will keep the heater on.                                                     *
 *                                                                              *
 * The solution: Once the temperature reaches the target, start                 *
 * observing. If the temperature stays too far below the                        *
 * target(hysteresis) for too long, the firmware will halt                      *
 * as a safety precaution.                                                      *
 *                                                                              *
 * Uncomment THERMAL_PROTECTION_HOTENDS to enable this feature for all hotends. *
 * Uncomment THERMAL_PROTECTION_BED to enable this feature for the heated bed.  *
 *                                                                              *
 ********************************************************************************/
//#define THERMAL_PROTECTION_HOTENDS

#define THERMAL_PROTECTION_PERIOD    40     // Seconds
#define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius

// Whenever an M104 or M109 increases the target temperature the firmware will wait for the
// WATCH TEMP PERIOD to expire, and if the temperature hasn't increased by WATCH TEMP INCREASE
// degrees, the machine is halted, requiring a hard reset. This test restarts with any M104/M109,
//but only if the current temperature is far enough below the target for a reliable test.
#define WATCH_TEMP_PERIOD  16               // Seconds
#define WATCH_TEMP_INCREASE 4               // Degrees Celsius

//#define THERMAL_PROTECTION_BED 

#define THERMAL_PROTECTION_BED_PERIOD    20 // Seconds
#define THERMAL_PROTECTION_BED_HYSTERESIS 2 // Degrees Celsius
/********************************************************************************/


/**************************************************************************
 **************************** Fan configuration ***************************
 **************************************************************************/
// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
// Only 8 bit boards
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT PWM SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)
//#define FAN_KICKSTART_TIME 100

// This defines the minimal speed for the main fan, run in PWM mode
// to enable uncomment and set minimal PWM speed for reliable running (1-255)
// if fan speed is [1 - (FAN_MIN_PWM-1)] it is set to FAN_MIN_PWM
//#define FAN_MIN_PWM 50

// This is for controlling a fan to cool down the stepper drivers
// it will turn on when any driver is enabled
// and turn off after the set amount of seconds from last driver being disabled again
// You need to set CONTROLLERFAN_PIN in Configuration_pins.h
//#define CONTROLLERFAN
#define CONTROLLERFAN_SECS       60   // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED     255   // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED   0

// Extruder cooling fans
// Configure fan pin outputs to automatically turn on/off when the associated
// extruder temperature is above/below EXTRUDER AUTO FAN TEMPERATURE.
// Multiple extruders can be assigned to the same pin in which case
// the fan will turn on when any selected extruder is above the threshold.
// You need to set _AUTO_FAN_PIN in Configuration_pins.h
//#define HOTEND_AUTO_FAN
#define HOTEND_0_AUTO_FAN_TEMPERATURE 50
#define HOTEND_1_AUTO_FAN_TEMPERATURE 50
#define HOTEND_2_AUTO_FAN_TEMPERATURE 50
#define HOTEND_3_AUTO_FAN_TEMPERATURE 50
#define HOTEND_AUTO_FAN_SPEED       255  // 255 = full speed
#define HOTEND_AUTO_FAN_MIN_SPEED     0
/**************************************************************************/


/**************************************************************************
 **************************** MEDIAN COUNT ********************************
 **************************************************************************
 *                                                                        *
 * For Smoother temperature                                               *
 * ONLY FOR DUE                                                           *
 **************************************************************************/
#define MEDIAN_COUNT 10
/**************************************************************************/

#endif
