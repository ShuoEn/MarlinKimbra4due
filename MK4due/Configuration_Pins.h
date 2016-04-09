#ifndef CONFIGURATION_PINS_H
#define CONFIGURATION_PINS_H

//=================================== BASIC ==================================

// X axis pins
#define X_STEP_PIN      ORIG_X_STEP_PIN
#define X_DIR_PIN       ORIG_X_DIR_PIN
#define X_ENABLE_PIN    ORIG_X_ENABLE_PIN

// Y axis pins
#define Y_STEP_PIN      ORIG_Y_STEP_PIN
#define Y_DIR_PIN       ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN    ORIG_Y_ENABLE_PIN

// Z axis pins
#define Z_STEP_PIN      ORIG_Z_STEP_PIN
#define Z_DIR_PIN       ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN    ORIG_Z_ENABLE_PIN

// E axis pins
#if DRIVER_EXTRUDERS > 0
  #define E0_STEP_PIN   ORIG_E0_STEP_PIN
  #define E0_DIR_PIN    ORIG_E0_DIR_PIN
  #define E0_ENABLE_PIN ORIG_E0_ENABLE_PIN
#endif

#if DRIVER_EXTRUDERS > 1
  #define E1_STEP_PIN   ORIG_E1_STEP_PIN
  #define E1_DIR_PIN    ORIG_E1_DIR_PIN
  #define E1_ENABLE_PIN ORIG_E1_ENABLE_PIN
#endif

#if DRIVER_EXTRUDERS > 2
  #define E2_STEP_PIN   ORIG_E2_STEP_PIN
  #define E2_DIR_PIN    ORIG_E2_DIR_PIN
  #define E2_ENABLE_PIN ORIG_E2_ENABLE_PIN
#endif

#if DRIVER_EXTRUDERS > 3
  #define E3_STEP_PIN   ORIG_E3_STEP_PIN
  #define E3_DIR_PIN    ORIG_E3_DIR_PIN
  #define E3_ENABLE_PIN ORIG_E3_ENABLE_PIN
#endif

#if DRIVER_EXTRUDERS > 4
  #define E4_STEP_PIN   ORIG_E4_STEP_PIN
  #define E4_DIR_PIN    ORIG_E4_DIR_PIN
  #define E4_ENABLE_PIN ORIG_E4_ENABLE_PIN
#endif

#if DRIVER_EXTRUDERS > 5
  #define E5_STEP_PIN   ORIG_E5_STEP_PIN
  #define E5_DIR_PIN    ORIG_E5_DIR_PIN
  #define E5_ENABLE_PIN ORIG_E5_ENABLE_PIN
#endif

// ENDSTOP pin
#define X_MIN_PIN       ORIG_X_MIN_PIN
#define X_MAX_PIN       ORIG_X_MAX_PIN
#define Y_MIN_PIN       ORIG_Y_MIN_PIN
#define Y_MAX_PIN       ORIG_Y_MAX_PIN
#define Z_MIN_PIN       ORIG_Z_MIN_PIN
#define Z_MAX_PIN       ORIG_Z_MAX_PIN

// HEATER pin
#define HOTEND_0_PIN    ORIG_HOTEND_0_PIN
#define HOTEND_1_PIN    ORIG_HOTEND_1_PIN
#define HOTEND_2_PIN    ORIG_HOTEND_2_PIN
#define HOTEND_3_PIN    ORIG_HOTEND_3_PIN
#define BED_0_PIN       ORIG_BED_0_PIN
#define BED_1_PIN       ORIG_BED_1_PIN
#define BED_2_PIN       ORIG_BED_2_PIN
#define BED_3_PIN       ORIG_BED_3_PIN

// TEMP pin
#define TEMP_0_PIN      ORIG_TEMP_0_PIN
#define TEMP_1_PIN      ORIG_TEMP_1_PIN
#define TEMP_2_PIN      ORIG_TEMP_2_PIN
#define TEMP_3_PIN      ORIG_TEMP_3_PIN
#define TEMP_BED_0_PIN  ORIG_TEMP_BED_0_PIN
#define TEMP_BED_1_PIN  ORIG_TEMP_BED_1_PIN
#define TEMP_BED_2_PIN  ORIG_TEMP_BED_2_PIN
#define TEMP_BED_3_PIN  ORIG_TEMP_BED_3_PIN

// FAN pin
#define FAN_PIN         -1

// PS ON pin
#define PS_ON_PIN       ORIG_PS_ON_PIN

// BEEPER pin
#define BEEPER_PIN      ORIG_BEEPER_PIN

//============================================================================

//================================= FEATURE ==================================

#if ENABLED(MKR4)
  #define E0E1_CHOICE_PIN -1
  #define E0E2_CHOICE_PIN -1
  #define E0E3_CHOICE_PIN -1
  #define E1E3_CHOICE_PIN -1
#endif

#if ENABLED(NPR2)
  #define E_MIN_PIN -1
#endif

#if ENABLED(LASERBEAM)
  #define LASER_PWR_PIN -1
  #define LASER_TTL_PIN -1
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FILRUNOUT_PIN -1
#endif

#if ENABLED(FILAMENT_SENSOR)
  #define FILWIDTH_PIN -1
#endif

#if ENABLED(POWER_CONSUMPTION)
  #define POWER_CONSUMPTION_PIN -1
#endif

#if ENABLED(PHOTOGRAPH)
  #define PHOTOGRAPH_PIN -1
#endif

#if ENABLED(CHDK)
  #define CHDK_PIN -1
#endif

#if ENABLED(CONTROLLERFAN)
  #define CONTROLLERFAN_PIN -1
#endif

#define HOTEND_0_AUTO_FAN_PIN ORIG_FAN_PIN
#define HOTEND_1_AUTO_FAN_PIN -1
#define HOTEND_2_AUTO_FAN_PIN -1
#define HOTEND_3_AUTO_FAN_PIN -1

#if ENABLED(X2_IS_TMC)
  #define X2_ENABLE_PIN -1
  #define X2_STEP_PIN   -1
  #define X2_DIR_PIN    -1
#endif

#if ENABLED(Z_PROBE_SLED)
  #define SLED_PIN -1
#endif

#if ENABLED(Z_PROBE_ENDSTOP)
  #define Z_PROBE_PIN -1
#endif

//============================================================================

#endif
