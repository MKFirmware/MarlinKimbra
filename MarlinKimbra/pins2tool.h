//============================================================================
//==================== Change PIN width Configurator Tool ====================
//============================================================================

//X axis pins
#define X_STEP_PIN    ORIG_X_STEP_PIN
#define X_DIR_PIN     ORIG_X_DIR_PIN
#define X_ENABLE_PIN  ORIG_X_ENABLE_PIN

//Y axis pins
#define Y_STEP_PIN    ORIG_Y_STEP_PIN
#define Y_DIR_PIN     ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN  ORIG_Y_ENABLE_PIN

//Z axis pins
#define Z_STEP_PIN    ORIG_Z_STEP_PIN
#define Z_DIR_PIN     ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN  ORIG_Z_ENABLE_PIN

//E axis pins
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

//HEATHER pin
#define HEATER_0_PIN    ORIG_HEATER_0_PIN
#define HEATER_1_PIN    ORIG_HEATER_1_PIN
#define HEATER_2_PIN    ORIG_HEATER_2_PIN
#define HEATER_3_PIN    ORIG_HEATER_3_PIN
#define HEATER_BED_PIN  ORIG_HEATER_BED_PIN

//TEMP pin
#define TEMP_0_PIN      ORIG_TEMP_0_PIN
#define TEMP_1_PIN      ORIG_TEMP_1_PIN
#define TEMP_2_PIN      ORIG_TEMP_2_PIN
#define TEMP_3_PIN      ORIG_TEMP_3_PIN
#define TEMP_BED_PIN    ORIG_TEMP_BED_PIN

//FAN pin
#define FAN_PIN       ORIG_FAN_PIN

//=========================== START YOUR CHANGE ==============================
// Example for change X_MIN_PIN
// #undef X_MIN_PIN
// #define X_MIN_PIN newpin


//============================================================================
