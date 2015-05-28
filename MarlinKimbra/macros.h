#ifndef MACROS_H
  #define MACROS_H

  // Macros for bit masks
  #define BIT(b) (1<<(b))
  #define TEST(n,b) (((n)&BIT(b))!=0)
  #define SET_BIT(n,b,value) (n) ^= ((-value)^(n)) & (BIT(b))

  // Macros for maths shortcuts
  #define M_PI 3.1415926536
  #define RADIANS(d) ((d)*M_PI/180.0)
  #define DEGREES(r) ((r)*180.0/M_PI)
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5

  // Macros to contrain values
  #define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
  #define NOMORE(v,n) do{ if (v > n) v = n; }while(0)

  // Macros to support option testing
  #define _CAT(a, ...) a ## __VA_ARGS__
  #define SWITCH_ENABLED_0 0
  #define SWITCH_ENABLED_1 1
  #define SWITCH_ENABLED_  1
  #define ENABLED(b) _CAT(SWITCH_ENABLED_, b)
  #define DISABLED(b) (!_CAT(SWITCH_ENABLED_, b))

  // Macros for check PIN
  #define PIN_EXISTS(PN) (defined(PN##_PIN) && PN##_PIN >= 0)

  /**
   * Shorthand for pin tests, used wherever needed
   */
  #define HAS_TEMP_0 (PIN_EXISTS(TEMP_0) && TEMP_SENSOR_0 != 0 && TEMP_SENSOR_0 != -2)
  #define HAS_TEMP_1 (PIN_EXISTS(TEMP_1) && TEMP_SENSOR_1 != 0)
  #define HAS_TEMP_2 (PIN_EXISTS(TEMP_2) && TEMP_SENSOR_2 != 0)
  #define HAS_TEMP_3 (PIN_EXISTS(TEMP_3) && TEMP_SENSOR_3 != 0)
  #define HAS_TEMP_BED (PIN_EXISTS(TEMP_BED) && TEMP_SENSOR_BED != 0)
  #define HAS_HEATER_0 (PIN_EXISTS(HEATER_0))
  #define HAS_HEATER_1 (PIN_EXISTS(HEATER_1))
  #define HAS_HEATER_2 (PIN_EXISTS(HEATER_2))
  #define HAS_HEATER_3 (PIN_EXISTS(HEATER_3))
  #define HAS_HEATER_BED (PIN_EXISTS(HEATER_BED))
  #define HAS_AUTO_FAN_0 (PIN_EXISTS(EXTRUDER_0_AUTO_FAN))
  #define HAS_AUTO_FAN_1 (PIN_EXISTS(EXTRUDER_1_AUTO_FAN))
  #define HAS_AUTO_FAN_2 (PIN_EXISTS(EXTRUDER_2_AUTO_FAN))
  #define HAS_AUTO_FAN_3 (PIN_EXISTS(EXTRUDER_3_AUTO_FAN))
  #define HAS_AUTO_FAN (HAS_AUTO_FAN_0 || HAS_AUTO_FAN_1 || HAS_AUTO_FAN_2 || HAS_AUTO_FAN_3)
  #define HAS_FAN (PIN_EXISTS(FAN))
  #define HAS_CONTROLLERFAN (PIN_EXISTS(CONTROLLERFAN))
  #define HAS_SERVO_0 (PIN_EXISTS(SERVO0))
  #define HAS_SERVO_1 (PIN_EXISTS(SERVO1))
  #define HAS_SERVO_2 (PIN_EXISTS(SERVO2))
  #define HAS_SERVO_3 (PIN_EXISTS(SERVO3))
  #define HAS_FILAMENT_SENSOR (ENABLED(FILAMENT_SENSOR) && PIN_EXISTS(FILWIDTH))
  #define HAS_POWER_CONSUMPTION_SENSOR (ENABLED(POWER_CONSUMPTION) && PIN_EXISTS(POWER_CONSUMPTION))
  #define HAS_FILRUNOUT (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FILRUNOUT))
  #define HAS_HOME (PIN_EXISTS(HOME))
  #define HAS_KILL (PIN_EXISTS(KILL))
  #define HAS_SUICIDE (PIN_EXISTS(SUICIDE))
  #define HAS_PHOTOGRAPH (PIN_EXISTS(PHOTOGRAPH))
  #define HAS_X_MIN (PIN_EXISTS(X_MIN))
  #define HAS_X_MAX (PIN_EXISTS(X_MAX))
  #define HAS_Y_MIN (PIN_EXISTS(Y_MIN))
  #define HAS_Y_MAX (PIN_EXISTS(Y_MAX))
  #define HAS_Z_MIN (PIN_EXISTS(Z_MIN))
  #define HAS_Z_MAX (PIN_EXISTS(Z_MAX))
  #define HAS_Z2_MIN (PIN_EXISTS(Z2_MIN))
  #define HAS_Z2_MAX (PIN_EXISTS(Z2_MAX))
  #define HAS_Z_PROBE (PIN_EXISTS(Z_PROBE))
  #define HAS_E_MIN (PIN_EXISTS(E_MIN))
  #define HAS_SOLENOID_1 (PIN_EXISTS(SOL1))
  #define HAS_SOLENOID_2 (PIN_EXISTS(SOL2))
  #define HAS_SOLENOID_3 (PIN_EXISTS(SOL3))
  #define HAS_MICROSTEPS (PIN_EXISTS(X_MS1))
  #define HAS_MICROSTEPS_E0 (PIN_EXISTS(E0_MS1))
  #define HAS_MICROSTEPS_E1 (PIN_EXISTS(E1_MS1))
  #define HAS_MICROSTEPS_E2 (PIN_EXISTS(E2_MS1))
  #define HAS_X_ENABLE (PIN_EXISTS(X_ENABLE))
  #define HAS_X2_ENABLE (PIN_EXISTS(X2_ENABLE))
  #define HAS_Y_ENABLE (PIN_EXISTS(Y_ENABLE))
  #define HAS_Y2_ENABLE (PIN_EXISTS(Y2_ENABLE))
  #define HAS_Z_ENABLE (PIN_EXISTS(Z_ENABLE))
  #define HAS_Z2_ENABLE (PIN_EXISTS(Z2_ENABLE))
  #define HAS_E0_ENABLE (PIN_EXISTS(E0_ENABLE))
  #define HAS_E1_ENABLE (PIN_EXISTS(E1_ENABLE))
  #define HAS_E2_ENABLE (PIN_EXISTS(E2_ENABLE))
  #define HAS_E3_ENABLE (PIN_EXISTS(E3_ENABLE))
  #define HAS_X_DIR (PIN_EXISTS(X_DIR))
  #define HAS_X2_DIR (PIN_EXISTS(X2_DIR))
  #define HAS_Y_DIR (PIN_EXISTS(Y_DIR))
  #define HAS_Y2_DIR (PIN_EXISTS(Y2_DIR))
  #define HAS_Z_DIR (PIN_EXISTS(Z_DIR))
  #define HAS_Z2_DIR (PIN_EXISTS(Z2_DIR))
  #define HAS_E0_DIR (PIN_EXISTS(E0_DIR))
  #define HAS_E1_DIR (PIN_EXISTS(E1_DIR))
  #define HAS_E2_DIR (PIN_EXISTS(E2_DIR))
  #define HAS_E3_DIR (PIN_EXISTS(E3_DIR))
  #define HAS_X_STEP (PIN_EXISTS(X_STEP))
  #define HAS_X2_STEP (PIN_EXISTS(X2_STEP))
  #define HAS_Y_STEP (PIN_EXISTS(Y_STEP))
  #define HAS_Y2_STEP (PIN_EXISTS(Y2_STEP))
  #define HAS_Z_STEP (PIN_EXISTS(Z_STEP))
  #define HAS_Z2_STEP (PIN_EXISTS(Z2_STEP))
  #define HAS_E0_STEP (PIN_EXISTS(E0_STEP))
  #define HAS_E1_STEP (PIN_EXISTS(E1_STEP))
  #define HAS_E2_STEP (PIN_EXISTS(E2_STEP))
  #define HAS_E3_STEP (PIN_EXISTS(E3_STEP))
  #define HAS_E0E1 (PIN_EXISTS(E0E1_CHOICE))
  #define HAS_E0E2 (PIN_EXISTS(E0E2_CHOICE))
  #define HAS_E0E3 (PIN_EXISTS(E0E3_CHOICE))
  #define HAS_E0E4 (PIN_EXISTS(E0E4_CHOICE))
  #define HAS_E1E3 (PIN_EXISTS(E1E3_CHOICE))
  #define HAS_BTN_BACK (PIN_EXISTS(BTN_BACK))

  /**
   * Shorthand for filament sensor and power sensor for ultralcd.cpp, dogm_lcd_implementation.h, ultralcd_implementation_hitachi_HD44780.h
   */
  #define HAS_LCD_FILAMENT_SENSOR (HAS_FILAMENT_SENSOR && defined(FILAMENT_LCD_DISPLAY))
  #define HAS_LCD_POWER_SENSOR (HAS_POWER_CONSUMPTION_SENSOR && defined(POWER_CONSUMPTION_LCD_DISPLAY))

#endif //__MACROS_H
