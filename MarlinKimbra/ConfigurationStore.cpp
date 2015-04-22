/**
 * ConfigurationStore.cpp
 *
 * Configuration and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
 * in the functions below, also increment the version number. This makes sure that
 * the default values are used whenever there is a change to the data, to prevent
 * wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same order.
 *       If a feature is disabled, some data must still be written that, when read,
 *       either sets a Sane Default, or results in No Change to the existing value.
 *
 */

#define EEPROM_VERSION "V21"

/**
 * V21 EEPROM Layout:
 *
 *  ver
 *  axis_steps_per_unit (x7)
 *  max_feedrate (x7)
 *  retraction_feedrate (x4)
 *  max_acceleration_units_per_sq_second (x7)
 *  acceleration
 *  retract_acceleration
 *  travel_acceleration
 *  minimumfeedrate
 *  mintravelfeedrate
 *  minsegmenttime
 *  max_xy_jerk
 *  max_z_jerk
 *  max_e_jerk
 *  home_offset (x3)
 *  zprobe_zoffset
 *
 * HOTEND OFFSET:
 *  hotend_offset (x4)
 *
 * DELTA:
 *  endstop_adj (x3)
 *  delta_radius
 *  delta_diagonal_rod
 *  max_pos
 *  tower_adj (x3)
 *  z_probe_offset
 *
 * Z_DUAL_ENDSTOPS
 *  z_endstop_adj
 *
 * ULTIPANEL:
 *  plaPreheatHotendTemp
 *  plaPreheatHPBTemp
 *  plaPreheatFanSpeed
 *  absPreheatHotendTemp
 *  absPreheatHPBTemp
 *  absPreheatFanSpeed
 *  gumPreheatHotendTemp
 *  gumPreheatHPBTemp
 *  gumPreheatFanSpeed
 *
 * PIDTEMP:
 *  Kp[0], Ki[0], Kd[0]
 *  Kp[1], Ki[1], Kd[1]
 *  Kp[2], Ki[2], Kd[2]
 *  Kp[3], Ki[3], Kd[3]
 *
 * PIDTEMPBED:
 *  bedKp, bedKi, bedKd
 *
 * DOGLCD:
 *  lcd_contrast
 *
 * SCARA:
 *  axis_scaling (x3)
 *
 * FWRETRACT:
 *  autoretract_enabled
 *  retract_length
 *  retract_length_swap
 *  retract_feedrate
 *  retract_zlift
 *  retract_recover_length
 *  retract_recover_length_swap
 *  retract_recover_feedrate
 *
 *  volumetric_enabled
 *
 *  filament_size (x4)
 *
 *  idleoozing_enabled
 *
 *
 *
 */
#include "Marlin.h"
#include "language.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "cardreader.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size) {
  uint8_t c;
  while(size--) {
    eeprom_write_byte((unsigned char*)pos, *value);
    c = eeprom_read_byte((unsigned char*)pos);
    if (c != *value) {
      ECHO_LM(ER, MSG_ERR_EEPROM_WRITE);
    }
    pos++;
    value++;
  };
}
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size) {
  do {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  } while (--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

//======================================================================================

#define DUMMY_PID_VALUE 3000.0f

#define EEPROM_OFFSET 100

#ifdef EEPROM_SETTINGS

void Config_StoreSettings() {
  float dummy = 0.0f;
  char ver[4] = "000";
  int i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver); // invalidate data first
  EEPROM_WRITE_VAR(i, axis_steps_per_unit);
  EEPROM_WRITE_VAR(i, max_feedrate);
  EEPROM_WRITE_VAR(i, max_retraction_feedrate);
  EEPROM_WRITE_VAR(i, max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i, acceleration);
  EEPROM_WRITE_VAR(i, retract_acceleration);
  EEPROM_WRITE_VAR(i, travel_acceleration);
  EEPROM_WRITE_VAR(i, minimumfeedrate);
  EEPROM_WRITE_VAR(i, mintravelfeedrate);
  EEPROM_WRITE_VAR(i, minsegmenttime);
  EEPROM_WRITE_VAR(i, max_xy_jerk);
  EEPROM_WRITE_VAR(i, max_z_jerk);
  EEPROM_WRITE_VAR(i, max_e_jerk);
  EEPROM_WRITE_VAR(i, home_offset);

  #ifndef DELTA
    EEPROM_WRITE_VAR(i, zprobe_zoffset);
  #endif

  #if HOTENDS > 1
    EEPROM_WRITE_VAR(i, hotend_offset);
  #endif

  #ifdef DELTA
    EEPROM_WRITE_VAR(i, endstop_adj);
    EEPROM_WRITE_VAR(i, delta_radius);
    EEPROM_WRITE_VAR(i, delta_diagonal_rod);
    EEPROM_WRITE_VAR(i, max_pos);
    EEPROM_WRITE_VAR(i, tower_adj);
    EEPROM_WRITE_VAR(i, z_probe_offset);
  #elif defined(Z_DUAL_ENDSTOPS)
    EEPROM_WRITE_VAR(i, z_endstop_adj);            // 1 floats
  #endif

  #ifndef ULTIPANEL
    int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
    int gumPreheatHotendTemp = GUM_PREHEAT_HOTEND_TEMP, gumPreheatHPBTemp = GUM_PREHEAT_HPB_TEMP, gumPreheatFanSpeed = GUM_PREHEAT_FAN_SPEED;
  #endif

  EEPROM_WRITE_VAR(i, plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i, plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i, plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i, absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i, absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i, absPreheatFanSpeed);
  EEPROM_WRITE_VAR(i, gumPreheatHotendTemp);
  EEPROM_WRITE_VAR(i, gumPreheatHPBTemp);
  EEPROM_WRITE_VAR(i, gumPreheatFanSpeed);

  for (int e = 0; e < 4; e++) {
    #ifdef PIDTEMP
      if (e < HOTENDS) {
        EEPROM_WRITE_VAR(i, PID_PARAM(Kp, e));
        EEPROM_WRITE_VAR(i, PID_PARAM(Ki, e));
        EEPROM_WRITE_VAR(i, PID_PARAM(Kd, e));
      }
      else
    #endif // !PIDTEMP
      {
        dummy = DUMMY_PID_VALUE; // When read, will not change the existing value
        EEPROM_WRITE_VAR(i, dummy);
        dummy = 0.0f;
        for (int q = 3; q--;) EEPROM_WRITE_VAR(i, dummy);
      }

  } // Extruders Loop

  #ifndef PIDTEMPBED
    float bedKp = DUMMY_PID_VALUE, bedKi = DUMMY_PID_VALUE, bedKd = DUMMY_PID_VALUE;
  #endif

  EEPROM_WRITE_VAR(i, bedKp);
  EEPROM_WRITE_VAR(i, bedKi);
  EEPROM_WRITE_VAR(i, bedKd);

  #if !defined(DOGLCD) || LCD_CONTRAST < 0
    int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i, lcd_contrast);

  #ifdef SCARA
    EEPROM_WRITE_VAR(i, axis_scaling); // 3 floats
  #else
    dummy = 1.0f;
    EEPROM_WRITE_VAR(i, dummy);
  #endif

  #ifdef FWRETRACT
    EEPROM_WRITE_VAR(i, autoretract_enabled);
    EEPROM_WRITE_VAR(i, retract_length);
    #if EXTRUDERS > 1
      EEPROM_WRITE_VAR(i, retract_length_swap);
    #else
      dummy = 0.0f;
      EEPROM_WRITE_VAR(i, dummy);
    #endif
    EEPROM_WRITE_VAR(i, retract_feedrate);
    EEPROM_WRITE_VAR(i, retract_zlift);
    EEPROM_WRITE_VAR(i, retract_recover_length);
    #if EXTRUDERS > 1
      EEPROM_WRITE_VAR(i, retract_recover_length_swap);
    #else
      dummy = 0.0f;
      EEPROM_WRITE_VAR(i, dummy);
    #endif
    EEPROM_WRITE_VAR(i, retract_recover_feedrate);
  #endif // FWRETRACT

  EEPROM_WRITE_VAR(i, volumetric_enabled);

  // Save filament sizes
  for (int q = 0; q < 4; q++) {
    if (q < EXTRUDERS) dummy = filament_size[q];
    EEPROM_WRITE_VAR(i, dummy);
  }
  
  #ifdef IDLE_OOZING_PREVENT
    EEPROM_WRITE_VAR(i, idleoozing_enabled);
  #endif

  char ver2[4] = EEPROM_VERSION;
  int j = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(j, ver2); // validate data

  // Report storage size
  ECHO_SMV(OK, "Settings Stored (", i);
  ECHO_EM(" bytes)");
}

void Config_RetrieveSettings() {

  int i = EEPROM_OFFSET;
  char stored_ver[4];
  char ver[4] = EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver); //read stored version

  if (strncmp(ver, stored_ver, 3) != 0) {
    Config_ResetDefault();
  }
  else {
    float dummy = 0;

    // version number match
    EEPROM_READ_VAR(i, axis_steps_per_unit);
    EEPROM_READ_VAR(i, max_feedrate);
    EEPROM_READ_VAR(i, max_retraction_feedrate);
    EEPROM_READ_VAR(i, max_acceleration_units_per_sq_second);

    // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
    reset_acceleration_rates();

    EEPROM_READ_VAR(i, acceleration);
    EEPROM_READ_VAR(i, retract_acceleration);
    EEPROM_READ_VAR(i, travel_acceleration);
    EEPROM_READ_VAR(i, minimumfeedrate);
    EEPROM_READ_VAR(i, mintravelfeedrate);
    EEPROM_READ_VAR(i, minsegmenttime);
    EEPROM_READ_VAR(i, max_xy_jerk);
    EEPROM_READ_VAR(i, max_z_jerk);
    EEPROM_READ_VAR(i, max_e_jerk);
    EEPROM_READ_VAR(i, home_offset);

    #ifndef DELTA
      EEPROM_READ_VAR(i, zprobe_zoffset);
    #endif

    #if HOTENDS > 1
      EEPROM_READ_VAR(i, hotend_offset);
    #endif

    #ifdef DELTA
      EEPROM_READ_VAR(i, endstop_adj);
      EEPROM_READ_VAR(i, delta_radius);
      EEPROM_READ_VAR(i, delta_diagonal_rod);
      EEPROM_READ_VAR(i, max_pos);
      EEPROM_READ_VAR(i, tower_adj);
      EEPROM_READ_VAR(i, z_probe_offset);
      // Update delta constants for updated delta_radius & tower_adj values
      set_delta_constants();
    #endif //DELTA

    #ifndef ULTIPANEL
      int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed,
          absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed,
          gumPreheatHotendTemp, gumPreheatHPBTemp, gumPreheatFanSpeed;
    #endif

    EEPROM_READ_VAR(i, plaPreheatHotendTemp);
    EEPROM_READ_VAR(i, plaPreheatHPBTemp);
    EEPROM_READ_VAR(i, plaPreheatFanSpeed);
    EEPROM_READ_VAR(i, absPreheatHotendTemp);
    EEPROM_READ_VAR(i, absPreheatHPBTemp);
    EEPROM_READ_VAR(i, absPreheatFanSpeed);
    EEPROM_READ_VAR(i, gumPreheatHotendTemp);
    EEPROM_READ_VAR(i, gumPreheatHPBTemp);
    EEPROM_READ_VAR(i, gumPreheatFanSpeed);

    #ifdef PIDTEMP
      for (int e = 0; e < 4; e++) { // 4 = max hotend currently supported
        EEPROM_READ_VAR(i, dummy); // Kp
        if (e < EXTRUDERS && dummy != DUMMY_PID_VALUE) {
          // do not need to scale PID values as the values in EEPROM are already scaled
          PID_PARAM(Kp, e) = dummy;
          EEPROM_READ_VAR(i, PID_PARAM(Ki, e));
          EEPROM_READ_VAR(i, PID_PARAM(Kd, e));
        }
        else {
          for (int q=3; q--;) EEPROM_READ_VAR(i, dummy); // Ki, Kd, Kc
        }
      }
    #else // !PIDTEMP
      // 4 x 3 = 12 slots for PID parameters
      for (int q = 12; q--;) EEPROM_READ_VAR(i, dummy);  // 4x Kp, Ki, Kd
    #endif // !PIDTEMP

    #ifndef PIDTEMPBED
      float bedKp, bedKi, bedKd;
    #endif

    EEPROM_READ_VAR(i, dummy); // bedKp
    if (dummy != DUMMY_PID_VALUE) {
      bedKp = dummy;
      EEPROM_READ_VAR(i, bedKi);
      EEPROM_READ_VAR(i, bedKd);
    }
    else {
      for (int q = 2; q--;) EEPROM_READ_VAR(i, dummy); // bedKi, bedKd
    }

    #if !defined(DOGLCD) || LCD_CONTRAST < 0
      int lcd_contrast;
    #endif //DOGLCD

    EEPROM_READ_VAR(i, lcd_contrast);

    #ifdef SCARA
      EEPROM_READ_VAR(i, axis_scaling);  // 3 floats
    #else
      EEPROM_READ_VAR(i, dummy);
    #endif

    #ifdef FWRETRACT
      EEPROM_READ_VAR(i, autoretract_enabled);
      EEPROM_READ_VAR(i, retract_length);
      #if EXTRUDERS > 1
        EEPROM_READ_VAR(i, retract_length_swap);
      #else
        EEPROM_READ_VAR(i, dummy);
      #endif
      EEPROM_READ_VAR(i, retract_feedrate);
      EEPROM_READ_VAR(i, retract_zlift);
      EEPROM_READ_VAR(i, retract_recover_length);
      #if EXTRUDERS > 1
        EEPROM_READ_VAR(i, retract_recover_length_swap);
      #else
        EEPROM_READ_VAR(i, dummy);
      #endif
      EEPROM_READ_VAR(i, retract_recover_feedrate);
    #endif // FWRETRACT

    EEPROM_READ_VAR(i, volumetric_enabled);

    for (int q = 0; q < 4; q++) {
      EEPROM_READ_VAR(i, dummy);
      if (q < EXTRUDERS) filament_size[q] = dummy;
    }

    calculate_volumetric_multipliers();

    #ifdef IDLE_OOZING_PREVENT
      EEPROM_READ_VAR(i, idleoozing_enabled);
    #endif

    // Call updatePID (similar to when we have processed M301)
    updatePID();

    // Report settings retrieved and length
    ECHO_SV(OK, ver);
    ECHO_MV(" stored settings retrieved (", i);
    ECHO_EM(" bytes)");
  }

  #ifdef EEPROM_CHITCHAT
    Config_PrintSettings();
  #endif
}

#endif // EEPROM_SETTINGS

void Config_ResetDefault() {

  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[] = DEFAULT_MAX_FEEDRATE;
  float tmp3[] = DEFAULT_RETRACTION_MAX_FEEDRATE;
  long tmp4[]  = DEFAULT_MAX_ACCELERATION;
  #ifdef PIDTEMP
    float tmp5[] = DEFAULT_Kp;
    float tmp6[] = DEFAULT_Ki;
    float tmp7[] = DEFAULT_Kd;
  #endif // PIDTEMP

  #if defined(HOTEND_OFFSET_X) && defined(HOTEND_OFFSET_Y)
    float tmp8[] = HOTEND_OFFSET_X;
    float tmp9[] = HOTEND_OFFSET_Y;
  #else
    float tmp8[] = {0,0,0,0};
    float tmp9[] = {0,0,0,0};
  #endif

  for (int i = 0; i < 3 + EXTRUDERS; i++) {
    axis_steps_per_unit[i] = tmp1[i];
    max_feedrate[i] = tmp2[i];
    max_acceleration_units_per_sq_second[i] = tmp4[i];
  }

  for (int i = 0; i < EXTRUDERS; i++) {
    max_retraction_feedrate[i] = tmp3[i];
    #if HOTENDS > 1
      hotend_offset[X_AXIS][i] = tmp8[i];
      hotend_offset[Y_AXIS][i] = tmp9[i];
    #endif
    #ifdef SCARA
      if (i < sizeof(axis_scaling) / sizeof(*axis_scaling))
        axis_scaling[i] = 1;
    #endif
  }

  // steps per sq second need to be updated to agree with the units per sq second
  reset_acceleration_rates();

  acceleration = DEFAULT_ACCELERATION;
  retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
  minsegmenttime = DEFAULT_MINSEGMENTTIME;
  mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
  max_xy_jerk = DEFAULT_XYJERK;
  max_z_jerk = DEFAULT_ZJERK;
  max_e_jerk = DEFAULT_EJERK;
  home_offset[X_AXIS] = home_offset[Y_AXIS] = home_offset[Z_AXIS] = 0;

  #ifdef ENABLE_AUTO_BED_LEVELING
    zprobe_zoffset = -Z_PROBE_OFFSET_FROM_EXTRUDER;
  #elif !defined DELTA
    zprobe_zoffset = 0;
  #endif //ENABLE_AUTO_BED_LEVELING

  #ifdef DELTA
    endstop_adj[X_AXIS] = endstop_adj[Y_AXIS] = endstop_adj[Z_AXIS] = 0;
    delta_radius = DEFAULT_DELTA_RADIUS;
    delta_diagonal_rod = DEFAULT_DELTA_DIAGONAL_ROD;
    tower_adj[0] = tower_adj[1] = tower_adj[2] = tower_adj[3] = tower_adj[4] = tower_adj[5] = 0;
    max_pos[2] = MANUAL_Z_HOME_POS;
    set_default_z_probe_offset();
    set_delta_constants();
  #endif //DELTA

  #ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
    gumPreheatHotendTemp = GUM_PREHEAT_HOTEND_TEMP;
    gumPreheatHPBTemp = GUM_PREHEAT_HPB_TEMP;
    gumPreheatFanSpeed = GUM_PREHEAT_FAN_SPEED;
  #endif

  #if defined(DOGLCD) && LCD_CONTRAST >= 0
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif //DOGLCD

  #ifdef PIDTEMP
    for (int e = 0; e < HOTENDS; e++) 
    {
      Kp[e] = tmp5[e];
      Ki[e] = scalePID_i(tmp6[e]);
      Kd[e] = scalePID_d(tmp7[e]);
    }
    // call updatePID (similar to when we have processed M301)
    updatePID();
  #endif//PIDTEMP

  #ifdef FWRETRACT
    autoretract_enabled = false;
    retract_length = RETRACT_LENGTH;
    #if EXTRUDERS > 1
      retract_length_swap = RETRACT_LENGTH_SWAP;
    #endif
    retract_feedrate = RETRACT_FEEDRATE;
    retract_zlift = RETRACT_ZLIFT;
    retract_recover_length = RETRACT_RECOVER_LENGTH;
    #if EXTRUDERS > 1
      retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
    #endif
    retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;
  #endif

  volumetric_enabled = false;
  filament_size[0] = DEFAULT_NOMINAL_FILAMENT_DIA;
  #if EXTRUDERS > 1
    filament_size[1] = DEFAULT_NOMINAL_FILAMENT_DIA;
    #if EXTRUDERS > 2
      filament_size[2] = DEFAULT_NOMINAL_FILAMENT_DIA;
      #if EXTRUDERS > 3
        filament_size[3] = DEFAULT_NOMINAL_FILAMENT_DIA;
      #endif //EXTRUDERS > 3
    #endif //EXTRUDERS > 2
  #endif //EXTRUDERS > 1
  calculate_volumetric_multipliers();

  #ifdef IDLE_OOZING_PREVENT
    idleoozing_enabled = true;
  #endif

  ECHO_LM(OK, "Hardcoded Default Settings Loaded");
}

void ConfigSD_ResetDefault() {
  #ifdef POWER_CONSUMPTION
   power_consumption_hour = 0;
  #endif
  printer_usage_seconds  = 0;
  ECHO_LM(OK, "Hardcoded SD Default Settings Loaded");
}

#ifndef DISABLE_M503

void Config_PrintSettings(bool forReplay) {
  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

  if (!forReplay) {
    ECHO_LM(OK, "Steps per unit:");
  }
  ECHO_SMV(OK, "  M92 X", axis_steps_per_unit[X_AXIS]);
  ECHO_MV(" Y", axis_steps_per_unit[Y_AXIS]);
  ECHO_MV(" Z", axis_steps_per_unit[Z_AXIS]);
  ECHO_MV(" E0 S", axis_steps_per_unit[E_AXIS + 0]);
  #if EXTRUDERS > 1
    ECHO_MV(" E1 S", axis_steps_per_unit[E_AXIS + 1]);
    #if EXTRUDERS > 2
      ECHO_MV(" E2 S", axis_steps_per_unit[E_AXIS + 2]);
      #if EXTRUDERS > 3
        ECHO_MV(" E3 S", axis_steps_per_unit[E_AXIS + 3]);
      #endif //EXTRUDERS > 3
    #endif //EXTRUDERS > 2
  #endif //EXTRUDERS > 1
  ECHO_E;

  #ifdef SCARA
    if (!forReplay) {
      ECHO_LM(OK, "Scaling factors:");
    }
    ECHO_SMV(OK, "  M365 X", axis_scaling[X_AXIS]);
    ECHO_MV(" Y", axis_scaling[Y_AXIS]);
    ECHO_EMV(" Z", axis_scaling[Z_AXIS]);
  #endif // SCARA

  if (!forReplay) {
    ECHO_LM(OK, "Maximum feedrates (mm/s):");
  }
  ECHO_SMV(OK, "  M203 X", max_feedrate[X_AXIS]);
  ECHO_MV(" Y", max_feedrate[Y_AXIS] ); 
  ECHO_MV(" Z", max_feedrate[Z_AXIS] ); 
  ECHO_MV(" E0 S", max_feedrate[E_AXIS + 0]);
  #if EXTRUDERS > 1
    ECHO_MV(" E1 S", max_feedrate[E_AXIS + 1]);
    #if EXTRUDERS > 2
      ECHO_MV(" E2 S", max_feedrate[E_AXIS + 2]);
      #if EXTRUDERS > 3
        ECHO_MV(" E3 S", max_feedrate[E_AXIS + 3]);
      #endif //EXTRUDERS > 3
    #endif //EXTRUDERS > 2
  #endif //EXTRUDERS > 1
  ECHO_E;

  if (!forReplay) {
    ECHO_LM(OK, "Retraction Steps per unit:");
  }
  ECHO_SMV(OK, "  E0 ",max_retraction_feedrate[0]);
  #if EXTRUDERS > 1
    ECHO_MV(" E1 ", max_retraction_feedrate[1]);
    #if EXTRUDERS > 2
      ECHO_MV(" E2 ", max_retraction_feedrate[2]);
      #if EXTRUDERS > 3
        ECHO_MV(" E3 ", max_retraction_feedrate[3]);
      #endif //EXTRUDERS > 3
    #endif //EXTRUDERS > 2
  #endif //EXTRUDERS > 1
  ECHO_E;

  if (!forReplay) {
    ECHO_LM(OK, "Maximum Acceleration (mm/s2):");
  }
  ECHO_SMV(OK, "  M201 X", max_acceleration_units_per_sq_second[X_AXIS] );
  ECHO_MV(" Y", max_acceleration_units_per_sq_second[Y_AXIS] );
  ECHO_MV(" Z", max_acceleration_units_per_sq_second[Z_AXIS] );
  ECHO_MV(" E0 S", max_acceleration_units_per_sq_second[E_AXIS]);
  #if EXTRUDERS > 1
    ECHO_MV(" E1 S", max_acceleration_units_per_sq_second[E_AXIS+1]);
    #if EXTRUDERS > 2
      ECHO_MV(" E2 S", max_acceleration_units_per_sq_second[E_AXIS+2]);
      #if EXTRUDERS > 3
        ECHO_MV(" E3 S", max_acceleration_units_per_sq_second[E_AXIS+3]);
      #endif //EXTRUDERS > 3
    #endif //EXTRUDERS > 2
  #endif //EXTRUDERS > 1
  ECHO_E;
  
  if (!forReplay) {
    ECHO_LM(OK, "Accelerations: P=printing, R=retract and T=travel");
  }
  ECHO_SMV(OK,"  M204 P", acceleration );
  ECHO_MV(" R", retract_acceleration);
  ECHO_EMV(" T", travel_acceleration);

  if (!forReplay) {
    ECHO_LM(OK, "Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
  }
  ECHO_SMV(OK, "  M205 S", minimumfeedrate );
  ECHO_MV(" T", mintravelfeedrate );
  ECHO_MV(" B", minsegmenttime );
  ECHO_MV(" X", max_xy_jerk );
  ECHO_MV(" Z", max_z_jerk);
  ECHO_EMV(" E", max_e_jerk);

  if (!forReplay) {
    ECHO_LM(OK, "Home offset (mm):");
  }
  ECHO_SMV(OK, "  M206 X", home_offset[X_AXIS] );
  ECHO_MV(" Y", home_offset[Y_AXIS] );
  ECHO_EMV(" Z", home_offset[Z_AXIS] );

  #if HOTENDS > 1
    if (!forReplay) {
      ECHO_LM(OK, "Hotend offset (mm):");
    }
    for (int e = 0; e < HOTENDS; e++) {
      ECHO_SMV(OK, "  M218 T", e);
      ECHO_MV(" X", hotend_offset[X_AXIS][e]);
      ECHO_EMV(" Y" ,hotend_offset[Y_AXIS][e]);
    }
  #endif //HOTENDS > 1
  
  #ifdef DELTA
    if (!forReplay) {
      ECHO_LM(OK, "Endstop adjustement (mm):");
    }
    ECHO_SMV(OK, "  M666 X", endstop_adj[X_AXIS] );
    ECHO_MV(" Y", endstop_adj[Y_AXIS] );
    ECHO_EMV(" Z", endstop_adj[Z_AXIS] );
    
    ECHO_SM(OK, "Delta Geometry adjustment:");
    ECHO_MV("  M666 A", tower_adj[0]);
    ECHO_MV(" B", tower_adj[1]);
    ECHO_MV(" C", tower_adj[2]);
    ECHO_MV(" E", tower_adj[3]);
    ECHO_MV(" F", tower_adj[4]);
    ECHO_MV(" G", tower_adj[5]);
    ECHO_MV(" R", delta_radius);
    ECHO_MV(" D", delta_diagonal_rod);
    ECHO_MV(" H", max_pos[2]);
    ECHO_EMV(" P", z_probe_offset[3]);
    
    ECHO_SM(OK, "Tower Positions:");
    ECHO_MV(" Tower1 X:", delta_tower1_x);
    ECHO_MV(" Y:", delta_tower1_y);
    ECHO_MV(" Tower2 X:", delta_tower2_x);
    ECHO_MV(" Y:", delta_tower2_y);
    ECHO_MV(" Tower3 X:", delta_tower3_x);
    ECHO_EMV(" Y:", delta_tower3_y);
    
  #elif defined(Z_DUAL_ENDSTOPS)
    if (!forReplay) {
      ECHO_LM(OK, "Z2 Endstop adjustement (mm):");
    }
    ECHO_LMV(OK, "  M666 Z", z_endstop_adj );
  #elif defined(ENABLE_AUTO_BED_LEVELING)
    if (!forReplay) {
      ECHO_LM(OK, "Z Probe offset (mm)");
    }
    ECHO_LMV(OK, "  M666 P", zprobe_zoffset);
  #endif // DELTA

  #if defined(PIDTEMP) || defined(PIDTEMPBED)
    if (!forReplay) {
      ECHO_LM(OK, "PID settings:");
    }
    #ifdef PIDTEMP
      for (int e = 0; e < HOTENDS; e++) {
        ECHO_SMV(OK, "  M301 E", e);
        ECHO_MV(" P", PID_PARAM(Kp, e));
        ECHO_MV(" I", unscalePID_i(PID_PARAM(Ki, e)));
        ECHO_EMV(" D", unscalePID_d(PID_PARAM(Kd, e)));
    }
    #endif
    #ifdef PIDTEMPBED
      ECHO_SMV("  M304 P", bedKp); // for compatibility with hosts, only echos values for E0
      ECHO_MV(" I", unscalePID_i(bedKi));
      ECHO_EMV(" D", unscalePID_d(bedKd));
    #endif
  #endif

  #ifdef FWRETRACT
    if (!forReplay) {
      ECHO_LM(OK,"Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
    }
    ECHO_SMV(OK, "  M207 S", retract_length);
    ECHO_MV(" F", retract_feedrate*60);
    ECHO_EMV(" Z", retract_zlift);
    
    if (!forReplay) {
      ECHO_LM(OK, "Recover: S=Extra length (mm) F:Speed (mm/m)");
    }
    ECHO_SMV(OK, "  M208 S", retract_recover_length);
    ECHO_MV(" F", retract_recover_feedrate*60);
    
    if (!forReplay) {
      ECHO_LM(OK,"Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
    }
    ECHO_LMV(OK,"  M209 S", autoretract_enabled);

    #if EXTRUDERS > 1
      if (!forReplay) {
        ECHO_LM(OK,"Multi-extruder settings:");
        ECHO_LMV(OK, "   Swap retract length (mm):    ", retract_length_swap);
        ECHO_LMV(OK, "   Swap rec. addl. length (mm): ", retract_recover_length_swap);
      }
    #endif // EXTRUDERS > 1

  #endif // FWRETRACT

  if (volumetric_enabled) {
    if (!forReplay) {
      ECHO_LM(OK, "Filament settings:");
    }
    ECHO_LMV(OK, "  M200 D", filament_size[0]);

    #if EXTRUDERS > 1
      ECHO_LMV(OK, "  M200 T1 D", filament_size[1]);
      #if EXTRUDERS > 2
        ECHO_LMV(OK, "  M200 T2 D", filament_size[2]);
        #if EXTRUDERS > 3
          ECHO_LMV(OK, "  M200 T3 D", filament_size[3]);
        #endif
      #endif
    #endif

  } else {
    if (!forReplay) {
      ECHO_LM(OK, "Filament settings: Disabled");
    }
  }
}

void ConfigSD_PrintSettings(bool forReplay) {
  // Always have this function, even with SD_SETTINGS disabled, the current values will be shown
  #ifdef POWER_CONSUMPTION
    if (!forReplay) {
      ECHO_LM(OK, "Watt/h consumed:");
    }
    ECHO_LVM(OK, power_consumption_hour," W/h");
  #endif
  if (!forReplay) {
    ECHO_LM(OK, "Power on time:");
  }
  ECHO_LVM(OK, printer_usage_seconds, " s");
}

#endif //!DISABLE_M503

/**
 * Configuration on SD card
 *
 * Author: Simone Primarosa
 *
 */

#if defined(SDSUPPORT) && defined(SD_SETTINGS)
  void ConfigSD_StoreSettings() {
    if(!IS_SD_INSERTED || card.isFileOpen() || card.sdprinting) return;
    card.openFile(CFG_SD_FILE, false, true, false);
    char buff[CFG_SD_MAX_VALUE_LEN];
    #ifdef POWER_CONSUMPTION
      ltoa(power_consumption_hour,buff,10);
      card.unparseKeyLine(cfgSD_KEY[SD_CFG_PWR], buff);
    #endif
    ltoa(printer_usage_seconds,buff,10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_TME], buff);
    
    card.closeFile(false);
    config_last_update = millis();
  }
  void ConfigSD_RetrieveSettings(bool addValue) {
    if(!IS_SD_INSERTED || card.isFileOpen() || card.sdprinting || !card.cardOK) return;
    char key[CFG_SD_MAX_KEY_LEN], value[CFG_SD_MAX_VALUE_LEN];
    int k_idx;
    int k_len, v_len;
    
    card.openFile(CFG_SD_FILE, true, true, false);
    while(true) {
      k_len = CFG_SD_MAX_KEY_LEN;
      v_len = CFG_SD_MAX_VALUE_LEN;
      card.parseKeyLine(key, value, k_len, v_len);
      if(k_len == 0 || v_len == 0) break; //no valid key or value founded
      k_idx = ConfigSD_KeyIndex(key);
      if(k_idx == -1) continue;    //unknow key ignore it
      switch(k_idx) {
        #ifdef POWER_CONSUMPTION
        case SD_CFG_PWR: {
          if(addValue) power_consumption_hour += (unsigned long)atol(value);
          else power_consumption_hour = (unsigned long)atol(value);
        }
        break;
        #endif
        case SD_CFG_TME: {
          if(addValue) printer_usage_seconds += (unsigned long)atol(value);
          else printer_usage_seconds = (unsigned long)atol(value);
        }
        break;
      }
    }
    card.closeFile(false);
    config_readed = true;
    config_last_update = millis();
  }
  
  int ConfigSD_KeyIndex(char *key) {    //At the moment a binary search algorithm is used for simplicity, if it will be necessary (Eg. tons of key), an hash search algorithm will be implemented.
    int begin = 0, end = SD_CFG_END - 1, middle, cond;
    while(begin <= end) {
      middle = (begin + end) / 2;
      cond = strcmp(cfgSD_KEY[middle], key);
      if(!cond) return middle;
      else if(cond < 0) begin = middle + 1;
      else end = middle - 1;
    }
    return -1;
  }
#endif
