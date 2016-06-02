/**
 * MK & MK4due 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * configuration_store.cpp
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

#include "base.h"

#define EEPROM_VERSION "MKV428"

/**
 * MKV428 EEPROM Layout:
 *
 *  ver
 *  M92   XYZ E0 ...      axis_steps_per_unit X,Y,Z,E0 ... (per extruder)
 *  M203  XYZ E0 ...      max_feedrate X,Y,Z,E0 ... (per extruder)
 *  M201  XYZ E0 ...      max_acceleration_units_per_sq_second X,Y,Z,E0 ... (per extruder)
 *  M204  P               acceleration
 *  M204  R   E0 ...      retract_acceleration (per extruder)
 *  M204  T               travel_acceleration
 *  M205  S               minimumfeedrate
 *  M205  T               mintravelfeedrate
 *  M205  B               minsegmenttime
 *  M205  X               max_xy_jerk
 *  M205  Z               max_z_jerk
 *  M205  E   E0 ...      max_e_jerk (per extruder)
 *  M206  XYZ             home_offset (x3)
 *  M218  T   XY          hotend_offset (x4) (T0..3)
 *  M666  P               zprobe_zoffset
 *
 * HOTENDS AD595:
 *  M595  H OS            Hotend AD595 Offset & Gain
 *
 * DELTA:
 *  M666  XYZ             endstop_adj (x3)
 *  M666  R               delta_radius
 *  M666  D               delta_diagonal_rod
 *  M666  H               Z sw_endstop_max
 *  M666  ABCIJK          tower_adj (x6)
 *  M666  UVW             diagrod_adj (x3)
 *  M666  P XYZ           XYZ probe_offset (x3)
 *
 * Z_DUAL_ENDSTOPS
 *  M666  Z               z_endstop_adj
 *
 * ULTIPANEL:
 *  M145  S0  H           plaPreheatHotendTemp
 *  M145  S0  B           plaPreheatHPBTemp
 *  M145  S0  F           plaPreheatFanSpeed
 *  M145  S1  H           absPreheatHotendTemp
 *  M145  S1  B           absPreheatHPBTemp
 *  M145  S1  F           absPreheatFanSpeed
 *  M145  S2  H           gumPreheatHotendTemp
 *  M145  S2  B           gumPreheatHPBTemp
 *  M145  S2  F           gumPreheatFanSpeed
 *
 * PIDTEMP:
 *  M301  E0  PIDC        Kp[0], Ki[0], Kd[0], Kc[0]
 *  M301  E1  PIDC        Kp[1], Ki[1], Kd[1], Kc[1]
 *  M301  E2  PIDC        Kp[2], Ki[2], Kd[2], Kc[2]
 *  M301  E3  PIDC        Kp[3], Ki[3], Kd[3], Kc[3]
 *  M301  L               lpq_len
 *
 * PIDTEMPBED:
 *  M304      PID         bedKp, bedKi, bedKd
 * PIDTEMPCHAMBER
 *  M305      PID         chamberKp, chamberKi, chamberKd
 * PIDTEMPCOOLER
 *  M306      PID         coolerKp, coolerKi, coolerKd
 *
 * DOGLCD:
 *  M250  C               lcd_contrast
 *
 * SCARA:
 *  M365  XYZ             axis_scaling (x3)
 *
 * FWRETRACT:
 *  M209  S               autoretract_enabled
 *  M207  S               retract_length
 *  M207  W               retract_length_swap
 *  M207  F               retract_feedrate
 *  M207  Z               retract_zlift
 *  M208  S               retract_recover_length
 *  M208  W               retract_recover_length_swap
 *  M208  F               retract_recover_feedrate
 *
 *  M200  D               volumetric_enabled (D>0 makes this enabled)
 *
 *  M200  T D             filament_size (x4) (T0..3)
 *
 *  M???  S               IDLE_OOZING_enabled
 *
 * ALLIGATOR:
 *  M906  XYZ T0-4 E      Motor current
 *
 */

void _EEPROM_writeData(int& pos, uint8_t* value, uint8_t size) {
  uint8_t c;
  while(size--) {
    eeprom_write_byte((unsigned char*)pos, *value);
    c = eeprom_read_byte((unsigned char*)pos);
    if (c != *value) {
      ECHO_LM(ER, SERIAL_ERR_EEPROM_WRITE);
    }
    pos++;
    value++;
  };
}

void _EEPROM_readData(int& pos, uint8_t* value, uint8_t size) {
  do {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  } while (--size);
}

#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

/**
 * Store Configuration Settings - M500
 */

#define EEPROM_OFFSET 100

#if ENABLED(EEPROM_SETTINGS)

void Config_StoreSettings() {
  float dummy = 0.0f;
  char ver[7] = "000000";
  int i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver); // invalidate data first
  EEPROM_WRITE_VAR(i, axis_steps_per_unit);
  EEPROM_WRITE_VAR(i, max_feedrate);
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
  EEPROM_WRITE_VAR(i, hotend_offset);

  #if ENABLED(MESH_BED_LEVELING)
    // Compile time test that sizeof(mbl.z_values) is as expected
    typedef char c_assert[(sizeof(mbl.z_values) == (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS) * sizeof(dummy)) ? 1 : -1];
    EEPROM_WRITE_VAR(i, mbl.active);
    EEPROM_WRITE_VAR(i, mbl.z_offset);
    EEPROM_WRITE_VAR(i, mbl.z_values);
  #endif

  #if !MECH(DELTA)
    EEPROM_WRITE_VAR(i, zprobe_zoffset);
  #endif

  #if HEATER_USES_AD595
    EEPROM_WRITE_VAR(i, ad595_offset);
    EEPROM_WRITE_VAR(i, ad595_gain);
  #endif

  #if MECH(DELTA)
    EEPROM_WRITE_VAR(i, endstop_adj);
    EEPROM_WRITE_VAR(i, delta_radius);
    EEPROM_WRITE_VAR(i, delta_diagonal_rod);
    EEPROM_WRITE_VAR(i, sw_endstop_max);
    EEPROM_WRITE_VAR(i, tower_adj);
    EEPROM_WRITE_VAR(i, diagrod_adj);
    EEPROM_WRITE_VAR(i, z_probe_offset);
  #elif ENABLED(Z_DUAL_ENDSTOPS)
    EEPROM_WRITE_VAR(i, z_endstop_adj);            // 1 floats
  #endif

  #if DISABLED(ULTIPANEL)
    int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED,
        absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED,
        gumPreheatHotendTemp = GUM_PREHEAT_HOTEND_TEMP, gumPreheatHPBTemp = GUM_PREHEAT_HPB_TEMP, gumPreheatFanSpeed = GUM_PREHEAT_FAN_SPEED;
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

  #if ENABLED(PIDTEMP)
    for (int h = 0; h < HOTENDS; h++) {
      EEPROM_WRITE_VAR(i, PID_PARAM(Kp, h));
      EEPROM_WRITE_VAR(i, PID_PARAM(Ki, h));
      EEPROM_WRITE_VAR(i, PID_PARAM(Kd, h));
      EEPROM_WRITE_VAR(i, PID_PARAM(Kc, h));
    }
  #endif

  #if DISABLED(PID_ADD_EXTRUSION_RATE)
    int lpq_len = 20;
  #endif
  EEPROM_WRITE_VAR(i, lpq_len);
  
  #if ENABLED(PIDTEMPBED)
    EEPROM_WRITE_VAR(i, bedKp);
    EEPROM_WRITE_VAR(i, bedKi);
    EEPROM_WRITE_VAR(i, bedKd);
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    EEPROM_WRITE_VAR(i, chamberKp);
    EEPROM_WRITE_VAR(i, chamberKi);
    EEPROM_WRITE_VAR(i, chamberKd);
  #endif

  #if ENABLED(PIDTEMPCOOLER)
    EEPROM_WRITE_VAR(i, coolerKp);
    EEPROM_WRITE_VAR(i, coolerKi);
    EEPROM_WRITE_VAR(i, coolerKd);
  #endif

  #if HASNT(LCD_CONTRAST)
    const int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i, lcd_contrast);

  #if MECH(SCARA)
    EEPROM_WRITE_VAR(i, axis_scaling); // 3 floats
  #endif

  #if ENABLED(FWRETRACT)
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
  for (int e = 0; e < EXTRUDERS; e++)
    EEPROM_WRITE_VAR(i, filament_size[e]);

  #if ENABLED(IDLE_OOZING_PREVENT)
    EEPROM_WRITE_VAR(i, IDLE_OOZING_enabled);
  #endif

  #if MB(ALLIGATOR)
    EEPROM_WRITE_VAR(i, motor_current);
  #endif

  char ver2[7] = EEPROM_VERSION;
  int j = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(j, ver2); // validate data

  // Report storage size
  ECHO_SMV(DB, "Settings Stored (", (unsigned long)i);
  ECHO_EM(" bytes)");
}

/**
 * Retrieve Configuration Settings - M501
 */
void Config_RetrieveSettings() {

  int i = EEPROM_OFFSET;
  char stored_ver[7];
  char ver[7] = EEPROM_VERSION;
  EEPROM_READ_VAR(i, stored_ver); // read stored version
  //ECHO_EM("Version: [" << ver << "] Stored version: [" << stored_ver << "]");

  if (strncmp(ver, stored_ver, 6) != 0) {
    Config_ResetDefault();
  }
  else {
    float dummy = 0;

    // version number match
    EEPROM_READ_VAR(i, axis_steps_per_unit);
    EEPROM_READ_VAR(i, max_feedrate);
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
    EEPROM_READ_VAR(i, hotend_offset);

    #if ENABLED(MESH_BED_LEVELING)
      EPROM_READ_VAR(i, mbl.active);
      EEPROM_READ_VAR(i, mbl.z_offset);
      EEPROM_READ_VAR(i, mbl.z_values);
    #endif

    #if !MECH(DELTA)
      EEPROM_READ_VAR(i, zprobe_zoffset);
    #endif

    #if HEATER_USES_AD595
      EEPROM_READ_VAR(i, ad595_offset);
      EEPROM_READ_VAR(i, ad595_gain);
      for (int8_t h = 0; h < HOTENDS; h++)
        if (ad595_gain[h] == 0) ad595_gain[h] == TEMP_SENSOR_AD595_GAIN;
    #endif

    #if MECH(DELTA)
      EEPROM_READ_VAR(i, endstop_adj);
      EEPROM_READ_VAR(i, delta_radius);
      EEPROM_READ_VAR(i, delta_diagonal_rod);
      EEPROM_READ_VAR(i, sw_endstop_max);
      EEPROM_READ_VAR(i, tower_adj);
      EEPROM_READ_VAR(i, diagrod_adj);
      EEPROM_READ_VAR(i, z_probe_offset);
      // Update delta constants for updated delta_radius & tower_adj values
      set_delta_constants();
    #endif //DELTA

    #if DISABLED(ULTIPANEL)
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

    #if ENABLED(PIDTEMP)
      for (int8_t h = 0; h < HOTENDS; h++) {
        EEPROM_READ_VAR(i, PID_PARAM(Kp, h));
        EEPROM_READ_VAR(i, PID_PARAM(Ki, h));
        EEPROM_READ_VAR(i, PID_PARAM(Kd, h));
        EEPROM_READ_VAR(i, PID_PARAM(Kc, h));
      }
    #endif // PIDTEMP

    #if DISABLED(PID_ADD_EXTRUSION_RATE)
      int lpq_len;
    #endif
    EEPROM_READ_VAR(i, lpq_len);

    #if ENABLED(PIDTEMPBED)
      EEPROM_READ_VAR(i, bedKp);
      EEPROM_READ_VAR(i, bedKi);
      EEPROM_READ_VAR(i, bedKd);
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      EEPROM_READ_VAR(i, chamberKp);
      EEPROM_READ_VAR(i, chamberKi);
      EEPROM_READ_VAR(i, chamberKd);
    #endif

    #if ENABLED(PIDTEMPCOOLER)
      EEPROM_READ_VAR(i, coolerKp);
      EEPROM_READ_VAR(i, coolerKi);
      EEPROM_READ_VAR(i, coolerKd);
    #endif

    #if HASNT(LCD_CONTRAST)
      int lcd_contrast;
    #endif

    EEPROM_READ_VAR(i, lcd_contrast);

    #if MECH(SCARA)
      EEPROM_READ_VAR(i, axis_scaling);  // 3 floats
    #endif

    #if ENABLED(FWRETRACT)
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

    for (int8_t e = 0; e < EXTRUDERS; e++)
      EEPROM_READ_VAR(i, filament_size[e]);

    calculate_volumetric_multipliers();

    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_READ_VAR(i, IDLE_OOZING_enabled);
    #endif

    #if MB(ALLIGATOR)
      EEPROM_READ_VAR(i, motor_current);
    #endif

    // Call updatePID (similar to when we have processed M301)
    updatePID();

    // Report settings retrieved and length
    ECHO_SV(DB, ver);
    ECHO_MV(" stored settings retrieved (", (unsigned long)i);
    ECHO_EM(" bytes)");
  }

  #if ENABLED(EEPROM_CHITCHAT)
    Config_PrintSettings();
  #endif
}

#endif // EEPROM_SETTINGS

/**
 * Reset Configuration Settings - M502
 */
void Config_ResetDefault() {
  float tmp1[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[] = DEFAULT_MAX_FEEDRATE;
  float tmp3[] = DEFAULT_MAX_ACCELERATION;
  float tmp4[] = DEFAULT_RETRACT_ACCELERATION;
  float tmp5[] = DEFAULT_EJERK;
  float tmp6[] = DEFAULT_Kp;
  float tmp7[] = DEFAULT_Ki;
  float tmp8[] = DEFAULT_Kd;
  float tmp9[] = DEFAULT_Kc;

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    float tmp10[] = HOTEND_OFFSET_X;
    float tmp11[] = HOTEND_OFFSET_Y;
    float tmp12[] = HOTEND_OFFSET_Z;
  #endif

  #if MB(ALLIGATOR)
    float tmp13[] = MOTOR_CURRENT;
  #endif

  for (int8_t i = 0; i < 3 + EXTRUDERS; i++) {
    axis_steps_per_unit[i] = tmp1[i];
    max_feedrate[i] = tmp2[i];
    max_acceleration_units_per_sq_second[i] = tmp3[i];
  }

  for (int8_t i = 0; i < EXTRUDERS; i++) {
    retract_acceleration[i] = tmp4[i];
    max_e_jerk[i] = tmp5[i];
  }

  #if MB(ALLIGATOR)
    for (int8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      motor_current[i] = tmp13[i];
  #endif

  for (int8_t i = 0; i < HOTENDS; i++) {
    #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
      hotend_offset[X_AXIS][i] = tmp10[i];
      hotend_offset[Y_AXIS][i] = tmp11[i];
      hotend_offset[Z_AXIS][i] = tmp12[i];
    #else
      hotend_offset[X_AXIS][i] = 0;
      hotend_offset[Y_AXIS][i] = 0;
      hotend_offset[Z_AXIS][i] = 0;
    #endif
  }

  #if MECH(SCARA)
    for (int8_t i = 0; i < NUM_AXIS; i++) {
      if (i < COUNT(axis_scaling))
        axis_scaling[i] = 1;
    }
  #endif

  // steps per sq second need to be updated to agree with the units per sq second
  reset_acceleration_rates();

  acceleration = DEFAULT_ACCELERATION;
  travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
  minsegmenttime = DEFAULT_MINSEGMENTTIME;
  mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
  max_xy_jerk = DEFAULT_XYJERK;
  max_z_jerk = DEFAULT_ZJERK;
  home_offset[X_AXIS] = home_offset[Y_AXIS] = home_offset[Z_AXIS] = 0;

  #if ENABLED(MESH_BED_LEVELING)
    mbl.active = false;
  #endif

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
  #elif !MECH(DELTA)
    zprobe_zoffset = 0;
  #endif

  #if MECH(DELTA)
    delta_radius = DEFAULT_DELTA_RADIUS;
    delta_diagonal_rod = DELTA_DIAGONAL_ROD;
    endstop_adj[0] = TOWER_A_ENDSTOP_ADJ;
    endstop_adj[1] = TOWER_B_ENDSTOP_ADJ;
    endstop_adj[2] = TOWER_C_ENDSTOP_ADJ;
    tower_adj[0] = TOWER_A_POSITION_ADJ;
    tower_adj[1] = TOWER_B_POSITION_ADJ;
    tower_adj[2] = TOWER_C_POSITION_ADJ;
    tower_adj[3] = TOWER_A_RADIUS_ADJ;
    tower_adj[4] = TOWER_B_RADIUS_ADJ;
    tower_adj[5] = TOWER_C_RADIUS_ADJ;
    diagrod_adj[0] = TOWER_A_DIAGROD_ADJ;
    diagrod_adj[1] = TOWER_B_DIAGROD_ADJ;
    diagrod_adj[2] = TOWER_C_DIAGROD_ADJ;
    z_probe_offset[0] = X_PROBE_OFFSET_FROM_EXTRUDER;
    z_probe_offset[1] = Y_PROBE_OFFSET_FROM_EXTRUDER;
    z_probe_offset[2] = Z_PROBE_OFFSET_FROM_EXTRUDER;
    set_delta_constants();
  #endif

  #if ENABLED(ULTIPANEL)
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

  #if HAS(LCD_CONTRAST)
    lcd_contrast = DEFAULT_LCD_CONTRAST;
  #endif

  #if ENABLED(PIDTEMP)
    for (int8_t h = 0; h < HOTENDS; h++) {
      Kp[h] = tmp6[h];
      Ki[h] = scalePID_i(tmp7[h]);
      Kd[h] = scalePID_d(tmp8[h]);
      Kc[h] = tmp9[h];
    }
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      lpq_len = 20; // default last-position-queue size
    #endif
    // call updatePID (similar to when we have processed M301)
    updatePID();
  #endif // PIDTEMP

  #if ENABLED(PIDTEMPBED)
    bedKp = DEFAULT_bedKp;
    bedKi = scalePID_i(DEFAULT_bedKi);
    bedKd = scalePID_d(DEFAULT_bedKd);
  #endif

  #if ENABLED(PIDTEMPCHAMBER)
    chamberKp = DEFAULT_chamberKp;
    chamberKi = scalePID_i(DEFAULT_chamberKi);
    chamberKd = scalePID_d(DEFAULT_chamberKd);
  #endif

  #if ENABLED(PIDTEMPCOOLER)
    coolerKp = DEFAULT_coolerKp;
    coolerKi = scalePID_i(DEFAULT_coolerKi);
    coolerKd = scalePID_d(DEFAULT_coolerKd);
  #endif

  #if ENABLED(FWRETRACT)
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
  calculate_volumetric_multipliers();

  #if ENABLED(IDLE_OOZING_PREVENT)
    IDLE_OOZING_enabled = true;
  #endif

  ECHO_LM(DB, "Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

  /**
   * Print Configuration Settings - M503
   */
  void Config_PrintSettings(bool forReplay) {
    // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

    if (!forReplay) {
      ECHO_LM(CFG, "Steps per unit:");
    }
    ECHO_SMV(CFG, "  M92 X", axis_steps_per_unit[X_AXIS]);
    ECHO_MV(" Y", axis_steps_per_unit[Y_AXIS]);
    ECHO_MV(" Z", axis_steps_per_unit[Z_AXIS]);
    ECHO_EMV(" E", axis_steps_per_unit[E_AXIS]);
    #if EXTRUDERS > 1
      for (short i = 1; i < EXTRUDERS; i++) {
        ECHO_SMV(CFG, "  M92 T", i);
        ECHO_EMV(" E", axis_steps_per_unit[E_AXIS + i]);
      }
    #endif //EXTRUDERS > 1

    #if MECH(SCARA)
      if (!forReplay) {
        ECHO_LM(CFG, "Scaling factors:");
      }
      ECHO_SMV(CFG, "  M365 X", axis_scaling[X_AXIS]);
      ECHO_MV(" Y", axis_scaling[Y_AXIS]);
      ECHO_EMV(" Z", axis_scaling[Z_AXIS]);
    #endif // SCARA

    if (!forReplay) {
      ECHO_LM(CFG, "Maximum feedrates (mm/s):");
    }
    ECHO_SMV(CFG, "  M203 X", max_feedrate[X_AXIS]);
    ECHO_MV(" Y", max_feedrate[Y_AXIS] );
    ECHO_MV(" Z", max_feedrate[Z_AXIS] );
    ECHO_EMV(" E", max_feedrate[E_AXIS]);
    #if EXTRUDERS > 1
      for (short i = 1; i < EXTRUDERS; i++) {
        ECHO_SMV(CFG, "  M203 T", i);
        ECHO_EMV(" E", max_feedrate[E_AXIS + i]);
      }
    #endif //EXTRUDERS > 1

    if (!forReplay) {
      ECHO_LM(CFG, "Maximum Acceleration (mm/s2):");
    }
    ECHO_SMV(CFG, "  M201 X", max_acceleration_units_per_sq_second[X_AXIS] );
    ECHO_MV(" Y", max_acceleration_units_per_sq_second[Y_AXIS] );
    ECHO_MV(" Z", max_acceleration_units_per_sq_second[Z_AXIS] );
    ECHO_EMV(" E", max_acceleration_units_per_sq_second[E_AXIS]);
    #if EXTRUDERS > 1
      for (int8_t i = 1; i < EXTRUDERS; i++) {
        ECHO_SMV(CFG, "  M201 T", i);
        ECHO_EMV(" E", max_acceleration_units_per_sq_second[E_AXIS + i]);
      }
    #endif //EXTRUDERS > 1
    ECHO_E;
    
    if (!forReplay) {
      ECHO_LM(CFG, "Accelerations: P=printing, V=travel and T* R=retract");
    }
    ECHO_SMV(CFG,"  M204 P", acceleration);
    ECHO_EMV(" V", travel_acceleration);
    #if EXTRUDERS > 0
      for (int8_t i = 0; i < EXTRUDERS; i++) {
        ECHO_SMV(CFG, "  M204 T", i);
        ECHO_EMV(" R", retract_acceleration[i]);
      }
    #endif

    if (!forReplay) {
      ECHO_LM(CFG, "Advanced variables: S=Min feedrate (mm/s), V=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    }
    ECHO_SMV(CFG, "  M205 S", minimumfeedrate );
    ECHO_MV(" V", mintravelfeedrate );
    ECHO_MV(" B", minsegmenttime );
    ECHO_MV(" X", max_xy_jerk );
    ECHO_MV(" Z", max_z_jerk);
    ECHO_EMV(" E", max_e_jerk[0]);
    #if (EXTRUDERS > 1)
      for(int8_t i = 1; i < EXTRUDERS; i++) {
        ECHO_SMV(CFG, "  M205 T", i);
        ECHO_EMV(" E" , max_e_jerk[i]);
      }
    #endif

    if (!forReplay) {
      ECHO_LM(CFG, "Home offset (mm):");
    }
    ECHO_SMV(CFG, "  M206 X", home_offset[X_AXIS] );
    ECHO_MV(" Y", home_offset[Y_AXIS] );
    ECHO_EMV(" Z", home_offset[Z_AXIS] );

    if (!forReplay) {
      ECHO_LM(CFG, "Hotend offset (mm):");
    }
    for (int8_t h = 0; h < HOTENDS; h++) {
      ECHO_SMV(CFG, "  M218 T", h);
      ECHO_MV(" X", hotend_offset[X_AXIS][h]);
      ECHO_MV(" Y", hotend_offset[Y_AXIS][h]);
      ECHO_EMV(" Z", hotend_offset[Z_AXIS][h]);
    }

    #if ENABLED(MESH_BED_LEVELING)
      if (!forReplay) {
        ECHO_LM(CFG, "Mesh bed leveling:");
      }
      ECHO_SMV(CFG, "  M420 S", mbl.active);
      ECHO_MV(" X", MESH_NUM_X_POINTS);
      ECHO_EMV(" Y", MESH_NUM_Y_POINTS);

      for (int py = 1; py <= MESH_NUM_Y_POINTS; py++) {
        for (int px = 1; px <= MESH_NUM_X_POINTS; px++) {
          ECHO_SMV(CFG, "  G29 S3 X", px);
          ECHO_MV(" Y", py);
          ECHO_EMV(" Z", mbl.z_values[py-1][px-1], 5);
        }
      }
    #endif
  
    #if HEATER_USES_AD595
      if (!forReplay) {
        ECHO_LM(CFG, "AD595 Offset and Gain:");
      }
      for (int8_t h = 0; h < HOTENDS; h++) {
        ECHO_SMV(CFG, "  M595 T", h);
        ECHO_MV(" O", ad595_offset[h]);
        ECHO_EMV(", S", ad595_gain[h]);
      }
    #endif // HEATER_USES_AD595

    #if MECH(DELTA)
      if (!forReplay) {
        ECHO_LM(CFG, "Delta Geometry adjustment:");
      }
      ECHO_SMV(CFG, "  M666 A", tower_adj[0], 3);
      ECHO_MV(" B", tower_adj[1], 3);
      ECHO_MV(" C", tower_adj[2], 3);
      ECHO_MV(" I", tower_adj[3], 3);
      ECHO_MV(" J", tower_adj[4], 3);
      ECHO_MV(" K", tower_adj[5], 3);
      ECHO_MV(" U", diagrod_adj[0], 3);
      ECHO_MV(" V", diagrod_adj[1], 3);
      ECHO_MV(" W", diagrod_adj[2], 3);
      ECHO_MV(" R", delta_radius);
      ECHO_MV(" D", delta_diagonal_rod);
      ECHO_EMV(" H", sw_endstop_max[2]);

      if (!forReplay) {
        ECHO_LM(CFG, "Endstop Offsets:");
      }
      ECHO_SMV(CFG, "  M666 X", endstop_adj[X_AXIS]);
      ECHO_MV(" Y", endstop_adj[Y_AXIS]);
      ECHO_EMV(" Z", endstop_adj[Z_AXIS]);

      if (!forReplay) {
        ECHO_LM(CFG, "Z-Probe Offset:");
      }
      ECHO_SMV(CFG, "  M666 P X", z_probe_offset[0]);
      ECHO_MV(" Y", z_probe_offset[1]);
      ECHO_EMV(" Z", z_probe_offset[2]);

    #elif ENABLED(Z_DUAL_ENDSTOPS)
      if (!forReplay) {
        ECHO_LM(CFG, "Z2 Endstop adjustement (mm):");
      }
      ECHO_LMV(CFG, "  M666 Z", z_endstop_adj );
    #elif ENABLED(AUTO_BED_LEVELING_FEATURE)
      if (!forReplay) {
        ECHO_LM(CFG, "Z Probe offset (mm)");
      }
      ECHO_LMV(CFG, "  M666 P", zprobe_zoffset);
    #endif

    #if ENABLED(ULTIPANEL)
      if (!forReplay) {
        ECHO_LM(CFG, "Material heatup parameters:");
      }
      ECHO_SMV(CFG, "  M145 S0 H", plaPreheatHotendTemp);
      ECHO_MV(" B", plaPreheatHPBTemp);
      ECHO_MV(" F", plaPreheatFanSpeed);
      ECHO_EM(" (Material PLA)");
      ECHO_SMV(CFG, "  M145 S1 H", absPreheatHotendTemp);
      ECHO_MV(" B", absPreheatHPBTemp);
      ECHO_MV(" F", absPreheatFanSpeed);
      ECHO_EM(" (Material ABS)");
      ECHO_SMV(CFG, "  M145 S2 H", gumPreheatHotendTemp);
      ECHO_MV(" B", gumPreheatHPBTemp);
      ECHO_MV(" F", gumPreheatFanSpeed);
      ECHO_EM(" (Material GUM)");
    #endif // ULTIPANEL

    #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
      if (!forReplay) {
        ECHO_LM(CFG, "PID settings:");
      }
      #if ENABLED(PIDTEMP)
        for (uint8_t h = 0; h < HOTENDS; h++) {
          ECHO_SMV(CFG, "  M301 H", h);
          ECHO_MV(" P", PID_PARAM(Kp, h));
          ECHO_MV(" I", unscalePID_i(PID_PARAM(Ki, h)));
          ECHO_MV(" D", unscalePID_d(PID_PARAM(Kd, h)));
          #if ENABLED(PID_ADD_EXTRUSION_RATE)
            ECHO_MV(" C", PID_PARAM(Kc, h));
          #endif
          ECHO_E;
        }
        #if ENABLED(PID_ADD_EXTRUSION_RATE)
          ECHO_SMV(CFG, "  M301 L", lpq_len);
        #endif
      #endif
      #if ENABLED(PIDTEMPBED)
        ECHO_SMV(CFG, "  M304 P", bedKp);
        ECHO_MV(" I", unscalePID_i(bedKi));
        ECHO_EMV(" D", unscalePID_d(bedKd));
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        ECHO_SMV(CFG, "  M305 P", chamberKp);
        ECHO_MV(" I", unscalePID_i(chamberKi));
        ECHO_EMV(" D", unscalePID_d(chamberKd));
      #endif
      #if ENABLED(PIDTEMPCOOLER)
        ECHO_SMV(CFG, "  M306 P", coolerKp);
        ECHO_MV(" I", unscalePID_i(coolerKi));
        ECHO_EMV(" D", unscalePID_d(coolerKd));
      #endif

    #endif

    #if ENABLED(FWRETRACT)
      if (!forReplay) {
        ECHO_LM(CFG, "Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
      }
      ECHO_SMV(CFG, "  M207 S", retract_length);
      ECHO_MV(" F", retract_feedrate*60);
      ECHO_EMV(" Z", retract_zlift);
      
      if (!forReplay) {
        ECHO_LM(CFG, "Recover: S=Extra length (mm) F:Speed (mm/m)");
      }
      ECHO_SMV(CFG, "  M208 S", retract_recover_length);
      ECHO_MV(" F", retract_recover_feedrate*60);
      
      if (!forReplay) {
        ECHO_LM(CFG, "Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
      }
      ECHO_LMV(CFG, "  M209 S", autoretract_enabled);

      #if EXTRUDERS > 1
        if (!forReplay) {
          ECHO_LM(CFG, "Multi-extruder settings:");
          ECHO_LMV(CFG, "   Swap retract length (mm):    ", retract_length_swap);
          ECHO_LMV(CFG, "   Swap rec. addl. length (mm): ", retract_recover_length_swap);
        }
      #endif // EXTRUDERS > 1

    #endif // FWRETRACT

    if (volumetric_enabled) {
      if (!forReplay) {
        ECHO_LM(CFG, "Filament settings:");
      }
      ECHO_LMV(CFG, "  M200 D", filament_size[0]);

      #if EXTRUDERS > 1
        ECHO_LMV(CFG, "  M200 T1 D", filament_size[1]);
        #if EXTRUDERS > 2
          ECHO_LMV(CFG, "  M200 T2 D", filament_size[2]);
          #if EXTRUDERS > 3
            ECHO_LMV(CFG, "  M200 T3 D", filament_size[3]);
          #endif
        #endif
      #endif

    } else {
      if (!forReplay) {
        ECHO_LM(CFG, "Filament settings: Disabled");
      }
    }

    #if MB(ALLIGATOR)
      if (!forReplay) {
        ECHO_LM(CFG, "Current:");
      }
      ECHO_SMV(CFG, "  M906 X", motor_current[X_AXIS]);
      ECHO_MV(" Y", motor_current[Y_AXIS]);
      ECHO_MV(" Z", motor_current[Z_AXIS]);
      ECHO_EMV(" E", motor_current[E_AXIS]);
      #if DRIVER_EXTRUDERS > 1
        for (uint8_t i = 1; i < DRIVER_EXTRUDERS; i++) {
          ECHO_SMV(CFG, "  M906 T", i);
          ECHO_EMV(" E", motor_current[E_AXIS + i]);
        }
      #endif // DRIVER_EXTRUDERS > 1
    #endif // ALLIGATOR

    ConfigSD_PrintSettings(forReplay);

  }

  void ConfigSD_PrintSettings(bool forReplay) {
    // Always have this function, even with SD_SETTINGS disabled, the current values will be shown

    #if HAS(POWER_CONSUMPTION_SENSOR)
      if (!forReplay) {
        ECHO_LM(INFO, "Watt/h consumed:");
      }
      ECHO_LVM(INFO, power_consumption_hour," Wh");
    #endif

    print_job_counter.showStats();
  }

#endif // !DISABLE_M503

/**
 * Configuration on SD card
 *
 * Author: Simone Primarosa
 *
 */
void ConfigSD_ResetDefault() {
  #if HAS(POWER_CONSUMPTION_SENSOR)
   power_consumption_hour = 0;
  #endif
  print_job_counter.initStats();
  ECHO_LM(OK, "Hardcoded SD Default Settings Loaded");
}

#if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
  static const char *cfgSD_KEY[] = { // Keep this in lexicographical order for better search performance(O(Nlog2(N)) insted of O(N*N)) (if you don't keep this sorted, the algorithm for find the key index won't work, keep attention.)
    "CPR",  // Number of complete prints
    "FIL",  // Filament Usage
    "NPR",  // Number of prints
  #if HAS(POWER_CONSUMPTION_SENSOR)
    "PWR",  // Power Consumption
  #endif
    "TME",  // Longest print job
    "TPR"   // Total printing time
  };

  enum cfgSD_ENUM {   // This need to be in the same order as cfgSD_KEY
    SD_CFG_CPR,
    SD_CFG_FIL,
    SD_CFG_NPR,
  #if HAS(POWER_CONSUMPTION_SENSOR)
    SD_CFG_PWR,
  #endif
    SD_CFG_TME,
    SD_CFG_TPR,
    SD_CFG_END // Leave this always as the last
  };

  void ConfigSD_StoreSettings() {
    if(!IS_SD_INSERTED || card.isFileOpen() || card.sdprinting) return;
    set_sd_dot();
    card.setroot(true);
    card.startWrite((char *)CFG_SD_FILE, false);
    char buff[CFG_SD_MAX_VALUE_LEN];
    ltoa(print_job_counter.data.completePrints, buff, 10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_CPR], buff);
    ltoa(print_job_counter.data.printer_usage_filament, buff, 10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_FIL], buff);
    ltoa(print_job_counter.data.numberPrints, buff, 10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_NPR], buff);
    #if HAS(POWER_CONSUMPTION_SENSOR)
      ltoa(power_consumption_hour, buff, 10);
      card.unparseKeyLine(cfgSD_KEY[SD_CFG_PWR], buff);
    #endif
    ltoa(print_job_counter.data.printer_usage_seconds, buff, 10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_TME], buff);
    ltoa(print_job_counter.data.printTime, buff, 10);
    card.unparseKeyLine(cfgSD_KEY[SD_CFG_TPR], buff);

    card.closeFile();
    card.setlast();
    unset_sd_dot();
  }

  void ConfigSD_RetrieveSettings(bool addValue) {
    if(!IS_SD_INSERTED || card.isFileOpen() || card.sdprinting || !card.cardOK) return;
    set_sd_dot();
    char key[CFG_SD_MAX_KEY_LEN], value[CFG_SD_MAX_VALUE_LEN];
    int k_idx;
    int k_len, v_len;
    card.setroot(true);
    card.selectFile((char *)CFG_SD_FILE);
    while(true) {
      k_len = CFG_SD_MAX_KEY_LEN;
      v_len = CFG_SD_MAX_VALUE_LEN;
      card.parseKeyLine(key, value, k_len, v_len);

      if(k_len == 0 || v_len == 0) break; // no valid key or value founded

      k_idx = ConfigSD_KeyIndex(key);
      if(k_idx == -1) continue;    // unknow key ignore it

      switch(k_idx) {
        case SD_CFG_CPR: {
          if(addValue) print_job_counter.data.completePrints += (unsigned long)atol(value);
          else print_job_counter.data.completePrints = (unsigned long)atol(value);
        }
        break;
        case SD_CFG_FIL: {
          if(addValue) print_job_counter.data.printer_usage_filament += (unsigned long)atol(value);
          else print_job_counter.data.printer_usage_filament = (unsigned long)atol(value);
        }
        break;
        case SD_CFG_NPR: {
          if(addValue) print_job_counter.data.numberPrints += (unsigned long)atol(value);
          else print_job_counter.data.numberPrints = (unsigned long)atol(value);
        }
        break;
      #if HAS(POWER_CONSUMPTION_SENSOR)
        case SD_CFG_PWR: {
          if(addValue) power_consumption_hour += (unsigned long)atol(value);
          else power_consumption_hour = (unsigned long)atol(value);
        }
        break;
      #endif
        case SD_CFG_TME: {
          if(addValue) print_job_counter.data.printer_usage_seconds += (unsigned long)atol(value);
          else print_job_counter.data.printer_usage_seconds = (unsigned long)atol(value);
        }
        break;
        case SD_CFG_TPR: {
          if(addValue) print_job_counter.data.printTime += (unsigned long)atol(value);
          else print_job_counter.data.printTime = (unsigned long)atol(value);
        }
        break;
      }
    }

    print_job_counter.loaded = true;
    card.closeFile();
    card.setlast();
    unset_sd_dot();
  }

  int ConfigSD_KeyIndex(char *key) {  // At the moment a binary search algorithm is used for simplicity, if it will be necessary (Eg. tons of key), an hash search algorithm will be implemented.
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
