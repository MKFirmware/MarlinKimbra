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

#define EEPROM_VERSION "MKV28"

/**
 * MKV428 EEPROM Layout:
 *
 *  Version (char x6)
 *  EEPROM Checksum (uint16_t)
 *
 *  M92   XYZ E0 ...      planner.axis_steps_per_mm X,Y,Z,E0 ... (float x9)
 *  M203  XYZ E0 ...      planner.max_feedrate X,Y,Z,E0 ... (float x9)
 *  M201  XYZ E0 ...      planner.max_acceleration_mm_per_s2 X,Y,Z,E0 ... (uint32_t x9)
 *  M204  P               planner.acceleration (float)
 *  M204  R   E0 ...      planner.retract_acceleration (float x6)
 *  M204  T               planner.travel_acceleration (float)
 *  M205  S               planner.min_feedrate (float)
 *  M205  T               planner.min_travel_feedrate (float)
 *  M205  B               planner.min_segment_time (ulong)
 *  M205  X               planner.max_xy_jerk (float)
 *  M205  Z               planner.max_z_jerk (float)
 *  M205  E   E0 ...      planner.max_e_jerk (float x6)
 *  M206  XYZ             home_offset (float x3)
 *  M218  T   XY          hotend_offset (float x6)
 *
 * Mesh bed leveling:
 *  M420  S               status (uint8)
 *                        z_offset (float)
 *                        mesh_num_x (uint8 as set in firmware)
 *                        mesh_num_y (uint8 as set in firmware)
 *  G29   S3  XYZ         z_values[][] (float x9, by default)
 *
 * HOTENDS AD595:
 *  M595  H OS            Hotend AD595 Offset & Gain
 *
 * DELTA:
 *  M666  XYZ             endstop_adj (float x3)
 *  M666  R               delta_radius (float)
 *  M666  D               delta_diagonal_rod (float)
 *  M666  H               Z sw_endstop_max (float)
 *  M666  ABCIJK          tower_adj (float x6)
 *  M666  UVW             diagrod_adj (float x3)
 *
 * Z_DUAL_ENDSTOPS:
 *  M666  Z               z_endstop_adj (float)
 *
 * Z PROBE:
 *  M666  P               zprobe_zoffset (float)
 *
 * ULTIPANEL:
 *  M145  S0  H           plaPreheatHotendTemp (int)
 *  M145  S0  B           plaPreheatHPBTemp (int)
 *  M145  S0  F           plaPreheatFanSpeed (int)
 *  M145  S1  H           absPreheatHotendTemp (int)
 *  M145  S1  B           absPreheatHPBTemp (int)
 *  M145  S1  F           absPreheatFanSpeed (int)
 *  M145  S2  H           gumPreheatHotendTemp (int)
 *  M145  S2  B           gumPreheatHPBTemp (int)
 *  M145  S2  F           gumPreheatFanSpeed (int)
 *
 * PIDTEMP:
 *  M301  E0  PIDC        Kp[0], Ki[0], Kd[0], Kc[0] (float x4)
 *  M301  E1  PIDC        Kp[1], Ki[1], Kd[1], Kc[1] (float x4)
 *  M301  E2  PIDC        Kp[2], Ki[2], Kd[2], Kc[2] (float x4)
 *  M301  E3  PIDC        Kp[3], Ki[3], Kd[3], Kc[3] (float x4)
 *  M301  L               lpq_len
 *
 * PIDTEMPBED:
 *  M304      PID         bedKp, bedKi, bedKd (float x3)
 * PIDTEMPCHAMBER
 *  M305      PID         chamberKp, chamberKi, chamberKd (float x3)
 * PIDTEMPCOOLER
 *  M306      PID         coolerKp, coolerKi, coolerKd (float x3)
 *
 * DOGLCD:
 *  M250  C               lcd_contrast (int)
 *
 * SCARA:
 *  M365  XYZ             axis_scaling (float x3)
 *
 * FWRETRACT:
 *  M209  S               autoretract_enabled (bool)
 *  M207  S               retract_length (float)
 *  M207  W               retract_length_swap (float)
 *  M207  F               retract_feedrate (float)
 *  M207  Z               retract_zlift (float)
 *  M208  S               retract_recover_length (float)
 *  M208  W               retract_recover_length_swap (float)
 *  M208  F               retract_recover_feedrate (float)
 *
 *  M200  D               volumetric_enabled (bool)
 *
 *  M200  T D             filament_size (float x6)
 *
 *  M???  S               IDLE_OOZING_enabled
 *
 * ALLIGATOR:
 *  M906  XYZ T0-4 E      Motor current (float x7)
 *
 */

uint16_t eeprom_checksum;
const char version[6] = EEPROM_VERSION;

void _EEPROM_writeData(int& pos, uint8_t* value, uint8_t size) {
  uint8_t c;
  while(size--) {
    eeprom_write_byte((unsigned char*)pos, *value);
    c = eeprom_read_byte((unsigned char*)pos);
    if (c != *value) {
      ECHO_LM(ER, SERIAL_ERR_EEPROM_WRITE);
    }
    eeprom_checksum += c;
    pos++;
    value++;
  };
}

void _EEPROM_readData(int& pos, uint8_t* value, uint8_t size) {
  do {
    byte c = eeprom_read_byte((unsigned char*)pos);
    *value = c;
    eeprom_checksum += c;
    pos++;
    value++;
  } while (--size);
}

#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))

#define EEPROM_OFFSET 100

/**
 * Post-process after Retrieve or Reset
 */
void Config_Postprocess() {
  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  #if ENABLED(DELTA)
    set_delta_constants();
  #endif

  #if ENABLED(PIDTEMP)
    updatePID();
  #endif

  calculate_volumetric_multipliers();
}

#if ENABLED(EEPROM_SETTINGS)

/**
 * M500 - Store Configuration
 */
void Config_StoreSettings() {
  float dummy = 0.0f;
  char ver[6] = "00000";
  int i = EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i, ver); // invalidate data first
  EEPROM_WRITE_VAR(i, eeprom_checksum);

  // Start calc checksum
  eeprom_checksum = 0;
  EEPROM_WRITE_VAR(i, planner.axis_steps_per_mm);
  EEPROM_WRITE_VAR(i, planner.max_feedrate);
  EEPROM_WRITE_VAR(i, planner.max_acceleration_mm_per_s2);
  EEPROM_WRITE_VAR(i, planner.acceleration);
  EEPROM_WRITE_VAR(i, planner.retract_acceleration);
  EEPROM_WRITE_VAR(i, planner.travel_acceleration);
  EEPROM_WRITE_VAR(i, planner.min_feedrate);
  EEPROM_WRITE_VAR(i, planner.min_travel_feedrate);
  EEPROM_WRITE_VAR(i, planner.min_segment_time);
  EEPROM_WRITE_VAR(i, planner.max_xy_jerk);
  EEPROM_WRITE_VAR(i, planner.max_z_jerk);
  EEPROM_WRITE_VAR(i, planner.max_e_jerk);
  EEPROM_WRITE_VAR(i, home_offset);
  EEPROM_WRITE_VAR(i, hotend_offset);

  #if ENABLED(MESH_BED_LEVELING)
    // Compile time test that sizeof(mbl.z_values) is as expected
    typedef char c_assert[(sizeof(mbl.z_values) == (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS) * sizeof(dummy)) ? 1 : -1];
    uint8_t mesh_num_x  = MESH_NUM_X_POINTS,
            mesh_num_y  = MESH_NUM_Y_POINTS,
            dummy_uint8 = mbl.status & _BV(MBL_STATUS_HAS_MESH_BIT);
    EEPROM_WRITE_VAR(i, dummy_uint8);
    EEPROM_WRITE_VAR(i, mbl.z_offset);
    EEPROM_WRITE_VAR(i, mesh_num_x);
    EEPROM_WRITE_VAR(i, mesh_num_y);
    EEPROM_WRITE_VAR(i, mbl.z_values);
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
  #elif ENABLED(Z_DUAL_ENDSTOPS)
    EEPROM_WRITE_VAR(i, z_endstop_adj);
  #endif

  #if HASNT(BED_PROBE)
    float zprobe_zoffset = 0;
  #endif
  EEPROM_WRITE_VAR(i, zprobe_zoffset);

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

  int j = EEPROM_OFFSET;
  uint16_t stored_checksum = eeprom_checksum;
  EEPROM_WRITE_VAR(j, version);
  EEPROM_WRITE_VAR(j, stored_checksum);

  // Report storage size
  ECHO_SMV(DB, "Settings Stored (", i);
  ECHO_EM(" bytes)");
}

/**
 * M501 - Retrieve Configuration
 */
void Config_RetrieveSettings() {
  int i = EEPROM_OFFSET;
  char stored_ver[6];
  uint16_t stored_checksum;
  EEPROM_READ_VAR(i, stored_ver);
  EEPROM_READ_VAR(i, stored_checksum);

  if (DEBUGGING(INFO)) {
    ECHO_SMV(INFO, "Version: [", version);
    ECHO_MV("] Stored version: [", stored_ver);
    ECHO_EM("]");
  }

  if (strncmp(version, stored_ver, 5) != 0) {
    Config_ResetDefault();
  }
  else {
    eeprom_checksum = 0;
    float dummy = 0;

    // version number match
    EEPROM_READ_VAR(i, planner.axis_steps_per_mm);
    EEPROM_READ_VAR(i, planner.max_feedrate);
    EEPROM_READ_VAR(i, planner.max_acceleration_mm_per_s2);

    EEPROM_READ_VAR(i, planner.acceleration);
    EEPROM_READ_VAR(i, planner.retract_acceleration);
    EEPROM_READ_VAR(i, planner.travel_acceleration);
    EEPROM_READ_VAR(i, planner.min_feedrate);
    EEPROM_READ_VAR(i, planner.min_travel_feedrate);
    EEPROM_READ_VAR(i, planner.min_segment_time);
    EEPROM_READ_VAR(i, planner.max_xy_jerk);
    EEPROM_READ_VAR(i, planner.max_z_jerk);
    EEPROM_READ_VAR(i, planner.max_e_jerk);
    EEPROM_READ_VAR(i, home_offset);
    EEPROM_READ_VAR(i, hotend_offset);

    #if ENABLED(MESH_BED_LEVELING)
      uint8_t mesh_num_x = 0, mesh_num_y = 0;
      EEPROM_READ_VAR(i, mbl.status);
      EEPROM_READ_VAR(i, mbl.z_offset);
      EEPROM_READ_VAR(i, mesh_num_x);
      EEPROM_READ_VAR(i, mesh_num_y);
      EEPROM_READ_VAR(i, mbl.z_values);
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
    #endif //DELTA

    #if HASNT(BED_PROBE)
      float zprobe_zoffset = 0;
    #endif
    EEPROM_READ_VAR(i, zprobe_zoffset);

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

    #if ENABLED(IDLE_OOZING_PREVENT)
      EEPROM_READ_VAR(i, IDLE_OOZING_enabled);
    #endif

    #if MB(ALLIGATOR)
      EEPROM_READ_VAR(i, motor_current);
    #endif

    if (stored_checksum == eeprom_checksum) {
      Config_Postprocess();
      ECHO_SV(DB, version);
      ECHO_MV(" stored settings retrieved (", i);
      ECHO_EM(" bytes)");
    }
    else {
      ECHO_LM(ER, "EEPROM checksum mismatch");
      Config_ResetDefault();
    }
  }

  #if ENABLED(EEPROM_CHITCHAT)
    Config_PrintSettings();
  #endif
}

#endif // EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
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
    for (int8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++)
      motor_current[i] = tmp13[i];
  #endif

  for (int8_t i = 0; i < 3 + EXTRUDERS; i++) {
    planner.axis_steps_per_mm[i] = tmp1[i];
    planner.max_feedrate[i] = tmp2[i];
    planner.max_acceleration_mm_per_s2[i] = tmp3[i];
  }

  for (int8_t i = 0; i < EXTRUDERS; i++) {
    planner.retract_acceleration[i] = tmp4[i];
    planner.max_e_jerk[i] = tmp5[i];
  }

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
  planner.reset_acceleration_rates();

  planner.acceleration = DEFAULT_ACCELERATION;
  planner.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.min_feedrate = DEFAULT_MINIMUMFEEDRATE;
  planner.min_segment_time = DEFAULT_MINSEGMENTTIME;
  planner.min_travel_feedrate = DEFAULT_MINTRAVELFEEDRATE;
  planner.max_xy_jerk = DEFAULT_XYJERK;
  planner.max_z_jerk = DEFAULT_ZJERK;
  home_offset[X_AXIS] = home_offset[Y_AXIS] = home_offset[Z_AXIS] = 0;

  #if ENABLED(MESH_BED_LEVELING)
    mbl.reset();
  #endif

  #if HAS(BED_PROBE)
    zprobe_zoffset = Z_PROBE_OFFSET_FROM_NOZZLE;
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

  #if ENABLED(IDLE_OOZING_PREVENT)
    IDLE_OOZING_enabled = true;
  #endif

  Config_Postprocess();

  ECHO_LM(DB, "Hardcoded Default Settings Loaded");
}

#if DISABLED(DISABLE_M503)

#define CONFIG_ECHO_START(str) do{ if (!forReplay) ECHO_LM(CFG, str); }while(0)

/**
 * M503 - Print Configuration
 */
void Config_PrintSettings(bool forReplay) {
  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown

  CONFIG_ECHO_START("Steps per unit:");
  ECHO_SMV(CFG, "  M92 X", planner.axis_steps_per_mm[X_AXIS]);
  ECHO_MV(" Y", planner.axis_steps_per_mm[Y_AXIS]);
  ECHO_MV(" Z", planner.axis_steps_per_mm[Z_AXIS]);
  ECHO_EMV(" E", planner.axis_steps_per_mm[E_AXIS]);
  #if EXTRUDERS > 1
    for (short i = 1; i < EXTRUDERS; i++) {
      ECHO_SMV(CFG, "  M92 T", i);
      ECHO_EMV(" E", planner.axis_steps_per_mm[E_AXIS + i]);
    }
  #endif //EXTRUDERS > 1

  #if MECH(SCARA)
    CONFIG_ECHO_START("Scaling factors:");
    ECHO_SMV(CFG, "  M365 X", axis_scaling[X_AXIS]);
    ECHO_MV(" Y", axis_scaling[Y_AXIS]);
    ECHO_EMV(" Z", axis_scaling[Z_AXIS]);
  #endif // SCARA

  CONFIG_ECHO_START("Maximum feedrates (mm/s):");
  ECHO_SMV(CFG, "  M203 X", planner.max_feedrate[X_AXIS]);
  ECHO_MV(" Y", planner.max_feedrate[Y_AXIS] );
  ECHO_MV(" Z", planner.max_feedrate[Z_AXIS] );
  ECHO_EMV(" E", planner.max_feedrate[E_AXIS]);
  #if EXTRUDERS > 1
    for (short i = 1; i < EXTRUDERS; i++) {
      ECHO_SMV(CFG, "  M203 T", i);
      ECHO_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS + i]);
    }
  #endif //EXTRUDERS > 1

  CONFIG_ECHO_START("Maximum Acceleration (mm/s2):");
  ECHO_SMV(CFG, "  M201 X", planner.max_acceleration_mm_per_s2[X_AXIS] );
  ECHO_MV(" Y", planner.max_acceleration_mm_per_s2[Y_AXIS] );
  ECHO_MV(" Z", planner.max_acceleration_mm_per_s2[Z_AXIS] );
  ECHO_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS]);
  #if EXTRUDERS > 1
    for (int8_t i = 1; i < EXTRUDERS; i++) {
      ECHO_SMV(CFG, "  M201 T", i);
      ECHO_EMV(" E", planner.max_acceleration_mm_per_s2[E_AXIS + i]);
    }
  #endif //EXTRUDERS > 1
  
  CONFIG_ECHO_START("Accelerations: P=printing, V=travel and T* R=retract");
  ECHO_SMV(CFG,"  M204 P", planner.acceleration);
  ECHO_EMV(" V", planner.travel_acceleration);
  #if EXTRUDERS > 0
    for (int8_t i = 0; i < EXTRUDERS; i++) {
      ECHO_SMV(CFG, "  M204 T", i);
      ECHO_EMV(" R", planner.retract_acceleration[i]);
    }
  #endif

  CONFIG_ECHO_START("Advanced variables: S=Min feedrate (mm/s), V=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
  ECHO_SMV(CFG, "  M205 S", planner.min_feedrate );
  ECHO_MV(" V", planner.min_travel_feedrate );
  ECHO_MV(" B", planner.min_segment_time );
  ECHO_MV(" X", planner.max_xy_jerk );
  ECHO_MV(" Z", planner.max_z_jerk);
  ECHO_EMV(" E", planner.max_e_jerk[0]);
  #if (EXTRUDERS > 1)
    for(int8_t i = 1; i < EXTRUDERS; i++) {
      ECHO_SMV(CFG, "  M205 T", i);
      ECHO_EMV(" E" , planner.max_e_jerk[i]);
    }
  #endif

  CONFIG_ECHO_START("Home offset (mm):");
  ECHO_SMV(CFG, "  M206 X", home_offset[X_AXIS] );
  ECHO_MV(" Y", home_offset[Y_AXIS] );
  ECHO_EMV(" Z", home_offset[Z_AXIS] );

  CONFIG_ECHO_START("Hotend offset (mm):");
  for (int8_t h = 0; h < HOTENDS; h++) {
    ECHO_SMV(CFG, "  M218 T", h);
    ECHO_MV(" X", hotend_offset[X_AXIS][h]);
    ECHO_MV(" Y", hotend_offset[Y_AXIS][h]);
    ECHO_EMV(" Z", hotend_offset[Z_AXIS][h]);
  }

  #if HAS(LCD_CONTRAST)
    CONFIG_ECHO_START("LCD Contrast:");
    ECHO_LMV(CFG, "  M250 C", lcd_contrast);
  #endif

  #if ENABLED(MESH_BED_LEVELING)
    CONFIG_ECHO_START("Mesh bed leveling:");
    ECHO_SMV(CFG, "  M420 S", mbl.has_mesh() ? 1 : 0);
    ECHO_MV(" X", MESH_NUM_X_POINTS);
    ECHO_MV(" Y", MESH_NUM_Y_POINTS);
    ECHO_E;

    for (uint8_t py = 1; py <= MESH_NUM_Y_POINTS; py++) {
      for (uint8_t px = 1; px <= MESH_NUM_X_POINTS; px++) {
        ECHO_SMV(CFG, "  G29 S3 X", px);
        ECHO_MV(" Y", py);
        ECHO_EMV(" Z", mbl.z_values[py-1][px-1], 5);
      }
    }
  #endif

  #if HEATER_USES_AD595
    CONFIG_ECHO_START("AD595 Offset and Gain:");
    for (int8_t h = 0; h < HOTENDS; h++) {
      ECHO_SMV(CFG, "  M595 T", h);
      ECHO_MV(" O", ad595_offset[h]);
      ECHO_EMV(", S", ad595_gain[h]);
    }
  #endif // HEATER_USES_AD595

  #if MECH(DELTA)
    CONFIG_ECHO_START("Delta Geometry adjustment:");
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

    CONFIG_ECHO_START("Endstop Offsets:");
    ECHO_SMV(CFG, "  M666 X", endstop_adj[X_AXIS]);
    ECHO_MV(" Y", endstop_adj[Y_AXIS]);
    ECHO_EMV(" Z", endstop_adj[Z_AXIS]);

  #elif ENABLED(Z_DUAL_ENDSTOPS)
    CONFIG_ECHO_START("Z2 Endstop adjustement (mm):");
    ECHO_LMV(CFG, "  M666 Z", z_endstop_adj );
  #endif // DELTA
  
  /**
   * Auto Bed Leveling
   */
  #if HAS(BED_PROBE)
    CONFIG_ECHO_START("Z Probe offset (mm):");
    ECHO_LMV(CFG, "  M666 P", zprobe_zoffset);
  #endif

  #if ENABLED(ULTIPANEL)
    CONFIG_ECHO_START("Material heatup parameters:");
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
    CONFIG_ECHO_START("PID settings:");
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
    CONFIG_ECHO_START("Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
    ECHO_SMV(CFG, "  M207 S", retract_length);
    #if EXTRUDERS > 1
      ECHO_MV(" W", retract_length_swap);
    #endif
    ECHO_MV(" F", retract_feedrate * 60);
    ECHO_EMV(" Z", retract_zlift);

    CONFIG_ECHO_START("Recover: S=Extra length (mm) F:Speed (mm/m)");
    ECHO_SMV(CFG, "  M208 S", retract_recover_length);
    #if EXTRUDERS > 1
      ECHO_MV(" W", retract_recover_length_swap);
    #endif
    ECHO_MV(" F", retract_recover_feedrate * 60);

    CONFIG_ECHO_START("Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries");
    ECHO_LMV(CFG, "  M209 S", autoretract_enabled ? 1 : 0);
  #endif // FWRETRACT

  if (volumetric_enabled) {
    CONFIG_ECHO_START("Filament settings:");
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

  }
  else
    CONFIG_ECHO_START("  M200 D0");

  #if MB(ALLIGATOR)
    CONFIG_ECHO_START("Motor current:");
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
    CONFIG_ECHO_START("Watt/h consumed:");
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
