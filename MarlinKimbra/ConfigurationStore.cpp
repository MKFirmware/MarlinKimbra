#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
  do
  {
    eeprom_write_byte((unsigned char*)pos, *value);
    pos++;
    value++;
  }
  while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
  do
  {
    *value = eeprom_read_byte((unsigned char*)pos);
    pos++;
    value++;
  }
  while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================

#define EEPROM_OFFSET 100

// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.

#define EEPROM_VERSION "V10"
#ifdef DELTA
  #define EEPROM_VERSION "V11"
#endif
#ifdef SCARA
  #define EEPROM_VERSION "V12"
#endif

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first
  EEPROM_WRITE_VAR(i,baudrate);
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);
  EEPROM_WRITE_VAR(i,max_feedrate);
  EEPROM_WRITE_VAR(i,max_retraction_feedrate);
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homing);
#ifdef DELTA
  EEPROM_WRITE_VAR(i,delta_radius);
  EEPROM_WRITE_VAR(i,delta_diagonal_rod);
  EEPROM_WRITE_VAR(i,max_pos);
  EEPROM_WRITE_VAR(i,endstop_adj);
  EEPROM_WRITE_VAR(i,tower_adj);
  EEPROM_WRITE_VAR(i,z_probe_offset);
#endif
#ifdef ENABLE_AUTO_BED_LEVELING
  EEPROM_WRITE_VAR(i,zprobe_zoffset);
#endif
#ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  int gumPreheatHotendTemp = GUM_PREHEAT_HOTEND_TEMP, gumPreheatHPBTemp = GUM_PREHEAT_HPB_TEMP, gumPreheatFanSpeed = GUM_PREHEAT_FAN_SPEED;
#endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,gumPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,gumPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,gumPreheatFanSpeed);
#ifdef PIDTEMP
  EEPROM_WRITE_VAR(i,Kp);
  EEPROM_WRITE_VAR(i,Ki);
  EEPROM_WRITE_VAR(i,Kd);
#else
  float dummy = 3000.0f;
  EEPROM_WRITE_VAR(i,dummy);
  dummy = 0.0f;
  EEPROM_WRITE_VAR(i,dummy);
  EEPROM_WRITE_VAR(i,dummy);
#endif
#ifndef DOGLCD
  int lcd_contrast = 32;
#endif
  EEPROM_WRITE_VAR(i,lcd_contrast);
#ifdef SCARA
  EEPROM_WRITE_VAR(i,axis_scaling);        // Add scaling for SCARA
#endif
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("Baudrate: ", baudrate);
    SERIAL_ECHOLN("");
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[X_AXIS]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[Y_AXIS]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[Z_AXIS]);
    SERIAL_ECHOPAIR(" E0 ",axis_steps_per_unit[3]);
    SERIAL_ECHOPAIR(" E1 ",axis_steps_per_unit[4]);
    SERIAL_ECHOPAIR(" E2 ",axis_steps_per_unit[5]);
    SERIAL_ECHOPAIR(" E3 ",axis_steps_per_unit[6]);
    SERIAL_ECHOLN("");
  
    SERIAL_ECHO_START;
#ifdef SCARA
    SERIAL_ECHOLNPGM("Scaling factors:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M365 X",axis_scaling[X_AXIS]);
    SERIAL_ECHOPAIR(" Y",axis_scaling[Y_AXIS]);
    SERIAL_ECHOPAIR(" Z",axis_scaling[Z_AXIS]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
#endif
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X ",max_feedrate[X_AXIS]);
    SERIAL_ECHOPAIR(" Y ",max_feedrate[Y_AXIS] ); 
    SERIAL_ECHOPAIR(" Z ", max_feedrate[Z_AXIS] ); 
    SERIAL_ECHOPAIR(" E0 ", max_feedrate[3]);
    SERIAL_ECHOPAIR(" E1 ", max_feedrate[4]);
    SERIAL_ECHOPAIR(" E2 ", max_feedrate[5]);
    SERIAL_ECHOPAIR(" E3 ", max_feedrate[6]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Retraction Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR(" E0 ",max_retraction_feedrate[0]);
    SERIAL_ECHOPAIR(" E1 ",max_retraction_feedrate[1]);
    SERIAL_ECHOPAIR(" E2 ",max_retraction_feedrate[2]);
    SERIAL_ECHOPAIR(" E3 ",max_retraction_feedrate[3]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X " ,max_acceleration_units_per_sq_second[X_AXIS] ); 
    SERIAL_ECHOPAIR(" Y " , max_acceleration_units_per_sq_second[Y_AXIS] ); 
    SERIAL_ECHOPAIR(" Z " ,max_acceleration_units_per_sq_second[Z_AXIS] );
    SERIAL_ECHOPAIR(" E0 " ,max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOPAIR(" E1 " ,max_acceleration_units_per_sq_second[4]);
    SERIAL_ECHOPAIR(" E2 " ,max_acceleration_units_per_sq_second[5]);
    SERIAL_ECHOPAIR(" E3 " ,max_acceleration_units_per_sq_second[6]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration ); 
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homing[X_AXIS] );
    SERIAL_ECHOPAIR(" Y" ,add_homing[Y_AXIS] );
    SERIAL_ECHOPAIR(" Z" ,add_homing[Z_AXIS] );
    SERIAL_ECHOLN("");
#ifdef DELTA
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Endstop adjustment (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M666 X",endstop_adj[0]);
    SERIAL_ECHOPAIR(" Y" ,endstop_adj[1]);
    SERIAL_ECHOPAIR(" Z" ,endstop_adj[2]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Delta Geometry adjustment:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M666 A",tower_adj[0]);
    SERIAL_ECHOPAIR(" B" ,tower_adj[1]);
    SERIAL_ECHOPAIR(" C" ,tower_adj[2]);
    SERIAL_ECHOPAIR(" E" ,tower_adj[3]);
    SERIAL_ECHOPAIR(" F" ,tower_adj[4]);
    SERIAL_ECHOPAIR(" G" ,tower_adj[5]);
    SERIAL_ECHOPAIR(" R" ,delta_radius);
    SERIAL_ECHOPAIR(" D" ,delta_diagonal_rod);
    SERIAL_ECHOPAIR(" H" ,max_pos[2]);
    SERIAL_ECHOPAIR(" P" ,z_probe_offset[3]);
    SERIAL_ECHOLN("");
  /*
    SERIAL_ECHOLN("Tower Positions");
    SERIAL_ECHOPAIR("Tower1 X:",delta_tower1_x);
    SERIAL_ECHOPAIR(" Y:",delta_tower1_y);
    SERIAL_ECHOLN("");
    SERIAL_ECHOPAIR("Tower2 X:",delta_tower2_x);
    SERIAL_ECHOPAIR(" Y:",delta_tower2_y);
    SERIAL_ECHOLN("");
    SERIAL_ECHOPAIR("Tower3 X:",delta_tower3_x);
    SERIAL_ECHOPAIR(" Y:",delta_tower3_y);
    SERIAL_ECHOLN("");
  */
#endif // DELTA

#ifdef ENABLE_AUTO_BED_LEVELING
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("Z Probe offset (mm):" ,zprobe_zoffset);
    SERIAL_ECHOLN("");
#endif // ENABLE_AUTO_BED_LEVELING

#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp[active_extruder]); 
    SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki[active_extruder])); 
    SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd[active_extruder]));
    SERIAL_ECHOLN(""); 
#endif
#ifdef FWRETRACT
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Retract: S=Length (mm) F:Speed (mm/m) Z: ZLift (mm)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M207 S",retract_length); 
    SERIAL_ECHOPAIR(" F" ,retract_feedrate*60); 
    SERIAL_ECHOPAIR(" Z" ,retract_zlift);
    SERIAL_ECHOLN(""); 
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Recover: S=Extra length (mm) F:Speed (mm/m)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M208 S",retract_recover_length); 
    SERIAL_ECHOPAIR(" F" ,retract_recover_feedrate*60); 
    SERIAL_ECHOLN(""); 
#endif
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
  int i=EEPROM_OFFSET;
  char stored_ver[4];
  char ver[4]=EEPROM_VERSION;
  EEPROM_READ_VAR(i,stored_ver); //read stored version
  //SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
  if (strncmp(ver,stored_ver,3) == 0)
  {
    // version number match
    EEPROM_READ_VAR(i,baudrate);
    if(baudrate!=9600 && baudrate!=14400 && baudrate!=19200 && baudrate!=28800 && baudrate!=38400 && baudrate!=56000 && baudrate!=115200 && baudrate!=250000) baudrate=BAUDRATE;
    EEPROM_READ_VAR(i,axis_steps_per_unit);
    EEPROM_READ_VAR(i,max_feedrate);
    EEPROM_READ_VAR(i,max_retraction_feedrate);
    EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);

    // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
    reset_acceleration_rates();

    EEPROM_READ_VAR(i,acceleration);
    EEPROM_READ_VAR(i,retract_acceleration);
    EEPROM_READ_VAR(i,minimumfeedrate);
    EEPROM_READ_VAR(i,mintravelfeedrate);
    EEPROM_READ_VAR(i,minsegmenttime);
    EEPROM_READ_VAR(i,max_xy_jerk);
    EEPROM_READ_VAR(i,max_z_jerk);
    EEPROM_READ_VAR(i,max_e_jerk);
    EEPROM_READ_VAR(i,add_homing);
#ifdef DELTA
    EEPROM_READ_VAR(i,delta_radius);
    EEPROM_READ_VAR(i,delta_diagonal_rod);
    EEPROM_READ_VAR(i,max_pos);
    EEPROM_READ_VAR(i,endstop_adj);
    EEPROM_READ_VAR(i,tower_adj);
    EEPROM_READ_VAR(i,z_probe_offset);
    // Update delta constants for updated delta_radius & tower_adj values
    set_delta_constants();
#endif
#ifdef ENABLE_AUTO_BED_LEVELING
    EEPROM_READ_VAR(i,zprobe_zoffset);
#endif
#ifndef ULTIPANEL
    int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
    int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
    int gumPreheatHotendTemp, gumPreheatHPBTemp, gumPreheatFanSpeed;
#endif
    EEPROM_READ_VAR(i,plaPreheatHotendTemp);
    EEPROM_READ_VAR(i,plaPreheatHPBTemp);
    EEPROM_READ_VAR(i,plaPreheatFanSpeed);
    EEPROM_READ_VAR(i,absPreheatHotendTemp);
    EEPROM_READ_VAR(i,absPreheatHPBTemp);
    EEPROM_READ_VAR(i,absPreheatFanSpeed);
    EEPROM_READ_VAR(i,gumPreheatHotendTemp);
    EEPROM_READ_VAR(i,gumPreheatHPBTemp);
    EEPROM_READ_VAR(i,gumPreheatFanSpeed);
#ifndef PIDTEMP
    float Kp,Ki,Kd;
#endif
    // do not need to scale PID values as the values in EEPROM are already scaled		
    EEPROM_READ_VAR(i,Kp);
    EEPROM_READ_VAR(i,Ki);
    EEPROM_READ_VAR(i,Kd);
#ifndef DOGLCD
    int lcd_contrast;
#endif
    EEPROM_READ_VAR(i,lcd_contrast);
#ifdef SCARA
    EEPROM_READ_VAR(i,axis_scaling);
#endif

    // Call updatePID (similar to when we have processed M301)
    updatePID();
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Stored settings retrieved");
  }
  else
  {
    Config_ResetDefault();
  }
#ifdef EEPROM_CHITCHAT
  Config_PrintSettings();
#endif
}
#endif

void Config_ResetDefault()
{
  //Setting default baudrate for serial
  baudrate=BAUDRATE;

  float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
  float tmp2[]=DEFAULT_MAX_FEEDRATE;
  float tmp3[]=DEFAULT_RETRACTION_MAX_FEEDRATE;
  long  tmp4[]=DEFAULT_MAX_ACCELERATION;
  float tmp5[]=DEFAULT_Kp;
  float tmp6[]=DEFAULT_Ki;
  float tmp7[]=DEFAULT_Kd;
  
  for (short i=0;i<7;i++) 
  {
    axis_steps_per_unit[i]=tmp1[i];
    max_feedrate[i]=tmp2[i];
    max_acceleration_units_per_sq_second[i]=tmp4[i];
  }

  for (short i=0;i<4;i++)
  {
    max_retraction_feedrate[i]=tmp3[i];
  #ifdef SCARA
    axis_scaling[i]=1;
  #endif
  }

  // steps per sq second need to be updated to agree with the units per sq second
  reset_acceleration_rates();

  acceleration=DEFAULT_ACCELERATION;
  retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
  minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
  minsegmenttime=DEFAULT_MINSEGMENTTIME;       
  mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
  max_xy_jerk=DEFAULT_XYJERK;
  max_z_jerk=DEFAULT_ZJERK;
  max_e_jerk=DEFAULT_EJERK;
  add_homing[0] = add_homing[1] = add_homing[2] = 0;
#ifdef DELTA
  delta_radius = DEFAULT_DELTA_RADIUS;
  delta_diagonal_rod = DEFAULT_DELTA_DIAGONAL_ROD;
  endstop_adj[0] = endstop_adj[1] = endstop_adj[2] = 0;
  tower_adj[0] = tower_adj[1] = tower_adj[2] = tower_adj[3] = tower_adj[4] = tower_adj[5] = 0;
  max_pos[2] = MANUAL_Z_HOME_POS;
  set_default_z_probe_offset();
  set_delta_constants();
#endif
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
#ifdef ENABLE_AUTO_BED_LEVELING
  zprobe_zoffset = -Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif
#ifdef DOGLCD
  lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
  for (short i=0;i<4;i++) 
  {
#ifdef SINGLENOZZLE
    Kp[i] = tmp5[0];
    Ki[i] = scalePID_i(tmp6[0]);
    Kd[i] = scalePID_d(tmp7[0]);
#else
    Kp[i] = tmp5[i];
    Ki[i] = scalePID_i(tmp6[i]);
    Kd[i] = scalePID_d(tmp7[i]);
#endif
  }

  // call updatePID (similar to when we have processed M301)
  updatePID();

#ifdef PID_ADD_EXTRUSION_RATE
  Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}
