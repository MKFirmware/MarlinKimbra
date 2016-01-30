#ifndef DIGIPOT_MCP4451_H
#define DIGIPOT_MCP4451_H

// Settings for the I2C based DIGIPOT (MCP4451) on Azteeg X3 Pro
#if MB(5DPRINT)
  #define DIGIPOT_I2C_FACTOR 117.96
  #define DIGIPOT_I2C_MAX_CURRENT 1.736
#else
  #define DIGIPOT_I2C_FACTOR 106.7
  #define DIGIPOT_I2C_MAX_CURRENT 2.5
#endif

static byte current_to_wiper(float current);
static void i2c_send(byte addr, byte a, byte b);
// This is for the MCP4451 I2C based digipot
void digipot_i2c_set_current(int channel, float current);
void digipot_i2c_init();

#endif