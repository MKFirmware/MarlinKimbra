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
 * stepper.cpp - A singleton object to execute motion plans using stepper motors
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
 * and Philipp Tiefenbacher.
 */

#include "../../base.h"
#include "stepper.h"
#include "speed_lookuptable.h"

#if HAS(DIGIPOTSS)
  #include <SPI.h>
#endif

//===========================================================================
//============================= public variables ============================
//===========================================================================
block_t* current_block;  // A pointer to the block currently being traced


//===========================================================================
//============================= private variables ===========================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char last_direction_bits = 0;  // The next stepping-bits to be output
static unsigned int cleaning_buffer_counter = 0;

#if ENABLED(LASERBEAM)
  static long counter_L;
  #if ENABLED(LASER_RASTER)
    static int counter_raster;
  #endif // LASER_RASTER
#endif // LASERBEAM

#if ENABLED(Z_DUAL_ENDSTOPS)
  static bool performing_homing = false,
              locked_z_motor = false,
              locked_z2_motor = false;
#endif

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  #if ENABLED(ABORT_ON_ENDSTOP_HIT_INIT)
    bool abort_on_endstop_hit = ABORT_ON_ENDSTOP_HIT_INIT;
  #else
    bool abort_on_endstop_hit = false;
  #endif
#endif

// Counter variables for the Bresenham line tracer
static long counter_X, counter_Y, counter_Z, counter_E;
volatile unsigned long step_events_completed; // The number of step events executed in the current block
volatile long endstops_trigsteps[3];
volatile long endstops_stepsTotal, endstops_stepsDone;

#if ENABLED(ADVANCE) || ENABLED(ADVANCE_LPC)
  unsigned char old_OCR0A;
  #if ENABLED(ADVANCE)
    static long advance_rate, advance, final_advance = 0;
    static long old_advance = 0;
    static long e_steps[EXTRUDERS];
  #elif ENABLED(ADVANCE_LPC)
    int extruder_advance_k = ADVANCE_LPC_K;
    volatile int e_steps[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
    volatile unsigned char eISR_Rate = 200; // Keep the ISR at a low rate until needed
    static int final_estep_rate;
    static int current_estep_rate[EXTRUDERS]; // Actual extruder speed [steps/s]
    static int current_adv_steps[EXTRUDERS];
  #endif
#endif

static long acceleration_time, deceleration_time;
// static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deceleration start point
static uint8_t step_loops;
static uint8_t step_loops_nominal;
static unsigned short OCR1A_nominal;

#if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
  int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  static long counter_m[DRIVER_EXTRUDERS];
#endif

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0 }; // Positions of stepper motors, in step units
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1 };


//===========================================================================
//================================ functions ================================
//===========================================================================

#if ENABLED(DUAL_X_CARRIAGE)
  #define X_APPLY_DIR(v,ALWAYS) \
    if (hotend_duplication_enabled || ALWAYS) { \
      X_DIR_WRITE(v); \
      X2_DIR_WRITE(v); \
    } \
    else { \
      if (current_block->active_driver) X2_DIR_WRITE(v); else X_DIR_WRITE(v); \
    }
  #define X_APPLY_STEP(v,ALWAYS) \
    if (hotend_duplication_enabled || ALWAYS) { \
      X_STEP_WRITE(v); \
      X2_STEP_WRITE(v); \
    } \
    else { \
      if (current_block->active_driver != 0) X2_STEP_WRITE(v); else X_STEP_WRITE(v); \
    }
#else
  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
#endif

#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  #define Y_APPLY_DIR(v,Q) { Y_DIR_WRITE(v); Y2_DIR_WRITE((v) != INVERT_Y2_VS_Y_DIR); }
  #define Y_APPLY_STEP(v,Q) { Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }
#else
  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
#endif

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #define Z_APPLY_DIR(v,Q) { Z_DIR_WRITE(v); Z2_DIR_WRITE(v); }
  #if ENABLED(Z_DUAL_ENDSTOPS)
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR > 0) {\
        if (!(TEST(old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
      else { \
        if (!(TEST(old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
    } \
    else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) { Z_STEP_WRITE(v); Z2_STEP_WRITE(v); }
  #endif
#else
  #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
  #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)
#endif

#if DISABLED(COLOR_MIXING_EXTRUDER)
  #define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)
#else
  #define E_APPLY_STEP(v,Q)
#endif

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %A1, %A2 \n\t" \
                 "add %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r0 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (charIn1), \
                 "d" (intIn2) \
                 : \
                 "r26" \
               )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */
void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;

  NOMORE(step_rate, MAX_STEP_FREQUENCY);

  if(step_rate > (2 * DOUBLE_STEP_FREQUENCY)) { // If steprate > 2*DOUBLE_STEP_FREQUENCY >> step 4 times
    step_rate >>= 2;
    step_loops = 4;
  }
  else if(step_rate > DOUBLE_STEP_FREQUENCY) { // If steprate > DOUBLE_STEP_FREQUENCY >> step 2 times
    step_rate >>= 1;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  NOLESS(step_rate, F_CPU / 500000);
  step_rate -= F_CPU / 500000; // Correct for minimal speed
  if (step_rate >= (8 * 256)) { // higher step rate
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address + 2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate) >> 1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
  }

  if (timer < 100) { // (20kHz this should never happen)
    timer = 100;
    ECHO_EMV(SERIAL_STEPPER_TOO_HIGH, step_rate);
  }

  return timer;
}

/**
 * Set the stepper direction of each axis
 *
 *   X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY or COREYX
 *   X_AXIS=A_AXIS and Z_AXIS=C_AXIS for COREXZ or COREZX
 */
void set_stepper_direction(bool onlye) {

  #define SET_STEP_DIR(AXIS) \
    if (motor_direction(AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }

  if (!onlye) {
    SET_STEP_DIR(X); // A
    SET_STEP_DIR(Y); // B
    SET_STEP_DIR(Z); // C
  }

  #if DISABLED(ADVANCE)
    if (motor_direction(E_AXIS)) {
      REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif // !ADVANCE
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {

  static int8_t last_driver = -1;

  if (current_block->direction_bits != last_direction_bits || current_block->active_driver != last_driver) {
    last_direction_bits = current_block->direction_bits;
    last_driver = current_block->active_driver;
    set_stepper_direction();
  }

  #if ENABLED(ADVANCE)
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_driver] += ((advance >> 8) - old_advance);
    old_advance = advance >>8;
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;

  #if ENABLED(ADVANCE_LPC)
    if (current_block->use_advance_lead) {
      current_estep_rate[current_block->active_driver] = ((unsigned long)acc_step_rate * current_block->e_speed_multiplier8) >> 8;
      final_estep_rate = (current_block->nominal_rate * current_block->e_speed_multiplier8) >> 8;
    }
  #endif
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
ISR(TIMER1_COMPA_vect) {

  if (cleaning_buffer_counter) {
    current_block = NULL;
    planner.discard_current_block();
    #if ENABLED(SD_FINISHED_RELEASECOMMAND)
      if ((cleaning_buffer_counter == 1) && (SD_FINISHED_STEPPERRELEASE)) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    cleaning_buffer_counter--;
    OCR1A = 200;
    return;
  }

  #if ENABLED(LASERBEAM) && (!ENABLED(LASER_PULSE_METHOD))
    if (laser.dur != 0 && (laser.last_firing + laser.dur < micros())) {
      if (laser.diagnostics)
        ECHO_LM(INFO, "Laser firing duration elapsed, in interrupt handler");

      laser_extinguish();
    }
  #endif

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = planner.get_current_block();
    if (current_block) {
      current_block->busy = true;
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);

      #if ENABLED(LASERBEAM)
         counter_L = counter_X;
         #if !ENABLED(LASER_PULSE_METHOD)
           laser.dur = current_block->laser_duration;
         #endif
      #endif

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++)
          counter_m[i] = -(current_block->step_event_count >> 1);
      #endif

      step_events_completed = 0;

      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_z();
          OCR1A = 2000; // 1ms wait
          return;
        }
      #endif

      #if ENABLED(LASERBEAM) && ENABLED(LASER_RASTER)
         if (current_block->laser_mode == RASTER) counter_raster = 0;
      #endif

      // #if ENABLED(ADVANCE)
      //   e_steps[current_block->active_driver] = 0;
      // #endif
    }
    else {
      OCR1A = 2000; // 1kHz
    }
  }

  if (current_block != NULL) {

    // Update endstops state, if enabled
    #if HAS(BED_PROBE)
      if (endstops.enabled || endstops.z_probe_enabled) endstops.update();
    #else
      if (endstops.enabled) endstops.update();
    #endif

    // Continuous firing of the laser during a move happens here, PPM and raster happen further down
    #if ENABLED(LASERBEAM)
      if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
        laser_fire(current_block->laser_intensity);

      #if !ENABLED(LASER_PULSE_METHOD)
        if (current_block->laser_status == LASER_OFF) {
          if (laser.diagnostics) ECHO_LM(INFO,"Laser status set to off, in interrupt handler");
          laser_extinguish();
        }
      #endif
    #endif

    // Take multiple steps per interrupt (For high speed moves)
    for (uint8_t i = 0; i < step_loops; i++) {

        MKSERIAL.checkRx(); // Check for serial chars.

      #if ENABLED(ADVANCE)
        counter_E += current_block->steps[E_AXIS];
        if (counter_E > 0) {
          counter_E -= current_block->step_event_count;
          #if DISABLED(COLOR_MIXING_EXTRUDER)
            // Don't step E for mixing extruder
            e_steps[current_block->active_driver] += motor_direction(E_AXIS) ? -1 : 1;
          #endif
        }

        #if ENABLED(COLOR_MIXING_EXTRUDER)
          long dir = motor_direction(E_AXIS) ? -1 : 1;
          for (uint8_t j = 0; j < DRIVER_EXTRUDERS; j++) {
            counter_m[j] += current_block->steps[E_AXIS];
            if (counter_m[j] > 0) {
              counter_m[j] -= current_block->mix_event_count[j];
              e_steps[j] += dir;
            }
          }
        #endif // !COLOR_MIXING_EXTRUDER
      #elif ENABLED(ADVANCE_LPC) // ADVANCE_LPC
        counter_E += current_block->steps[E_AXIS];
        if (counter_E > 0) {
          counter_E -= current_block->step_event_count;
          count_position[E_AXIS] += count_direction[E_AXIS];
          e_steps[current_block->active_driver] += motor_direction(E_AXIS) ? -1 : 1;
        }

        if (current_block->use_advance_lead) {
          int delta_adv_steps; // Maybe a char would be enough?
          delta_adv_steps = (((long)extruder_advance_k * current_estep_rate[current_block->active_driver]) >> 9) - current_adv_steps[current_block->active_driver];
          e_steps[current_block->active_driver] += delta_adv_steps;
          current_adv_steps[current_block->active_driver] += delta_adv_steps;
        }
      #endif

      #define _COUNTER(AXIS) counter_## AXIS
      #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
      #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

      #define STEP_START(AXIS) \
        _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(AXIS) > 0) _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0);

      #define STEP_START_MIXING \
        for (uint8_t j = 0; j < DRIVER_EXTRUDERS; j++) {  \
          counter_m[j] += current_block->mix_event_count[j];  \
          if (counter_m[j] > 0) En_STEP_WRITE(j, !INVERT_E_STEP_PIN); \
        }

      #define STEP_END(AXIS) \
        if (_COUNTER(AXIS) > 0) { \
          _COUNTER(AXIS) -= current_block->step_event_count; \
          count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
          _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
        }

      #define STEP_END_MIXING \
        for (uint8_t j = 0; j < DRIVER_EXTRUDERS; j++) {  \
          if (counter_m[j] > 0) { \
            counter_m[j] -= current_block->step_event_count;  \
            En_STEP_WRITE(j, INVERT_E_STEP_PIN);  \
          } \
        }

      STEP_START(X);
      STEP_START(Y);
      STEP_START(Z);
      #if DISABLED(ADVANCE) && DISABLED(ADVANCE_LPC)
        STEP_START(E);
        #if ENABLED(COLOR_MIXING_EXTRUDER)
          STEP_START_MIXING;
        #endif
      #endif

      #if ENABLED(STEPPER_HIGH_LOW) && STEPPER_HIGH_LOW_DELAY > 0
        HAL::delayMicroseconds(STEPPER_HIGH_LOW_DELAY);
      #endif

      STEP_END(X);
      STEP_END(Y);
      STEP_END(Z);
      #if DISABLED(ADVANCE) && DISABLED(ADVANCE_LPC)
        STEP_END(E);
        #if ENABLED(COLOR_MIXING_EXTRUDER)
          STEP_END_MIXING;
        #endif
      #endif

      #if ENABLED(LASERBEAM)
        counter_L += current_block->steps_l;
        if (counter_L > 0) {
          if (current_block->laser_mode == PULSED && current_block->laser_status == LASER_ON) { // Pulsed Firing Mode
            #if ENABLED(LASER_PULSE_METHOD)
              uint32_t ulValue = current_block->laser_raster_intensity_factor * 255;
              laser_pulse(ulValue, current_block->laser_duration);
              laser.time += current_block->laser_duration / 1000; 
            #else
              laser_fire(current_block->laser_intensity);
            #endif
            if (laser.diagnostics) {
              ECHO_MV("X: ", counter_X);
              ECHO_MV("Y: ", counter_Y);
              ECHO_MV("L: ", counter_L);
            }
          }
          #if ENABLED(LASER_RASTER)
            if (current_block->laser_mode == RASTER && current_block->laser_status == LASER_ON) { // Raster Firing Mode
              #if ENABLED(LASER_PULSE_METHOD)
                uint32_t ulValue = current_block->laser_raster_intensity_factor * 
                                   current_block->laser_raster_data[counter_raster];
                laser_pulse(ulValue, current_block->laser_duration);
                counter_raster++;
                laser.time += current_block->laser_duration/1000; 
              #else
                // For some reason, when comparing raster power to ppm line burns the rasters were around 2% more powerful
                // going from darkened paper to burning through paper.
                laser_fire(current_block->laser_raster_data[counter_raster]); 
              #endif
              if (laser.diagnostics) ECHO_EMV("Pixel: ", (float)current_block->laser_raster_data[counter_raster]);
              counter_raster++;
            }
          #endif // LASER_RASTER
          counter_L -= current_block->step_event_count;
        }
        #if !ENABLED(LASER_PULSE_METHOD)
        if (current_block->laser_duration != 0 && (laser.last_firing + current_block->laser_duration < micros())) {
          if (laser.diagnostics) {
            ECHO_MV("X: ", counter_X);
            ECHO_MV(", Y: ", counter_Y);
            ECHO_MV(", L: ", counter_L);
            ECHO_MV(", Z: ", counter_L);
            ECHO_MV(", E: ", counter_E);
            ECHO_MV(", steps done: ",step_events_completed);
            ECHO_MV(", event count: ", current_block->step_event_count);
            ECHO_EM(", <--------------------");
            ECHO_LM(INFO, "Laser firing duration elapsed, in interrupt fast loop ");
			 }
          laser_extinguish();
        }
        #endif
      #endif // LASERBEAM

      step_events_completed++;
      if (step_events_completed >= current_block->step_event_count) break;
    }

    #if ENABLED(ADVANCE_LPC)
      // If we have esteps to execute, fire the next ISR "now"
      if (e_steps[current_block->active_driver]) OCR0A = TCNT0 + 2;
    #endif

    // Calculate new timer value
    unsigned short timer, step_rate;
    if (step_events_completed <= (unsigned long)current_block->accelerate_until) {

      MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      NOMORE(acc_step_rate, current_block->nominal_rate);

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;

      #if ENABLED(ADVANCE)

        advance += advance_rate * step_loops;
        //NOLESS(advance, current_block->advance);

        // Do E steps + advance steps
        #if ENABLED(COLOR_MIXING_EXTRUDER)
          // Move mixing steppers proportionally
          for (uint8_t j = 0; j < DRIVER_EXTRUDERS; j++)
            e_steps[j] += ((advance >> 8) - old_advance) * current_block->step_event_count / current_block->mix_event_count[j];
        #else
          e_steps[current_block->active_driver] += ((advance >> 8) - old_advance);
        #endif

        old_advance = advance >> 8;
      #elif ENABLED(ADVANCE_LPC) // ADVANCE_LPC
        if (current_block->use_advance_lead)
          current_estep_rate[current_block->active_driver] = ((unsigned long)acc_step_rate * current_block->e_speed_multiplier8) >> 8;
      #endif

      #if ENABLED(ADVANCE) || ENABLED(ADVANCE_LPC)
        eISR_Rate = (timer >> 2) / e_steps[current_block->active_driver];
      #endif
    }
    else if (step_events_completed > (unsigned long)current_block->decelerate_after) {
      MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if (step_rate <= acc_step_rate) {
        step_rate = acc_step_rate - step_rate; // Decelerate from acceleration end point.
        NOLESS(step_rate, current_block->final_rate);
      }
      else {
        step_rate = current_block->final_rate;
      }

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;

      #if ENABLED(ADVANCE)
        advance -= advance_rate * step_loops;
        NOLESS(advance, final_advance);

        // Do E steps + advance steps
        uint32_t advance_whole = advance >> 8;

        #if ENABLED(MIXING_EXTRUDER_FEATURE)
          for (uint8_t j = 0; j < DRIVER_EXTRUDERS; j++)
            e_steps[current_block->active_driver] += (advance_whole - old_advance) * current_block->mix_factor[j];
        #else
          e_steps[current_block->active_driver] += advance_whole - old_advance;
        #endif

        old_advance = advance_whole;
      #elif ENABLED(ADVANCE_LPC) // ADVANCE_LPC
        if (current_block->use_advance_lead)
          current_estep_rate[current_block->active_driver] = ((unsigned long)step_rate * current_block->e_speed_multiplier8) >> 8;
      #endif

      #if ENABLED(ADVANCE) || ENABLED(ADVANCE_LPC)
        eISR_Rate = (timer >> 2) / e_steps[current_block->active_driver];
      #endif
    }
    else {
      #if ENABLED(ADVANCE_LPC)
        if (current_block->use_advance_lead)
          current_estep_rate[current_block->active_driver] = final_estep_rate;
        eISR_Rate = (OCR1A_nominal >> 2) / e_steps[current_block->active_driver];
      #endif

      OCR1A = OCR1A_nominal;
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    OCR1A = (OCR1A < (TCNT1 + 16)) ? (TCNT1 + 16) : OCR1A;

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      planner.discard_current_block();
      #if ENABLED(LASERBEAM) && ENABLED(LASER_PULSE_METHOD)
        if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
          laser_extinguish();
      #endif
    }
  }
}

#if ENABLED(ADVANCE) || ENABLED(ADVANCE_LPC)

  // Timer interrupt for E. e_steps is set in the main routine;
  // Timer 0 is shared with millies
  ISR(TIMER0_COMPA_vect) {

    old_OCR0A += eISR_Rate;
    OCR0A = old_OCR0A;

    #define STEP_E_ONCE(INDEX) \
      if (e_steps[INDEX] != 0) { \
        E## INDEX ##_STEP_WRITE(INVERT_E_STEP_PIN); \
        if (e_steps[INDEX] < 0) { \
          E## INDEX ##_DIR_WRITE(INVERT_E## INDEX ##_DIR); \
          e_steps[INDEX]++; \
        } \
        else if (e_steps[INDEX] > 0) { \
          E## INDEX ##_DIR_WRITE(!INVERT_E## INDEX ##_DIR); \
          e_steps[INDEX]--; \
        } \
        E## INDEX ##_STEP_WRITE(!INVERT_E_STEP_PIN); \
      }

    // Step all E steppers that have steps
    STEP_E_ONCE(0);
    #if EXTRUDERS > 1
      STEP_E_ONCE(1);
      #if EXTRUDERS > 2
        STEP_E_ONCE(2);
        #if EXTRUDERS > 3
          STEP_E_ONCE(3);
          #if EXTRUDERS > 4
            STEP_E_ONCE(4);
            #if EXTRUDERS > 5
              STEP_E_ONCE(5);
            #endif
          #endif
        #endif
      #endif
    #endif
  }
#endif

void st_init() {
  digipot_init(); //Initialize Digipot Motor Current
  microstep_init(); //Initialize Microstepping Pins

  // initialise TMC Steppers
  #if ENABLED(HAVE_TMCDRIVER)
    tmc_init();
  #endif
    // initialise L6470 Steppers
  #if ENABLED(HAVE_L6470DRIVER)
    L6470_init();
  #endif

  // Initialize Dir Pins
  #if HAS(X_DIR)
    X_DIR_INIT;
  #endif
  #if HAS(X2_DIR)
    X2_DIR_INIT;
  #endif
  #if HAS(Y_DIR)
    Y_DIR_INIT;
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS(Y2_DIR)
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS(Z_DIR)
    Z_DIR_INIT;
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS(Z2_DIR)
      Z2_DIR_INIT;
    #endif
  #endif
  #if HAS(E0_DIR)
    E0_DIR_INIT;
  #endif
  #if HAS(E1_DIR)
    E1_DIR_INIT;
  #endif
  #if HAS(E2_DIR)
    E2_DIR_INIT;
  #endif
  #if HAS(E3_DIR)
    E3_DIR_INIT;
  #endif
  #if HAS(E4_DIR)
    E4_DIR_INIT;
  #endif
  #if HAS(E5_DIR)
    E5_DIR_INIT;
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if HAS(X_ENABLE)
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(X2_ENABLE)
    X2_ENABLE_INIT;
    if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(Y_ENABLE)
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);

  #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS(Y2_ENABLE)
    Y2_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
  #endif
  #endif
  #if HAS(Z_ENABLE)
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);

    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS(Z2_ENABLE)
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS(E0_ENABLE)
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E1_ENABLE)
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E2_ENABLE)
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E3_ENABLE)
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E4_ENABLE)
    E4_ENABLE_INIT;
    if (!E_ENABLE_ON) E4_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E5_ENABLE)
    E5_ENABLE_INIT;
    if (!E_ENABLE_ON) E5_ENABLE_WRITE(HIGH);
  #endif

  //Choice E0-E1 or E0-E2 or E1-E3 pin
  #if ENABLED(MKR4) && HAS(E0E1)
    OUT_WRITE_RELE(E0E1_CHOICE_PIN, LOW);
  #endif
  #if ENABLED(MKR4) && HAS(E0E2)
    OUT_WRITE_RELE(E0E2_CHOICE_PIN, LOW);
  #endif
  #if ENABLED(MKR4) && HAS(E0E3)
    OUT_WRITE_RELE(E0E3_CHOICE_PIN, LOW);
  #endif
  #if ENABLED(MKR4) && HAS(E0E4)
    OUT_WRITE_RELE(E0E4_CHOICE_PIN, LOW);
  #endif
  #if ENABLED(MKR4) && HAS(E0E5)
    OUT_WRITE_RELE(E0E5_CHOICE_PIN, LOW);
  #endif
  #if ENABLED(MKR4) && HAS(E1E3)
    OUT_WRITE_RELE(E1E3_CHOICE_PIN, LOW);
  #endif

  //
  // Init endstops and pullups here
  //
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  #define E_AXIS_INIT(NUM) AXIS_INIT(e## NUM, E## NUM, E)

  // Initialize Step Pins
  #if HAS(X_STEP)
    AXIS_INIT(x, X, X);
  #endif
  #if HAS(X2_STEP)
    AXIS_INIT(x, X2, X);
  #endif
  #if HAS(Y_STEP)
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS(Y2_STEP)
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(y, Y, Y);
  #endif
  #if HAS(Z_STEP)
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS(Z2_STEP)
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(z, Z, Z);
  #endif
  #if HAS(E0_STEP)
    E_AXIS_INIT(0);
  #endif
  #if HAS(E1_STEP)
    E_AXIS_INIT(1);
  #endif
  #if HAS(E2_STEP)
    E_AXIS_INIT(2);
  #endif
  #if HAS(E3_STEP)
    E_AXIS_INIT(3);
  #endif
  #if HAS(E4_STEP)
    E_AXIS_INIT(4);
  #endif
  #if HAS(E5_STEP)
    E_AXIS_INIT(5);
  #endif

  // waveform generation = 0100 = CTC
  CBI(TCCR1B, WGM13);
  SBI(TCCR1B, WGM12);
  CBI(TCCR1A, WGM11);
  CBI(TCCR1A, WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #if ENABLED(ADVANCE) || ENABLED(ADVANCE_LPC)
    #if ENABLED(ADVANCE)
      for (uint8_t i = 0; i < EXTRUDERS; i++) e_steps[i] = 0;
    #elif ENABLED(ADVANCE_LPC)
      for (uint8_t i = 0; i < EXTRUDERS; i++) {
        e_steps[i] = 0;
        current_adv_steps[i] = 0;
      }
    #endif
      
    #if defined(TCCR0A) && defined(WGM01)
      CBI(TCCR0A, WGM01);
      CBI(TCCR0A, WGM00);
    #endif
    SBI(TIMSK0, OCIE0A);

  #endif // ADVANCE or ADVANCE_LPC

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_stepper_direction(); // Init directions to last_direction_bits = 0
}


/**
 * Block until all buffered steps are executed
 */
void st_synchronize() { while (planner.blocks_queued()) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void st_set_position(const long& x, const long& y, const long& z, const long& e) {
  CRITICAL_SECTION_START;

  #if MECH(COREXY)
    // corexy positioning
    count_position[A_AXIS] = x + COREX_YZ_FACTOR * y;
    count_position[B_AXIS] = x - COREX_YZ_FACTOR * y;
    count_position[Z_AXIS] = z;
  #elif MECH(COREYX)
    // coreyx positioning
    count_position[A_AXIS] = y + COREX_YZ_FACTOR * x;
    count_position[B_AXIS] = y - COREX_YZ_FACTOR * x;
    count_position[Z_AXIS] = z;
  #elif MECH(COREXZ)
    // corexz planning
    count_position[A_AXIS] = x + COREX_YZ_FACTOR * z;
    count_position[Y_AXIS] = y;
    count_position[C_AXIS] = x - COREX_YZ_FACTOR * z;
  #elif MECH(COREZX)
    // corezx planning
    count_position[A_AXIS] = z + COREX_YZ_FACTOR * x;
    count_position[Y_AXIS] = y;
    count_position[C_AXIS] = z - COREX_YZ_FACTOR * x;
  #else
    // default non-h-bot planning
    count_position[X_AXIS] = x;
    count_position[Y_AXIS] = y;
    count_position[Z_AXIS] = z;
  #endif

  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long& e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis) {
  CRITICAL_SECTION_START;
  long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

float st_get_axis_position_mm(AxisEnum axis) {
  float axis_pos;
  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      long  pos1 = count_position[CORE_AXIS_1],
            pos2 = count_position[CORE_AXIS_2];
      CRITICAL_SECTION_END;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_pos = (pos1 + ((axis == CORE_AXIS_1) ? pos2 : -pos2)) / 2.0f;
    }
    else
      axis_pos = st_get_position(axis);
  #else
    axis_pos = st_get_position(axis);
  #endif

  return axis_pos / planner.axis_steps_per_mm[axis];
}

void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}

void finishAndDisableSteppers() {
  st_synchronize();
  disable_all_steppers();
}

void quickStop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void endstop_triggered(AxisEnum axis) {

  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)

    float axis_pos = count_position[axis];
    if (axis == CORE_AXIS_1)
      axis_pos = (axis_pos + count_position[CORE_AXIS_2]) / 2;
    else if (axis == CORE_AXIS_2)
      axis_pos = (count_position[CORE_AXIS_1] - axis_pos) / 2;
    endstops_trigsteps[axis] = axis_pos;

  #else // ! COREXY || COREYX || COREXZ || COREZX

    endstops_trigsteps[axis] = count_position[axis];

  #endif // ! COREXY || COREYX || COREXZ || COREZX

  kill_current_block();
}

void report_positions() {
  CRITICAL_SECTION_START;
  long xpos = count_position[X_AXIS],
       ypos = count_position[Y_AXIS],
       zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;

  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    ECHO_M(SERIAL_COUNT_A);
  #elif MECH(DELTA)
    ECHO_M(SERIAL_COUNT_ALPHA);
  #else
    ECHO_M(SERIAL_COUNT_X);
  #endif
  ECHO_V(xpos);

  #if MECH(COREXY) || MECH(COREYX)
    ECHO_M(" B:");
  #elif MECH(DELTA)
    ECHO_M(" Beta:");
  #else
    ECHO_M(" Y:");
  #endif
  ECHO_V(ypos);

  #if MECH(COREXZ) || MECH(COREZX)
    ECHO_M(" C:");
  #elif MECH(DELTA)
    ECHO_M(" Teta:");
  #else
    ECHO_M(" Z:");
  #endif
  ECHO_V(zpos);

  ECHO_E;
}

void kill_current_block() {
  step_events_completed = current_block->step_event_count;
}

float triggered_position_mm(AxisEnum axis) {
  return endstops_trigsteps[axis] / planner.axis_steps_per_mm[axis];
}

bool motor_direction(AxisEnum axis) { return TEST(last_direction_bits, axis); }

#if ENABLED(NPR2)
  void colorstep(long csteps,const bool direction) {
    enable_e1();
    //setup new step
    WRITE(E1_DIR_PIN,(INVERT_E1_DIR)^direction);
    //perform step
    for(long i=0; i<=csteps; i++){
      WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
      delayMicroseconds(COLOR_SLOWRATE);
      WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
      delayMicroseconds(COLOR_SLOWRATE);
    }
  }  
#endif //NPR2

#if ENABLED(BABYSTEPPING)

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void babystep(const uint8_t axis, const bool direction) {

    #define _ENABLE(axis) enable_## axis()
    #define _READ_DIR(AXIS) AXIS ##_DIR_READ
    #define _INVERT_DIR(AXIS) INVERT_## AXIS ##_DIR
    #define _APPLY_DIR(AXIS, INVERT) AXIS ##_APPLY_DIR(INVERT, true)

    #define BABYSTEP_AXIS(axis, AXIS, INVERT) { \
        _ENABLE(axis); \
        uint8_t old_pin = _READ_DIR(AXIS); \
        _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^direction^INVERT); \
        _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true); \
        HAL::delayMicroseconds(2); \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true); \
        _APPLY_DIR(AXIS, old_pin); \
      }

    switch (axis) {

      case X_AXIS:
        BABYSTEP_AXIS(x, X, false);
        break;

      case Y_AXIS:
        BABYSTEP_AXIS(y, Y, false);
        break;

      case Z_AXIS: {

        #if !MECH(DELTA)

          BABYSTEP_AXIS(z, Z, BABYSTEP_INVERT_Z);

        #else // DELTA

          bool z_direction = direction ^ BABYSTEP_INVERT_Z;

          enable_x();
          enable_y();
          enable_z();
          uint8_t old_x_dir_pin = X_DIR_READ,
                  old_y_dir_pin = Y_DIR_READ,
                  old_z_dir_pin = Z_DIR_READ;
          //setup new step
          X_DIR_WRITE(INVERT_X_DIR ^ z_direction);
          Y_DIR_WRITE(INVERT_Y_DIR ^ z_direction);
          Z_DIR_WRITE(INVERT_Z_DIR ^ z_direction);
          // perform step
          X_STEP_WRITE(!INVERT_X_STEP_PIN);
          Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          HAL::delayMicroseconds(1U);
          X_STEP_WRITE(INVERT_X_STEP_PIN);
          Y_STEP_WRITE(INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);
          //get old pin state back.
          X_DIR_WRITE(old_x_dir_pin);
          Y_DIR_WRITE(old_y_dir_pin);
          Z_DIR_WRITE(old_z_dir_pin);

        #endif

      } break;

      default: break;
    }
  }

#endif //BABYSTEPPING

// From Arduino DigitalPotControl example
void digitalPotWrite(int address, int value) {
  #if HAS(DIGIPOTSS)
    digitalWrite(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    digitalWrite(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //HAL::delayMilliseconds(10);
  #else
    UNUSED(address);
    UNUSED(value);
  #endif
}

// Initialize Digipot Motor Current
void digipot_init() {
  #if HAS(DIGIPOTSS)
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

    SPI.begin();
    pinMode(DIGIPOTSS_PIN, OUTPUT);
    for (int i = 0; i <= 4; i++) {
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i, digipot_motor_current[i]);
    }
  #endif
  #if HAS(MOTOR_CURRENT_PWM_XY)
    pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
    digipot_current(0, motor_current_setting[0]);
    digipot_current(1, motor_current_setting[1]);
    digipot_current(2, motor_current_setting[2]);
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif

  #if MB(ALLIGATOR)
    set_driver_current();
  #endif // MB(ALLIGATOR)
}

#if MB(ALLIGATOR)
  void set_driver_current() {
    uint8_t digipot_motor = 0;
    for (uint8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++) {
      digipot_motor = 255 * motor_current[i] / 3.3;
      ExternalDac::setValue(i, digipot_motor);
    }
  }
#endif

void digipot_current(uint8_t driver, int current) {
  #if HAS(DIGIPOTSS)
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
  #elif HAS(MOTOR_CURRENT_PWM_XY)
    switch (driver) {
      case 0: analogWrite(MOTOR_CURRENT_PWM_XY_PIN, 255L * current / MOTOR_CURRENT_PWM_RANGE); break;
      case 1: analogWrite(MOTOR_CURRENT_PWM_Z_PIN, 255L * current / MOTOR_CURRENT_PWM_RANGE); break;
      case 2: analogWrite(MOTOR_CURRENT_PWM_E_PIN, 255L * current / MOTOR_CURRENT_PWM_RANGE); break;
    }
  #else
    UNUSED(driver);
    UNUSED(current);
  #endif
}

void microstep_init() {
  #if HAS(MICROSTEPS_E1)
    pinMode(E1_MS1_PIN, OUTPUT);
    pinMode(E1_MS2_PIN, OUTPUT);
  #endif

  #if HAS(MICROSTEPS)
    pinMode(X_MS1_PIN, OUTPUT);
    pinMode(X_MS2_PIN, OUTPUT);
    pinMode(Y_MS1_PIN, OUTPUT);
    pinMode(Y_MS2_PIN, OUTPUT);
    pinMode(Z_MS1_PIN, OUTPUT);
    pinMode(Z_MS2_PIN, OUTPUT);
    pinMode(E0_MS1_PIN, OUTPUT);
    pinMode(E0_MS2_PIN, OUTPUT);
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  #endif
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2) {
  if (ms1 >= 0) switch (driver) {
    case 0: digitalWrite(X_MS1_PIN, ms1); break;
    case 1: digitalWrite(Y_MS1_PIN, ms1); break;
    case 2: digitalWrite(Z_MS1_PIN, ms1); break;
    case 3: digitalWrite(E0_MS1_PIN, ms1); break;
    #if HAS(MICROSTEPS_E1)
      case 4: digitalWrite(E1_MS1_PIN, ms1); break;
    #endif
  }
  if (ms2 >= 0) switch (driver) {
    case 0: digitalWrite(X_MS2_PIN, ms2); break;
    case 1: digitalWrite(Y_MS2_PIN, ms2); break;
    case 2: digitalWrite(Z_MS2_PIN, ms2); break;
    case 3: digitalWrite(E0_MS2_PIN, ms2); break;
    #if PIN_EXISTS(E1_MS2)
      case 4: digitalWrite(E1_MS2_PIN, ms2); break;
    #endif
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode) {
  switch (stepping_mode) {
    case 1: microstep_ms(driver,  MICROSTEP1); break;
    case 2: microstep_ms(driver,  MICROSTEP2); break;
    case 4: microstep_ms(driver,  MICROSTEP4); break;
    case 8: microstep_ms(driver,  MICROSTEP8); break;
    case 16: microstep_ms(driver, MICROSTEP16); break;
    #if MB(ALLIGATOR)
      case 32: microstep_ms(driver, MICROSTEP32); break;
    #endif
  }
}

void microstep_readings() {
  ECHO_SM(DB, SERIAL_MICROSTEP_MS1_MS2);
  ECHO_M(SERIAL_MICROSTEP_X);
  ECHO_V(digitalRead(X_MS1_PIN));
  ECHO_EV(digitalRead(X_MS2_PIN));
  ECHO_SM(DB, SERIAL_MICROSTEP_Y);
  ECHO_V(digitalRead(Y_MS1_PIN));
  ECHO_EV(digitalRead(Y_MS2_PIN));
  ECHO_SM(DB, SERIAL_MICROSTEP_Z);
  ECHO_V(digitalRead(Z_MS1_PIN));
  ECHO_EV(digitalRead(Z_MS2_PIN));
  ECHO_SM(DB, SERIAL_MICROSTEP_E0);
  ECHO_V(digitalRead(E0_MS1_PIN));
  ECHO_EV(digitalRead(E0_MS2_PIN));
  #if HAS(MICROSTEPS_E1)
    ECHO_SM(DB, SERIAL_MICROSTEP_E1);
    ECHO_V(digitalRead(E1_MS1_PIN));
    ECHO_EV(digitalRead(E1_MS2_PIN));
  #endif
}

#if ENABLED(Z_DUAL_ENDSTOPS)
  void set_homing_flag(bool state) { performing_homing = state; }
  void set_z_lock(bool state) { locked_z_motor = state; }
  void set_z2_lock(bool state) { locked_z2_motor = state; }
#endif
