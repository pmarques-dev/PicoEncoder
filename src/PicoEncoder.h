/*
  PicoEncoder - High resolution quadrature encoder using the PIO on the RP2040
  Created by Paulo Marques, Pedro Pereira, Paulo Costa, 2022
  Distributed under the BSD 2-clause license. For full terms, please refer to the LICENSE file.
*/
#ifndef PicoEncoder_h
#define PicoEncoder_h

#include "Arduino.h"

#ifdef ARDUINO_ARCH_RP2040

#include "hardware/pio.h"

// constant used to represent 4 phases with exactly the same size, which is the
// default used before any phase calibration is applied
#define DEFAULT_PHASES 0x404040

class PicoEncoder
{
  public:
    // call update() to update these fields:

    // current encoder speed in "substeps per second"
    int speed;

    // current encoder position in substeps. Note: to get a good speed estimate
    // it's always better to use the "speed" field than to try and compute
    // differences in position between samples
    int position;

    // current position in raw quadrature steps. This is useful to handle steps
    // in rotary encoders, for instance
    uint step;


    // the constructor just initializes some internal fields
    PicoEncoder();

    // initialize and start the PIO code to track the encoder. The two phases of
    // the encoder must be connected to consecutive pins and "firstPin" is the
    // lowest numbered pin using arduino naming. Note that the pins must be
    // consecutive in the RP2040 and may not correspond to consecutive numbers
    // in the arduino mapping. For instance, on the Arduino Nano RP2040 board,
    // pins D2 and D3 are GPIO25 and GPIO15, so they can not be used for this
    // purpose. However, pin D4 is GPIO16, which means pins D3 and D4 could be
    // used to connect the encoder by passing p3 as "firstPin" (and actually all
    // pins up to D9 are all consecutive). The method also sets the pins as
    // inputs and will turn on the pull-ups on the pins if "pullUp" is true.
    // Many encoders have open-collector outputs and require pull-ups. If
    // unsure, leave the default value of true.
    //
    // Returns 0 on success, -1 if there is no PIO available, -2 if there are
    // too many encoder instances
    int begin(int firstPin, bool pullUp = true);

    // read the encoder state and update "speed", "position" and "step". As a
    // rule of thumb, for best efficiency, call it once in your control loop
    // just before checking the values of "speed", "position" and/or "step".
    void update(void);


    // incrementally update the phase measurements, so that the substep
    // estimation takes phase sizes into account. This method should be called
    // at high frequency to be able to measure step sizes (see the SpeedMeasure
    // example code for a usage scenario)
    void autoCalibratePhases(void);

    // return true if the phase auto calibration is ready
    int autoCalibrationDone(void) {
      return (calib_count >= 128);
    }

    // get the current phase sizes. The user may save this value and pass it to
    // setPhases to immediately start using calibrated values without having to
    // call "autoCalibratePhases"
    int getPhases(void);

    // set the phase sizes using the result from a previous calibration
    void setPhases(int phases);


    // in the unlikely event that we want to restart the auto calibration
    // process (if there was a change in the encoder geometry, for some reason),
    // the user can call this method to reset the calibration data
    void resetAutoCalibration(void);


    // reset the position reference. After calling this method the current
    // position will be zero. This is useful to reset the position reference
    // using some absolute reference, like a limit switch, for instance
    void resetPosition(void);

  private:
    // configuration data:
    uint calibration_data[4]; // relative phase sizes
    uint clocks_per_us;       // save the clk_sys frequency in clocks per us

    // PIO resources being used by this encoder
    PIO pio;
    uint sm;

    // internal fields to keep track of the previous state:
    uint prev_trans_pos, prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;

    // variables used to keep track of incremental calibration
    uint calib_last_us, calib_last_step, calib_count;
    uint calib_sum[4], calib_data[4];

    // position reset value
    uint internal_position, position_reset;

    // internal helper methods
    void read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward);
    uint get_step_start_transition_pos(uint step);
};

#else // ARCH
#error PicoEncoder library requires a PIO peripheral and only works on the RP2040 architecture
#endif

#endif
