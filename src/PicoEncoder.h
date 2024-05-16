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

class PicoEncoder
{
  public:
    // call update() to update these fields:

    // current encoder speed in "substeps per second"
    int speed;

    // current encoder position in substeps
    int position;

    // current position in raw quadrature steps
    uint step;


    // the constructor just initilizes some internal fields
    PicoEncoder();

    // initialize and start the PIO code to track the encoder. The two phases of the 
    // encoder must be connected to consecutive pins and "firstPin" is the lowest
    // numbered pin using arduino naming. Note that the pins must be consecutive 
    // in the RP2040 and may not correspond to consecutive numbers in the arduino
    // mapping. For instance, on the Arduino Nano RP2040 board, pins D2 and D3 are 
    // GPIO25 and GPIO15, so they can not be used for this purpose. However, pin D4 
    // is GPIO16, which means pins D3 and D4 could be used to connect the encoder by
    // passing p3 as "firstPin" (and actually all pins up to D9 are all consecutive).
    //  The method also sets the pins as inputs and will turn on the
    // pull-ups on the pins if "pullUp" is true. Many encoders have open-collector
    // outputs and require pull-ups. If unsure, leave the default value of true.
    //
    // Returns 0 on success, -1 if there is no PIO available, -2 if there are too many
    // encoder instances
    int begin(int firstPin, bool pullUp = true);

    // read the encoder state and update "speed", "position" and "step"
    void update(void);


    // measure the relative phase sizes of the encoder. On success returns a
    // positive number that represents the sizes (and can be passed later to
    // setPhases to calibrate the phase sizes). On failure returns a negative
    // number:
    //  -1 : speed too slow (less than 20 raw steps per second)
    //  -2 : speed too high (while measuring the current step jumped by more than 
    //        one, so either the encoder is rotating to fast or some interrupt
    //        routine is interrupting the code long enough to lose steps)
    int measurePhases(void);

    // set the phase sizes using the result from a previous call to measurePhases
    void setPhases(int phases);


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

    // internal helper methods
    void read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward);
    uint get_step_start_transition_pos(uint step);
};

#else // ARCH
#error PicoEncoder library requires a PIO peripheral and only works on the RP2040 architecture
#endif

#endif
