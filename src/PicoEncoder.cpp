/*
  PicoEncoder - High resolution quadrature encoder using the PIO on the RP2040
  Created by Paulo Marques, Pedro Pereira, Paulo Costa, 2022
  Distributed under the BSD 2-clause license. For full terms, please refer to the LICENSE file.
*/

#include "PicoEncoder.h"

#ifdef ARDUINO_ARCH_RP2040

#include <pico_encoder.pio.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>


// global configuration: after this number of samples with no step change, 
// consider the encoder stopped
static const int idle_stop_samples = 3;

// global structures to control which PIO and state machines to use
static int encoder_count;
static PIO pio_used[2];


// low level PIO interface

// initialize the PIO state and the substep_state_t structure that keeps track
// of the encoder state
static inline void pico_encoder_program_init(PIO pio, uint sm, uint pin_A)
{
	uint pin_state, position, ints;

	pio_sm_set_consecutive_pindirs(pio, sm, pin_A, 2, false);

	pio_sm_config c = pico_encoder_program_get_default_config(0);
	sm_config_set_in_pins(&c, pin_A); // for WAIT, IN
	// shift to left, auto-push at 32 bits
	sm_config_set_in_shift(&c, false, true, 32);
	sm_config_set_out_shift(&c, true, false, 32);
	// don't join FIFO's
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

	// always run at sysclk, to have the maximum possible time resolution
	sm_config_set_clkdiv(&c, 1.0);

	pio_sm_init(pio, sm, 0, &c);

	// set up status to be rx_fifo < 1
	pio->sm[sm].execctrl = ((pio->sm[sm].execctrl & 0xFFFFFF80) | 0x12);

	// init the state machine according to the current phase. Since we are
	// setting the state running PIO instructions from C state, the encoder may
	// step during this initialization. This should not be a problem though,
	// because as long as it is just one step, the state machine will update
	// correctly when it starts. We disable interrupts anyway, to be safe
	ints = save_and_disable_interrupts();

	pin_state = (gpio_get_all() >> pin_A) & 3;

	// to setup the state machine, we need to set the lower 2 bits of OSR to be
	// the negated pin state
	pio_sm_exec(pio, sm, pio_encode_set(pio_y, ~pin_state));
	pio_sm_exec(pio, sm, pio_encode_mov(pio_osr, pio_y));

	// also set the Y (current step) so that the lower 2 bits of Y have a 1:1
	// mapping to the current phase (input pin state). That simplifies the code
	// to compensate for differences in encoder phase sizes:
	switch (pin_state) {
		case 0: position = 0; break;
		case 1: position = 3; break;
		case 2: position = 1; break;
		case 3: position = 2; break;
	} 
	pio_sm_exec(pio, sm, pio_encode_set(pio_y, position));

	pio_sm_set_enabled(pio, sm, true);
	
	restore_interrupts(ints);
}

static inline void pico_encoder_get_counts(PIO pio, uint sm, uint *step, int *cycles, uint *us)
{
	int i, pairs;
	uint ints;
	
	pairs = pio_sm_get_rx_fifo_level(pio, sm) >> 1;

	// read all data with interrupts disabled, so that there can not be a
	// big time gap between reading the PIO data and the current us
	ints = save_and_disable_interrupts();
	for (i = 0; i < pairs + 1; i++) {
		*cycles = pio_sm_get_blocking(pio, sm);
		*step = pio_sm_get_blocking(pio, sm);
	}
	*us = time_us_32();
	restore_interrupts(ints);
}



// PicoEncoder class definition

PicoEncoder::PicoEncoder()
{
  // we set the default phase sizes here, so that if the user sets the phase 
  // sizes before calling begin, those will override the default and it will
  // just work
  setPhases(0x404040);

  // just set the position/speed fields to zero
  position = 0;
  speed = 0;
  step = 0;
}

void PicoEncoder::setPhases(int phases)
{
  calibration_data[0] = 0;
  calibration_data[1] = (phases & 0xFF);
  calibration_data[2] = calibration_data[1] + ((phases >> 8) & 0xFF);
  calibration_data[3] = calibration_data[2] + ((phases >> 16) & 0xFF);
}

// internal helper functions

void PicoEncoder::read_pio_data(uint *step, uint *step_us, uint *transition_us, int *forward)
{
  int cycles;

  // get the raw data from the PIO state machine
  pico_encoder_get_counts(pio, sm, step, &cycles, step_us);

  // when the PIO program detects a transition, it sets cycles to either zero
  // (when step is incrementing) or 2^31 (when step is decrementing) and keeps
  // decrementing it on each 13 clock loop. We can use this information to get
  // the time and direction of the last transition
  if (cycles < 0) {
      cycles = -cycles;
      *forward = 1;
  } else {
      cycles = 0x80000000 - cycles;
      *forward = 0;
  }
  *transition_us = *step_us - ((cycles * 13) / clocks_per_us);
}

// get the sub-step position of the start of a step
uint PicoEncoder::get_step_start_transition_pos(uint step)
{
  return ((step << 6) & 0xFFFFFF00) | calibration_data[step & 3];
}

// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds
static int substep_calc_speed(int delta_substep, int delta_us)
{
  return ((int64_t) delta_substep << 20) / delta_us;
}


// function to measure the difference between the different steps on the encoder
int PicoEncoder::measurePhases(void)
{
  int forward, s1, s2, s3;
  uint count, cur_us, last_us, step_us, step, last_step, start_us, delta;
  int64_t sum[4], total;

  memset(sum, 0, sizeof(sum));

  // keep reading the PIO state in a tight loop to get all steps and use the
  // transition measures of the PIO code to measure the time of each step
  last_step = 0;
  last_us = 0;
  count = 0;

  start_us = time_us_32();

  while (1) {

    read_pio_data(&step, &step_us, &cur_us, &forward);

    // if we don't have a transition, just check some stopping conditions and
    // keep waiting for a transition
    if (step == last_step) {
      delta = time_us_32() - start_us;
      // if we have less than 20 steps per second, this is too slow for the
      // calibration to produce good results
      if (count * (1000000 / 20) + 500000 < delta)
        return -1;
      
      // never take more than 10 seconds to run the calibration, even if we only
      // have 200 steps. Just use whatever information was gathered so far
      if (delta > 10000000)
        break;

      // if we already have 1024 steps and have measured for over 2 seconds, we
      // are fine as well. If we have a really fast step rate, then in two
      // seconds we may be able to get a high step count
      if (count > 1024 && delta > 2000000)
        break;

      continue;
    }

    // check that we are not skipping steps
    if (count > 10 && abs((int)(last_step - step)) > 1)
      return -2;

    // sum the step period in the correct step sum
    if (last_us != 0) {
      if (forward)
        sum[(step - 1) & 3] += cur_us - last_us;
      else
        sum[(step + 1) & 3] += cur_us - last_us;
    }

    last_step = step;
    last_us = cur_us;
    count++;
  }

  // scale the sizes to a total of 256 to be used as sub-steps
  total = sum[0] + sum[1] + sum[2] + sum[3];
  s1 = (sum[0] * 256 + total / 2) / total;
  s2 = ((sum[0] + sum[1]) * 256 + total / 2) / total;
  s3 = ((sum[0] + sum[1] + sum[2]) * 256 + total / 2) / total;

  s3 -= s2;
  s2 -= s1;

  return s1 | (s2 << 8) | (s3 << 16);
}


// some Arduino mbed Pico boards have non trivial pin mappings and require a
// function to translate
#if defined(ARDUINO_ARCH_MBED)
#include <pinDefinitions.h>
static int translate_pin(int pin) { return digitalPinToPinName(pin); }
#else
static int translate_pin(int pin) { return pin; }
#endif

int PicoEncoder::begin(int firstPin, bool pullUp)
{
  int i, forward, gpio_pin;

  // the first encoder needs to load a PIO with the PIO code
  if (encoder_count == 0) {
    // it can either use pio0
    if (pio_can_add_program(pio0, &pico_encoder_program))
      pio_used[0] = pio0;
    else if (pio_can_add_program(pio1, &pico_encoder_program))
      pio_used[0] = pio1; // or pio1
    else
      return -1; // or give up
    // load the code into the PIO
    pio_add_program(pio_used[0], &pico_encoder_program);
    // claim all SM's on this PIO, as it's a safer option to avoid having other
    // libraries thinking they can claim SM's without checking that they can
    // actually load their code into the PIO
    for (i = 0; i < 4; i++)
      pio_sm_claim(pio, i);

  } else if (encoder_count == 4) {
    // the 5th encoder needs to try to use the other PIO
    pio_used[1] = (pio_used[0] == pio0) ? pio1 : pio0;
    if (!pio_can_add_program(pio_used[1], &pico_encoder_program))
      return -1;
    pio_add_program(pio_used[1], &pico_encoder_program);
    for (i = 0; i < 4; i++)
      pio_sm_claim(pio, i);

  } else if (encoder_count >= 8) {
    // we don't support more than 8 encoders
    return -2;
  }

  // assign the PIO and state machine and update the encoder count
  pio = pio_used[encoder_count / 4];
  sm = encoder_count % 4;
  encoder_count++;

  // set all fields to zero by default
  prev_trans_pos = 0;
  prev_low = 0;
  prev_high = 0;
  idle_stop_sample_count = 0;
  speed_2_20 = 0;
  speed = 0;

  // save the pin used
  gpio_pin = translate_pin(firstPin);

  // the PIO init code sets the pins as inputs. Optionally turn on pull ups
  // here if the user asked for it
  if (pullUp) {
    gpio_set_pulls(gpio_pin, true, false);
    gpio_set_pulls(gpio_pin + 1, true, false);
  }

  // initialize the PIO program (and save the PIO reference)
  pico_encoder_program_init(pio, sm, gpio_pin);

  // start "stopped" so that we don't use stale data to compute speeds
  stopped = 1;

  // cache the PIO cycles per us
  clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

  // initialize the "previous state"
  read_pio_data(&step, &prev_step_us, &prev_trans_us, &forward);

  position = get_step_start_transition_pos(step) + 32;

  return 0;
}


// read the PIO data and update the speed / position estimate
void PicoEncoder::update(void)
{
  uint new_step, step_us, transition_us, transition_pos, low, high;
  int forward, speed_high, speed_low;

  // read the current encoder state from the PIO
  read_pio_data(&new_step, &step_us, &transition_us, &forward);

  // from the current step we can get the low and high boundaries in
  // substeps of the current position
  low = get_step_start_transition_pos(new_step);
  high = get_step_start_transition_pos(new_step + 1);

  // if we were not stopped, but the last transition was more than
  // "idle_stop_samples" ago, we are stopped now
  if (new_step == step)
    idle_stop_sample_count++;
  else
    idle_stop_sample_count = 0;

  if (!stopped && idle_stop_sample_count >= idle_stop_samples) {
    speed = 0;
    speed_2_20 = 0;
    stopped = 1;
  }

  // when we are at a different step now, there is certainly a transition
  if (step != new_step) {
    // the transition position depends on the direction of the move
    transition_pos = forward ? low : high;

    // if we are not stopped, that means there is valid previous transition
    // we can use to estimate the current speed
    if (!stopped)
      speed_2_20 = substep_calc_speed(transition_pos - prev_trans_pos, transition_us - prev_trans_us);

    // if we have a transition, we are not stopped now
    stopped = 0;
    // save the timestamp and position of this transition to use later to
    // estimate speed
    prev_trans_pos = transition_pos;
    prev_trans_us = transition_us;
  }

  // if we are stopped, speed is zero and the position estimate remains
  // constant. If we are not stopped, we have to update the position and speed
  if (!stopped) {
    // although the current step doesn't give us a precise position, it does
    // give boundaries to the position, which together with the last
    // transition gives us boundaries for the speed value. This can be very
    // useful especially in two situations:
    // - we have been stopped for a while and start moving quickly: although
    //   we only have one transition initially, the number of steps we moved
    //   can already give a non-zero speed estimate
    // - we were moving but then stop: without any extra logic we would just
    //   keep the last speed for a while, but we know from the step
    //   boundaries that the speed must be decreasing

    // if there is a transition between the last sample and now, and that
    // transition is closer to now than the previous sample time, we should
    // use the slopes from the last sample to the transition as these will
    // have less numerical issues
    if (prev_trans_us > prev_step_us && 
        (int)(prev_trans_us - prev_step_us) > (int)(step_us - prev_trans_us)) {
      speed_high = substep_calc_speed(prev_trans_pos - prev_low, prev_trans_us - prev_step_us);
      speed_low = substep_calc_speed(prev_trans_pos - prev_high, prev_trans_us - prev_step_us);
    } else {
      // otherwise use the slopes from the last transition to now
      speed_high = substep_calc_speed(high - prev_trans_pos, step_us - prev_trans_us);
      speed_low = substep_calc_speed(low - prev_trans_pos, step_us - prev_trans_us);
    }
    // make sure the current speed estimate is between the maximum and
    // minimum values obtained from the step slopes
    if (speed_2_20 > speed_high)
      speed_2_20 = speed_high;
    if (speed_2_20 < speed_low)
      speed_2_20 = speed_low;

    // convert the speed units from "sub-steps per 2^20 us" to "sub-steps
    // per second"
    speed = (speed_2_20 * 62500LL) >> 16;

    // estimate the current position by applying the speed estimate to the
    // most recent transition
    position = prev_trans_pos + (((int64_t)speed_2_20 * (step_us - transition_us)) >> 20);

    // make sure the position estimate is between "low" and "high", as we
    // can be sure the actual current position must be in this range
    if ((int)(position - high) > 0)
      position = high;
    else if ((int)(position - low) < 0)
      position = low;
  }

  // save the current values to use on the next sample
  prev_low = low;
  prev_high = high;
  step = new_step;
  prev_step_us = step_us;
}

#else // ARCH
#error PicoEncoder library requires a PIO peripheral and only works on the RP2040 architecture
#endif
