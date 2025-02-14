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

  pio_gpio_init(pio, pin_A);
  pio_gpio_init(pio, pin_A + 1);

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
  setPhases(DEFAULT_PHASES);

  // just set the position/speed fields to zero
  position = 0;
  speed = 0;
  step = 0;

  // reset incremental calibration data
  resetAutoCalibration();
}

void PicoEncoder::setPhases(int phases)
{
  calibration_data[0] = 0;
  calibration_data[1] = (phases & 0xFF);
  calibration_data[2] = calibration_data[1] + ((phases >> 8) & 0xFF);
  calibration_data[3] = calibration_data[2] + ((phases >> 16) & 0xFF);
}

int PicoEncoder::getPhases(void)
{
  return calibration_data[1] |
    ((calibration_data[2] - calibration_data[1]) << 8) |
    ((calibration_data[3] - calibration_data[2]) << 16);
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


// incrementally update the phase measure, so that the substep estimation takes
// phase sizes into account. The function is not allowed to block for more than
// "period_us" microseconds
void PicoEncoder::autoCalibratePhases(void)
{
  uint cur_us, step_us, step, delta;
  int forward, steps, need_rescale, i, total;

  // read raw encoder data. Reading encoder data is an idempotent operation,
  // so we can still continue calling update and everything should just work
  read_pio_data(&step, &step_us, &cur_us, &forward);

  // if we are still on the same step as before, there is nothing to see
  if (step == calib_last_step)
    return;

  // if calib_last_us is zero, that means we haven't started yet, so don't try
  // to use a delta to nothing
  if (calib_last_us == 0)
    delta = 0;
  else
    delta = cur_us - calib_last_us;
  steps = calib_last_step - step;

  calib_last_step = step;
  calib_last_us = cur_us;

  // if we've skipped a step, we can not use this information (and we need to
  // reset the data). Also, do the same if we didn't skip a step but the last
  // step was just too slow to be usable (> 20ms)
  if (abs(steps) > 1 || delta > 20000 || delta == 0) {
    memset(calib_data, 0, sizeof(calib_data));
    return;
  }

  // save the step period in the correct data slot
  if (forward)
    calib_data[(step - 1) & 3] = delta;
  else
    calib_data[(step + 1) & 3] = delta;

  // if we don't have a measure of all the steps yet, just continue
  if (calib_data[0] == 0 || calib_data[1] == 0 || calib_data[2] == 0 || calib_data[3] == 0)
    return;

  // otherwise, use the measurement. Sum the just acquired 4 step sizes to the
  // step size total accumulator. Check if the values in the accumulator are
  // getting too big and halve them in that case, to keep them manageable
  need_rescale = 0;
  for (i = 0; i < 4; i++) {
    calib_sum[i] += calib_data[i];
    calib_data[i] = 0;
    if (calib_sum[i] > 2500000)
      need_rescale = 1;
  }

  total = 0;
  for (i = 0; i < 4; i++) {
    if (need_rescale)
      calib_sum[i] >>= 1;
    total += calib_sum[i];
  }
  calib_count++;

  // if we don't have at least 32 full measurements, don't use them yet, as
  // we may still have a big bias (this is just an heuristic)
  if (calib_count < 32)
    return;

  // scale the sizes to a total of 256 to be used as sub-steps
  calibration_data[0] = 0;
  calibration_data[1] = (calib_sum[0] * 256 + total / 2) / total;
  calibration_data[2] = ((calib_sum[0] + calib_sum[1]) * 256 + total / 2) / total;
  calibration_data[3] = ((calib_sum[0] + calib_sum[1] + calib_sum[2]) * 256 + total / 2) / total;
}

void PicoEncoder::resetAutoCalibration(void)
{
  memset(calib_sum, 0, sizeof(calib_sum));
  calib_count = 0;
  calib_last_us = 0;
}


// some Arduino mbed Pico boards have non trivial pin mappings and require a
// function to translate
#if defined(ARDUINO_ARCH_MBED)
#include <pinDefinitions.h>
static int translate_pin(int pin) { return digitalPinToPinName(pin); }
#else
static int translate_pin(int pin) { return pin; }
#endif

// try to claim all SM's and load the program to PIO 'pio'. Return true on
// success, false on failure
static bool pico_encoder_claim_pio(PIO pio)
{
  // check that we can load the program on this PIO
  if (!pio_can_add_program(pio, &pico_encoder_program))
    return false;

  // check that all SM's are free. Some libraries claim an SM and later load the
  // PIO code and that would not interact well with code that uses the entire
  // PIO code space. So just make sure we can claim all the SM's to prevent this
  for (int i = 0; i < 4; i++)
    if (pio_sm_is_claimed(pio, i))
      return false;

  // claim all SM's
  for (int i = 0; i < 4; i++)
    pio_sm_claim(pio, i);

  // load the code into the PIO
  pio_add_program(pio, &pico_encoder_program);

  return true;
}

int PicoEncoder::begin(int firstPin, bool pullUp)
{
  int forward, gpio_pin;

  // the first encoder needs to load a PIO with the PIO code
  if (encoder_count == 0) {
    // it can either use pio0
    if (pico_encoder_claim_pio(pio0))
      pio_used[0] = pio0;
    else if (pico_encoder_claim_pio(pio1))
      pio_used[0] = pio1; // or pio1
    else
      return -1; // or give up

  } else if (encoder_count == 4) {
    // the 5th encoder needs to try to use the other PIO
    pio_used[1] = (pio_used[0] == pio0) ? pio1 : pio0;
    if (!pico_encoder_claim_pio(pio_used[1]))
      return -1;

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


// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds
static int substep_calc_speed(int delta_substep, int delta_us)
{
  return ((int64_t) delta_substep << 20) / delta_us;
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
    internal_position = prev_trans_pos + (((int64_t)speed_2_20 * (step_us - transition_us)) >> 20);

    // make sure the position estimate is between "low" and "high", as we
    // can be sure the actual current position must be in this range
    if ((int)(internal_position - high) > 0)
      internal_position = high;
    else if ((int)(internal_position - low) < 0)
      internal_position = low;
  }

  // compute the user position, as the difference to the reset position
  position = internal_position - position_reset;

  // save the current values to use on the next sample
  prev_low = low;
  prev_high = high;
  step = new_step;
  prev_step_us = step_us;
}

void PicoEncoder::resetPosition(void)
{
  position_reset = internal_position;
  position = 0;
}


#else // ARCH
#error PicoEncoder library requires a PIO peripheral and only works on the RP2040 architecture
#endif
