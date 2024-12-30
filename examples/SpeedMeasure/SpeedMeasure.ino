#include "Arduino.h"
#include "PicoEncoder.h"

// declare two encoders that will actually be reading the same pins:
// one will measure the phase sizes and compensate for differences, while
// the other will work as if the phase sizes were perfectly uniform
PicoEncoder encoder;
PicoEncoder non_calibrated_encoder;


// this section is an optional DC motor control code to help test an encoder
// attached to a DC motor (and calibrate phase sizes)

const int dir_pin = p10;
const int pwm_pin = p11;

// must be consecutive pins on the rp2040
const int encoder_pinA = p5;
const int encoder_pinB = p6;

// keep track of current time
uint period_start_us;

static void set_pwm(float value)
{
  if (value < 0) {
    digitalWrite(dir_pin, 1);
    analogWrite(pwm_pin, 255 * value + 255);
  } else {
    digitalWrite(dir_pin, 0);
    analogWrite(pwm_pin, 255 * value);
  }
}


void setup()
{
  Serial.begin(115200);

  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  set_pwm(0);

  // use the same pins for both the calibrated and non-calibrated encoders
  encoder.begin(encoder_pinA);
  non_calibrated_encoder.begin(encoder_pinA);

  delay(2000);

  period_start_us = time_us_32();
}

void loop()
{
  static int count;

  count++;
  if ((count & 63) == 0) {
    if (count > 1000)
      set_pwm(0.0);
    else
      set_pwm(random(-100, 100) * 0.01);
  }

  // wait for the next sample period, while calling "autoCalibratePhases"
  // to measure the phase sizes dynamically. If we had two encoders (to
  // read two motors, for instance), we could call both autoCalibratePhases
  // methods here, to calibrate both encoders simultaneously
  while ((int)(time_us_32() - period_start_us) < 10000)
    encoder.autoCalibratePhases();
  period_start_us += 10000;

  encoder.update();
  non_calibrated_encoder.update();

  if (count < 1050) {
    Serial.print("speed: ");
    Serial.print(encoder.speed);
    Serial.print(", position: ");
    Serial.print(encoder.position);
    Serial.print(", step: ");
    Serial.print(encoder.step);
    if (encoder.autoCalibrationDone()) {
      Serial.print(", phases: 0x");
      Serial.print(encoder.getPhases(), HEX);
    }
    Serial.print(", non calibrated encoder speed: ");
    Serial.println(non_calibrated_encoder.speed);
  }
}
