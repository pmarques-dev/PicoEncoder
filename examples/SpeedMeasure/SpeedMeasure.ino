#include "Arduino.h"
#include "PicoEncoder.h"

PicoEncoder encoder;

// this section is an optional DC motor control code to help test an encoder
// attached to a DC motor (and calibrate phase sizes)

const int dir_pin = p10;
const int pwm_pin = p11;

// must be consecutive pins on the rp2040
const int encoder_pinA = p5;
const int encoder_pinB = p6;

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

static void measure(void)
{
  int phases;

  // turn on motor
  set_pwm(0.25);
  // wait for the speed to become steady
  delay(500);
  // measure the relative phase sizes
  phases = encoder.measurePhases(4096);
  // stop the motor
  set_pwm(0.0);

  // check the result, if it's negative, report the error
  if (phases < 0) {
    Serial.print("error measuring phases: ");
    Serial.println(phases);
  } else {
    // otherwise report the phase calibration number
    Serial.print("measurement successful, phases: 0x");
    Serial.println(phases, HEX);
  }  
}

void setup()
{
  Serial.begin(115200);

  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  set_pwm(0);

  encoder.begin(encoder_pinA);

  delay(2000);

  // comment after running one measurement
  measure();

  // replace with the value measured
  encoder.setPhases(0x334B35);
}

void loop()
{
  static int count;

  count++;
  if ((count & 63) == 0) {
    if (count > 500)
      set_pwm(0.0);
    else
      set_pwm(random(-100, 100) * 0.01);
  }

  delay(10);
  encoder.update();

  Serial.print("speed: ");
  Serial.print(encoder.speed);
  Serial.print(", position: ");
  Serial.print(encoder.position);
  Serial.print(", step: ");
  Serial.println(encoder.step);
}
