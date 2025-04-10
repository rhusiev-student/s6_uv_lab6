#ifndef INCLUDE_DELAYS_HPP_
#define INCLUDE_DELAYS_HPP_

#include <Arduino.h>

const uint8_t single_delay = 2;
const uint8_t move_delays = 20 / single_delay;
const uint8_t update_delays = 50 / single_delay;

const uint8_t servo_delays_initial = 4 / single_delay;

const uint8_t sonar_delays = 50 / single_delay;

const int meow_duration = 220 / single_delay;

#endif // INCLUDE_DELAYS_HPP_