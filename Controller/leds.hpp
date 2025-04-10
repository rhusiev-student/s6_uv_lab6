#ifndef INCLUDE_LEDS_HPP_
#define INCLUDE_LEDS_HPP_

#include <limits>
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include "delays.hpp"

#define LEDS_COUNT  12    //Define the count of WS2812
#define LEDS_PIN  	32    //Define the pin number for ESP32
#define CHANNEL	    0     //Define the channels that control WS2812

extern Freenove_ESP32_WS2812 strip;

const uint32_t max_int = ((uint32_t) 0) - ((uint32_t) 1);

void blocking_blink(uint8_t times);

void back();

void brake();

void front();

void off_front_right();
void off_front_left();

void off_back_right();
void off_back_left();

const uint32_t turn_delay = 100 / single_delay;

void turn_left(bool front, bool back, bool brake, uint32_t i, bool reset);

void turn_right(bool front, bool back, bool brake, uint32_t i, bool reset);

void processLeds(ControllerPtr ctl, int32_t i);

#endif // INCLUDE_LEDS_HPP_