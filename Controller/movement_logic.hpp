#ifndef INCLUDE_MOVEMENT_LOGIC_HPP_
#define INCLUDE_MOVEMENT_LOGIC_HPP_

#include <Arduino.h>
#include "delays.hpp"
#include "gamepad.hpp"
#include "motor.hpp"

const float controller_max = 512;
const float  controller_max_cube = controller_max * controller_max * controller_max;

const float motor_max = 2000;
const float motor_min = 600;
const float motor_start_movement = 200;

const int32_t servo_max = 1024;

int32_t to_motor_speed_cube(float speed);
int32_t to_motor_speed(float speed);

void move_frame(ControllerPtr ctl);

const int32_t max_servo_speed = 20;
const int32_t servo_speed_bump = 19;

int32_t to_servo_speed(int32_t speed, bool hor, int32_t i);

void move_servo(ControllerPtr ctl, int32_t i);

#endif // INCLUDE_MOVEMENT_LOGIC_HPP_