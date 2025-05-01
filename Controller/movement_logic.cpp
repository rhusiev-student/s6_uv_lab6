#include "movement_logic.hpp"

int32_t angle_vertical = 90;
int32_t angle_horizontal = 90;
uint8_t servo_delays_hor = servo_delays_initial;
uint8_t servo_delays_ver = servo_delays_initial;

bool move_by_track(int32_t i) {
  static int8_t direction = 0;
  static bool last_fork_left = false;
  static bool last_fork = false;
  static bool prev_last_fork = false;
  static bool last_solid = true;
  static int32_t lost = 0;
  static int32_t after_fork = 0;

  static float integral = 0.0f;
  static float last_error = 0.0f; // for derivative

  const int32_t initial_speed_base = 1800;
  const float coef = static_cast<float>(initial_speed_base) / 1000;
  const float Kp = 1390.0f * coef;
  const float Ki = 5.0f * coef;
  const float Kd = 1410.0f * coef;
  const float ERR_WHEN_LOST = 1.75f;
  const float ERR_WHEN_100 = 1.15f;
  const float ERR_WHEN_110 = 1.1f;
  const float MAX_INTEGRAL = 1000.0f;
  const int32_t FORCE_TURN_AFTER_FORK = 60;
  constexpr int RING_SIZE = 2;

  static float derivative_ring[RING_SIZE];
  static size_t der_idx = 0;

  int32_t speed_base = initial_speed_base;
  // for fork
  int32_t speed_fast = 2000;
  int32_t speed_slow = 500;

  Track_Read();
  bool track_l = sensorValue[0];
  bool track_c = sensorValue[1];
  bool track_r = sensorValue[2];

  if (last_fork && !(track_l && !track_c && track_r)) {
    if (after_fork == 0) {
      after_fork = i;
    }

    int32_t time_since_fork_exit = i - after_fork;

    if (time_since_fork_exit > FORCE_TURN_AFTER_FORK) {
        last_fork = false;
    } else {
        if (last_fork_left) {
             Motor_Move(speed_fast, speed_fast, -speed_slow, -speed_fast);
             direction = -1;
        } else {
             Motor_Move(-speed_slow, -speed_fast, speed_fast, speed_fast);
             direction = 1;
        }
        prev_last_fork = true;
        return true;
    }
  }

  if (prev_last_fork) {
    prev_last_fork = false;
    last_fork_left = !last_fork_left;
    ledcWriteTone(BUZZER_CHN, 800);
  } else {
    ledcWriteTone(BUZZER_CHN, 0);
  }

  float error = 0.0f;
  bool line_lost = false;

  if (track_l && !track_c && track_r) {

    if (last_solid) {
        Motor_Move(0, 0, 0, 0);
        integral = 0;
        last_error = 0;
        lost = 0;
        return true;
    } else {
        error = 0.0f;
        last_fork = true;
        after_fork = 0;
        speed_base = speed_slow;
    }
  } else {
    last_fork = false;

    if (!track_l && track_c && !track_r) {
      error = 0.0f;
      direction = 0;
    } else if (!track_l && track_c && track_r) {
      error = -ERR_WHEN_100;
      direction = 1;
    } else if (!track_l && !track_c && track_r) {
      error = -ERR_WHEN_110;
      direction = 1;
    } else if (track_l && track_c && !track_r) {
      error = ERR_WHEN_100;
      direction = -1;
    } else if (track_l && !track_c && !track_r) {
      error = ERR_WHEN_110;
      direction = -1;
    } else if (track_l && track_c && track_r) {
      error = 0.0f;
      speed_base = speed_slow;
      last_solid = true;
      // direction = 0;
    } else if (!track_l && !track_c && !track_r) {
      line_lost = true;
      if (lost == 0) {
        lost = i;
      }
      if (std::abs(last_error) > 0.1) {
         error = std::copysign(ERR_WHEN_LOST, last_error);
      } else if (direction == -1) {
         error = ERR_WHEN_LOST;
      } else if (direction == 1) {
         error = -ERR_WHEN_LOST;
      } else {
         error = 0.0f;
      }

      if (i - lost > 8000) {
        Motor_Move(0, 0, 0, 0);
        integral = 0;
        return false;
      }
    } else {
        error = 0.0f;
    }
  }

  if (!line_lost) {
    lost = 0;
  }
  if (!(track_l && track_c && track_r)) {
    last_solid = false;
  }

  integral += error;
  integral = std::max(-MAX_INTEGRAL, std::min(MAX_INTEGRAL, integral));

  float derivative = error - last_error;
  derivative_ring[der_idx] = derivative;
  derivative = 0;
  for (size_t i = 0; i < RING_SIZE; ++i) {
    derivative += derivative_ring[(i + der_idx) % RING_SIZE] / sqrt(static_cast<float>(i + 1));
  }
  der_idx = (der_idx) % RING_SIZE;

  float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  last_error = error;


  float pid_output_l = -pid_output;
  float pid_output_r = pid_output;
  if (pid_output > 0) {
    pid_output_l /= 2;
  } else {
    pid_output_r /= 2;
  }
  int32_t left_speed = speed_base - static_cast<int32_t>(pid_output);
  int32_t right_speed = speed_base + static_cast<int32_t>(pid_output);

  Motor_Move(left_speed, left_speed, right_speed, right_speed);


  return true;
}

int32_t to_motor_speed_cube(float speed) {
  if (abs(speed) < 15) {
    return 0;
  }
  auto motor_speed = (motor_max - motor_min) * speed / controller_max * speed / controller_max * speed / controller_max;
  if (motor_speed > motor_start_movement) {
    motor_speed += motor_min;
  } else if (motor_speed < -motor_start_movement) {
    motor_speed -= motor_min;
  }
  return motor_speed;
}
int32_t to_motor_speed(float speed) {
  if (abs(speed) < 15) {
    return 0;
  }
  auto motor_speed = (motor_max - motor_min) * speed / controller_max;
  if (motor_speed > motor_start_movement) {
    motor_speed += motor_min;
  } else if (motor_speed < -motor_start_movement) {
    motor_speed -= motor_min;
  }
  return motor_speed;
}


void rotate_frame(int32_t left_l, int32_t forward, bool pivot, bool pivot_sideways, bool rotate) {
  if (forward > -15) {
    if (pivot) {
      if (left_l > 0) {
        angle_horizontal = 45;
        Motor_Move(left_l, left_l, 0, 0);
      } else {
        angle_horizontal = 170 - 45;
        Motor_Move(0, 0, -left_l, -left_l);
      }
    } else if (pivot_sideways) {
      if (left_l > 0) {
        angle_horizontal = 20;
      } else {
        angle_horizontal = 170 - 20;
      }
      Motor_Move(left_l, 0, -left_l, 0);
    } else if (rotate) {
      if (left_l > 0) {
        angle_horizontal = 0;
      } else {
        angle_horizontal = 170;
      }
      Motor_Move(left_l, left_l, -left_l, -left_l);
    }
  } else {
    if (left_l > 0) {
      angle_horizontal = 0;
    } else {
      angle_horizontal = 170;
    }
    if (pivot) {
      if (left_l > 0) {
        Motor_Move(-left_l, -left_l, 0, 0);
      } else {
        Motor_Move(0, 0, left_l, left_l);
      }
    } else if (pivot_sideways) {
      Motor_Move(0, left_l, 0, -left_l);
    } else if (rotate) {
      Motor_Move(left_l, left_l, -left_l, -left_l);
    }
  }
}

void move_frame(ControllerPtr ctl) {
  auto forward = to_motor_speed_cube(-ctl->axisRY());
  auto left = to_motor_speed_cube(ctl->axisRX());
  auto left_l = ctl->axisX();
  static bool pivot_sideways = false;
  static bool rotate = false;
  if (abs(left_l) > 15) {
    left_l = to_motor_speed_cube(left_l);
    bool pivot = true;
    if (ctl->r1() && !(rotate && ctl->l1())) {
      pivot = false;
      pivot_sideways = true;
      rotate = false;
    } else if (ctl->l1() && !(pivot_sideways && ctl->r1())) {
      pivot = false;
      rotate = true;
      pivot_sideways = false;
    } else {
      rotate = true;
      pivot_sideways = false;
    }
    rotate_frame(left_l, forward, pivot, pivot_sideways, rotate);
    return;
  }
  forward = forward * 2 / 3;
  if (forward >= 0) {
    if (left >= -15 && left <= 15) {
      angle_horizontal = 90;
    } else {
      angle_horizontal = static_cast<int32_t>(std::atan2(forward, left) / PI * 180);
    }
  } else if (forward < -15) {
    if (left > 0) {
      angle_horizontal = 0;
    } else {
      angle_horizontal = 170;
    }
  }

  if (forward == 0 && left == 0) {
    return;
  }

  int movement[4];
  movement[0] = forward + left;
  movement[1] = forward - left;
  movement[2] = forward - left;
  movement[3] = forward + left;
  Motor_Move(movement[0], movement[1], movement[2], movement[3]);
}

int32_t to_servo_speed(int32_t speed, bool hor, int32_t i) {
  auto diff = max_servo_speed * speed / servo_max;
  if (diff == 0) {
    if (hor) {
      servo_delays_hor = servo_delays_initial;
    } else {
      servo_delays_ver = servo_delays_initial;
    }
    return 0;
  }
  int32_t abs_diff;
  int32_t sign;
  if (diff > 0) {
    abs_diff = diff;
    sign = 1;
  } else {
    abs_diff = -diff;
    sign = -1;
  }
  if (abs_diff < servo_speed_bump) {
    if (hor) {
      servo_delays_hor = servo_delays_initial * (servo_speed_bump - abs_diff);
    } else {
      servo_delays_ver = servo_delays_initial * (servo_speed_bump - abs_diff);
    }
    return sign;
  }
  if (hor) {
    servo_delays_hor = servo_delays_initial;
  } else {
    servo_delays_ver = servo_delays_initial;
  }
  return diff - sign * (servo_speed_bump - 1);
}

void move_servo(ControllerPtr ctl, int32_t i) {
  auto buttons = ctl->buttons();
  auto up = ctl->brake();
  auto down = ctl->throttle();
  auto left = false;
  auto right = false;
  if (i % servo_delays_hor == 0) {
    if (right) {
      angle_horizontal -= to_servo_speed(right, true, i);
    }
    if (left) {
      angle_horizontal += to_servo_speed(left, true, i);
    }
    if (angle_horizontal > 170)
      angle_horizontal = 170;
    if (angle_horizontal < 0)
      angle_horizontal = 0;
  }
  if (i % servo_delays_ver == 0) {
    if (up) {
      angle_vertical += 1;
    }
    if (down) {
      angle_vertical -= 1;
    }
    if (angle_vertical > 150)
      angle_vertical = 150;
    if (angle_vertical < 90)
      angle_vertical = 90;
  }

  Servo_1_Angle(angle_horizontal);
  Servo_2_Angle(angle_vertical);
}