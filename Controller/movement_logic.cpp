#include "movement_logic.hpp"

int32_t angle_vertical = 90;
int32_t angle_horizontal = 90;
uint8_t servo_delays_hor = servo_delays_initial;
uint8_t servo_delays_ver = servo_delays_initial;

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

bool move_by_track(int32_t i) {
  static int8_t direction = 0;
  static bool last_fork_left = false;
  static bool last_fork = false;
  static bool prev_last_fork = false;
  static int32_t lost = 0;
  static int32_t after_fork = 0;
  int32_t speed_fast = 1800;
  int32_t speed_slow = 1100;
  Track_Read();
  if (last_fork && !(sensorValue[0] && !sensorValue[1] && sensorValue[2]) && !(sensorValue[0] && sensorValue[1] && sensorValue[2])) {
    if (after_fork == 0) {
      after_fork = i;
    }
    if (i - after_fork > 50) {
      last_fork = false;
    } else if (i - after_fork < 20) {
      Motor_Move(speed_slow, speed_slow, speed_slow, speed_slow);
    } else if (last_fork_left) {
      Motor_Move(speed_fast, speed_fast, -speed_slow, -speed_fast);
      direction = -1;
    } else {
      Motor_Move(0, -speed_slow, speed_fast, speed_fast);
      direction = 1;
    }
    prev_last_fork = true;
    return true;
  }
  if (prev_last_fork) {
    prev_last_fork = false;
    last_fork_left = !last_fork_left;
    ledcWriteTone(BUZZER_CHN, 800);
  } else {
    ledcWriteTone(BUZZER_CHN, 0);
  }
  if (sensorValue[2] && !sensorValue[0]) {
    Motor_Move(speed_fast, speed_fast, 0, -speed_slow);
    direction = -1;
  } else if (!sensorValue[2] && sensorValue[0]) {
    Motor_Move(-speed_slow, -speed_fast, speed_fast, speed_fast);
    direction = 1;
  } else if (!sensorValue[2] && !sensorValue[0] && !sensorValue[1]) {
    if (lost == 0) {
      lost = i;
    }
    if (direction == -1) {
      Motor_Move(speed_fast, speed_fast, -speed_slow, -speed_fast);
    } else if (direction == 1) {
      Motor_Move(-speed_slow, -speed_fast, speed_fast, speed_fast);
    }
    if (i - lost > 500){
      Motor_Move(0, 0, 0, 0);
    }
    return false;
  } else if (sensorValue[0] && sensorValue[1] && sensorValue[2]) {
    Motor_Move(speed_slow, speed_slow, speed_slow, speed_slow);
    direction = 0;
  } else if (sensorValue[0] && !sensorValue[1] && sensorValue[2]) {
    Motor_Move(0, 0, 0, 0);
    last_fork = true;
    after_fork = 0;
  } else {
    Motor_Move(speed_fast, speed_fast, speed_fast, speed_fast);
    direction = 0;
  }
  if (!(sensorValue[0] && !sensorValue[1] && sensorValue[2])) {
    last_fork = false;
  }
  if (!(!sensorValue[2] && !sensorValue[0] && !sensorValue[1])) {
    lost = 0;
  }
  return true;
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