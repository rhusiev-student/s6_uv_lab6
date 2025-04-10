#include "Freenove_WS2812_Lib_for_ESP32.h"
#include <Bluepad32.h>
#include "leds.hpp"
#include "delays.hpp"

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

void blocking_blink(uint8_t times) {
  for (uint8_t i = 0; i < times; ++i) {
    brake();
    front();
    strip.show();
    delay(200);
    off_front_left();
    off_front_right();
    off_back_left();
    off_back_right();
    strip.show();
    delay(100);
  }
}

void back() {
  strip.setLedColorData(6, 50, 0, 0);
  strip.setLedColorData(7, 50, 0, 0);
  strip.setLedColorData(8, 50, 0, 0);
  strip.setLedColorData(9, 50, 0, 0);
  strip.setLedColorData(10, 50, 0, 0);
  strip.setLedColorData(11, 50, 0, 0);
}

void brake() {
  strip.setLedColorData(6, 255, 0, 0);
  strip.setLedColorData(7, 255, 0, 0);
  strip.setLedColorData(8, 255, 0, 0);
  strip.setLedColorData(9, 255, 0, 0);
  strip.setLedColorData(10, 255, 0, 0);
  strip.setLedColorData(11, 255, 0, 0);
}

void front() {
  strip.setLedColorData(0, 255, 255, 255);
  strip.setLedColorData(1, 255, 255, 255);
  strip.setLedColorData(2, 255, 255, 255);
  strip.setLedColorData(3, 255, 255, 255);
  strip.setLedColorData(4, 255, 255, 255);
  strip.setLedColorData(5, 255, 255, 255);
}

void off_front_left() {
  strip.setLedColorData(0, 0, 0, 0);
  strip.setLedColorData(1, 0, 0, 0);
  strip.setLedColorData(2, 0, 0, 0);
}

void off_front_right() {
  strip.setLedColorData(3, 0, 0, 0);
  strip.setLedColorData(4, 0, 0, 0);
  strip.setLedColorData(5, 0, 0, 0);
}

void off_back_left() {
  strip.setLedColorData(9, 0, 0, 0);
  strip.setLedColorData(10, 0, 0, 0);
  strip.setLedColorData(11, 0, 0, 0);
}

void off_back_right() {
  strip.setLedColorData(6, 0, 0, 0);
  strip.setLedColorData(7, 0, 0, 0);
  strip.setLedColorData(8, 0, 0, 0);
}

void turn_left(bool front, bool back, bool brake, uint32_t i, bool reset) {
  int color1, color2, color3;
  color1 = 0;
  color2 = 0;
  color3 = 0;
  if (front) {
    color1 = 255;
    color2 = 255;
    color3 = 255;
  }
  int color4, color5, color6;
  color4 = 0;
  color5 = 0;
  color6 = 0;
  if (back) {
    color4 = 100;
  }
  if (brake) {
    color4 = 255;
  }

  static uint32_t start_left_i = max_int;
  if (start_left_i == max_int || reset) {
    start_left_i = i;
  }
  auto diff = i - start_left_i;
  if (diff >= 0 && diff <= 7 * turn_delay) {
    strip.setLedColorData(2, 200, 30, 0);
    strip.setLedColorData(9, 200, 30, 0);
  }
  if (diff == 7 * turn_delay) {
    strip.setLedColorData(2, color1, color2, color3);
    strip.setLedColorData(9, color4, color5, color6);
  }
  if (diff >= turn_delay && diff <= 8 * turn_delay) {
    strip.setLedColorData(1, 200, 30, 0);
    strip.setLedColorData(10, 200, 30, 0);
  }
  if (diff == 8 * turn_delay) {
    strip.setLedColorData(1, color1, color2, color3);
    strip.setLedColorData(10, color4, color5, color6);
  }
  if (diff >= 2 * turn_delay && diff <= 9 * turn_delay) {
    strip.setLedColorData(0, 200, 30, 0);
    strip.setLedColorData(11, 200, 30, 0);
  }
  if (diff == 9 * turn_delay) {
    strip.setLedColorData(0, color1, color2, color3);
    strip.setLedColorData(11, color4, color5, color6);
    start_left_i = max_int;
  }
}

void turn_right(bool front, bool back, bool brake, uint32_t i, bool reset) {
  int color1, color2, color3;
  color1 = 0;
  color2 = 0;
  color3 = 0;
  if (front) {
    color1 = 255;
    color2 = 255;
    color3 = 255;
  }
  int color4, color5, color6;
  color4 = 0;
  color5 = 0;
  color6 = 0;
  if (back) {
    color4 = 100;
  }
  if (brake) {
    color4 = 255;
  }

  static uint32_t start_right_i = max_int;
  if (start_right_i == max_int || reset) {
    start_right_i = i;
  }
  auto diff = i - start_right_i;
  if (diff >= 0 && diff <= 7 * turn_delay) {
    strip.setLedColorData(3, 200, 30, 0);
    strip.setLedColorData(8, 200, 30, 0);
  }
  if (diff == 7 * turn_delay) {
    strip.setLedColorData(3, color1, color2, color3);
    strip.setLedColorData(8, color4, color5, color6);
  }
  if (diff >= turn_delay && diff <= 8 * turn_delay) {
    strip.setLedColorData(4, 200, 30, 0);
    strip.setLedColorData(7, 200, 30, 0);
  }
  if (diff == 8 * turn_delay) {
    strip.setLedColorData(4, color1, color2, color3);
    strip.setLedColorData(7, color4, color5, color6);
  }
  if (diff >= 2 * turn_delay && diff <= 9 * turn_delay) {
    strip.setLedColorData(5, 200, 30, 0);
    strip.setLedColorData(6, 200, 30, 0);
  }
  if (diff == 9 * turn_delay) {
    strip.setLedColorData(5, color1, color2, color3);
    strip.setLedColorData(6, color4, color5, color6);
    start_right_i = max_int;
  }
}

void processLeds(ControllerPtr ctl, int32_t i) {
  auto buttons = ctl->buttons();
  static uint8_t led_turn_right = 0;
  static bool right_prev = false;
  static uint8_t led_turn_left = 0;
  static bool left_prev = false;

  static bool led_front = false; // todo
  static bool front_prev = false;
  static bool led_back = false;
  if (buttons & 0x08) {
    if (!front_prev) {
      led_front = !led_front;
      led_back = !led_back;
    }
    front_prev = true;
  } else {
    front_prev = false;
  }
  bool led_brake = ctl->axisRY() > 10;

  if (buttons & 0x0002) {
    if (!right_prev) {
      if (led_turn_right == 0) {
        led_turn_right = 1;
        led_turn_left = 0;
      } else {
        led_turn_right = 0;
      }
    }
    right_prev = true;
  } else {
    right_prev = false;
  }
  if (buttons & 0x0004) {
    if (!left_prev) {
      if (led_turn_left == 0) {
        led_turn_left = 1;
        led_turn_right = 0;
      } else {
        led_turn_left = 0;
      }
    }
    left_prev = true;
  } else {
    left_prev = false;
  }

  if (led_front) {
    front();
  } else {
    if (!led_turn_left) {
      off_front_left();
    }
    if (!led_turn_right) {
      off_front_right();
    }
  }
  if (led_back) {
    back();
  }
  if (led_brake) {
    brake();
  }
  if (!led_back && !led_brake) {
    if (!led_turn_left) {
      off_back_left();
    }
    if (!led_turn_right) {
      off_back_right();
    }
  }

  if (led_turn_right) {
    turn_right(led_front, led_back, led_brake, i, led_turn_right == 1);
    led_turn_right = 2;
  }
  if (led_turn_left) {
    turn_left(led_front, led_back, led_brake, i, led_turn_left == 1);
    led_turn_left = 2;
  }
  strip.show();
}