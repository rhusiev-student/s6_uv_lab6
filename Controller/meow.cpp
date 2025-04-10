#include "meow.hpp"
#include "motor.hpp"
#include "delays.hpp"

void meow(int32_t i, bool bibi) {
  static size_t internal_counter = 0;
  if (!bibi) {
    ledcWriteTone(BUZZER_CHN, 0);
    internal_counter = 0;
    return;
  }
  if (i % meow_duration == 0) {
    ledcWriteTone(BUZZER_CHN, melody[internal_counter++]);
  }
}