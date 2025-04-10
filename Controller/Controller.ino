#include <Arduino.h>
#include "gamepad.hpp"
#include "motor.hpp"
#include "movement_logic.hpp"
#include "delays.hpp"
#include "leds.hpp"
#include "meow.hpp"

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

void reset() {
  Motor_Move(0, 0, 0, 0);
  Servo_1_Angle(90);
  Servo_2_Angle(90);
  off_front_left();
  off_front_right();
  off_back_left();
  off_back_right();
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

  strip.begin();
  strip.setBrightness(255);

  PCA9685_Setup();
  reset();
  Buzzer_Setup();
  Ultrasonic_Setup();
  blocking_blink(1);
  // Buzzer_Alert(2, 1);
}

uint32_t i = 0;
bool dataUpdated = false;

void loop() {
  i++; // Will wrap to 0 after 255
  delay(single_delay);
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  if (i % update_delays == 0) {
    dataUpdated = BP32.update();
  }
  if (!dataUpdated) {
    return;
  }
  if (!(myController && myController->isConnected() && myController->hasData())) {
    return;
  }
  if (!myController->isGamepad()) {
    Serial.println("Unsupported controller");
    return;
  }

  if (i % update_delays == 0) {
    processGamepad(myController);
    dumpGamepad(myController);
  }
  if (i % move_delays == 0) {
    move_frame(myController);
  }
  move_servo(myController, i);

  processLeds(myController, i);

  bool BIBI = myController->buttons() & 0x0001 > 0;
  meow(i, BIBI);

  if (i % sonar_delays == 0) {
    float distance = Get_Sonar(); // in cm
    if (distance < 30) {
      myController->playDualRumble(0, sonar_delays * single_delay, 180, 180);
    }
    if (distance < 50) {
      myController->setColorLED(255, 0, 0);
    } else {
      myController->setColorLED(0, 255, 0);
    }
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  // vTaskDelay(1);
}
