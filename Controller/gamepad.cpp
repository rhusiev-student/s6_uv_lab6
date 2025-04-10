#include "gamepad.hpp"
#include "motor.hpp"
#include "leds.hpp"

ControllerPtr myController;
uint8_t myControllerAddr[6] = {0xA0, 0x5A, 0x5E, 0x3B, 0x56, 0x00};
//{0x83, 0x45, 0x66, 0x66, 0x3d, 0xc8};

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
  Serial.printf("CALLBACK: Controller is connected\n");
  ControllerProperties properties = ctl->getProperties();
  for (size_t i = 0; i < 6; ++i) {
    if (myControllerAddr[i] != properties.btaddr[i]) {
      Serial.printf("CALLBACK: Controller connected, but with wrong address: %2X:%2X:%2X:%2X:%2X:%2X. Disconnecting",
        properties.btaddr[0], properties.btaddr[1], properties.btaddr[2], properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
      ctl->disconnect();
      return;
    }
  }
  Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x", ctl->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
//   ledcWrite(BUZZER_CHN, 580);
//   delay(30);
//   ledcWrite(BUZZER_CHN, 600);
//   delay(50);
//   ledcWrite(BUZZER_CHN, 580);
//   delay(30);
//   ledcWrite(BUZZER_CHN, 560);
//   delay(20);
//   ledcWrite(BUZZER_CHN, 540);
//   delay(15);
//   ledcWrite(BUZZER_CHN, 0);
  blocking_blink(2);
  ctl->setPlayerLEDs(0b11111111);
  ctl->playDualRumble(0, 1500, 255, 255);
  myController = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    Serial.println("Disconnect happened");
    bool foundController = false;

    if (myController == ctl) {
      Serial.printf("CALLBACK: Controller disconnected\n");
      myController = nullptr;
      foundController = true;
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myController");
    }
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);
}