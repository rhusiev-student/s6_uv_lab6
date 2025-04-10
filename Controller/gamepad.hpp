#ifndef INCLUDE_GAMEPAD_HPP_
#define INCLUDE_GAMEPAD_HPP_

#include <Bluepad32.h>

const uint8_t MAX_CONNECTIONS = 1;
extern ControllerPtr myController;
extern uint8_t myControllerAddr[6];

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);

#endif // INCLUDE_GAMEPAD_HPP_