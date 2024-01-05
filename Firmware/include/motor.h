#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

typedef enum {
  HOLD,
  SELECT,
  SMALL_SELECT,
  FREE,
  BUTTON_PRESS,
  BUTTON_RELEASE,
  COUNT,
} motor_state_E;

void initMotorTask(UBaseType_t priority);
int16_t getMotorAngleDeg(void);
int16_t getMotorTargetAngleDeg(void);
void sendMotorState(motor_state_E state);
motor_state_E getMotorState(void);

#endif // MOTOR_H