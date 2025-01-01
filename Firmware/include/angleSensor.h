#ifndef ANGLESENSOR_H
#define ANGLESENSOR_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

void initAngleSensor(void);
float getAngleDeg(void);
float getAngleRad(void);
void initAngleSensorTask(UBaseType_t priority);

#endif // ANGLESENSOR_H