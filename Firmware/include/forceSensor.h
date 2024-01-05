#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

void initForceSensorTask(UBaseType_t priority);

#endif // FORCESENSOR_H