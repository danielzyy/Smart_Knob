#ifndef USB_H
#define USB_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

void initUSBTask(UBaseType_t priority);
void sendKey(uint8_t key);

#endif // USB_H