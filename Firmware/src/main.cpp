#include "motor.h"
#include "forceSensor.h"
#include "display.h"
#include "usb.h"
#include "angleSensor.h"

#include <Arduino.h>
#include <STM32FreeRTOS.h>

// RTOS Variables
// SemaphoreHandle_t xForceSensorSemaphore; // Mutex to access force sensor pressed states


void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect (to serial monitor) before starting 
  // }
  initMotorTask(2);
  initForceSensorTask(0);
  initDisplayTask(1);
  initUSBTask(1);
  initAngleSensorTask(0);
  // // Create Mutex
  // if ( xForceSensorSemaphore == NULL )
  // {
  //   xForceSensorSemaphore = xSemaphoreCreateMutex();
  //   if ( ( xForceSensorSemaphore ) != NULL )
  //     xSemaphoreGive( ( xForceSensorSemaphore ) );
  // }
  
  // Start RTOS
  // NVIC_SetPriorityGrouping(4);
  vTaskStartScheduler();
  while(1);
}

void loop() {
  
  // Empty
}
