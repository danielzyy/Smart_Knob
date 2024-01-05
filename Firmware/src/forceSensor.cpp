#include "forceSensor.h"
#include "motor.h"
#include "usb.h"

// #include <MediaKeyboard.h>
#include <Arduino.h>
#include <STM32FreeRTOS.h>

#define FORCE_PIN      (PC3)
#define BUTTON_HYST    (75U)

static void TaskForceSensor(void *pvParameters)
{
  (void) pvParameters;

  // Sample every 2ms
  const TickType_t xDelay = 2 / portTICK_PERIOD_MS;

  float force_threshold = 300.0f; // 3.3k pulldown resistor
  float forceLPF = 0.15f; // Set to 1 for no filter
  int currentForce = 0;
  bool pressed = false;
  int prevForce = 0;

  while (1)
  {
    currentForce = forceLPF*analogRead(FORCE_PIN) + (1-forceLPF)*prevForce;
    prevForce = currentForce;
    // Serial.println(currentForce);

    if (currentForce > force_threshold+BUTTON_HYST && !pressed)
    {
      Serial.println("pressed");
      pressed = true;
      // sendKey(MEDIA_PLAY_PAUSE);
      sendMotorState(BUTTON_PRESS);
    }
    else if (currentForce < force_threshold-BUTTON_HYST && pressed)
    {
      pressed = false;
      sendMotorState(BUTTON_RELEASE);
    }

    vTaskDelay(xDelay);
  }
}

void initForceSensorTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskForceSensor
    ,  (const portCHAR *)"Force Sensor"
    ,  128
    ,  NULL
    ,  priority
    ,  NULL );
}
