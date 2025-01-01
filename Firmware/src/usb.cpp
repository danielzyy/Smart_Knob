#include "usb.h"
// Only define functions if DPIO_FRAMEWORK_ARDUINO_ENABLE_HID is defined in platformio.ini
#if defined(USBCON) && defined(USBD_USE_HID_COMPOSITE)

#include "angleSensor.h"
#include <Arduino.h>
#include "Keyboard.h"
// #include <MediaKeyboard.h>
#include <STM32FreeRTOS.h>


static QueueHandle_t usbQueue;

static void TaskUSB(void *pvParameters)
{
  (void) pvParameters;
  usbQueue = xQueueCreate( 3, sizeof( uint8_t ) );

  // Sample every 20ms
  const TickType_t xDelay = 20 / portTICK_PERIOD_MS;

  Keyboard.begin();
  // MediaKeyboard.begin();
  // for (int i = 0; i<100; i++)
  // {
  //   Keyboard.write(0x00);
  //   delay(1);
  // }
  uint16_t prevAngle = 0;
  
  // uint8_t currentKey = VOLUME_DOWN;
  // int count = 0;
  while (1)
  {
    uint16_t currAngle = getAngleDeg();
    if(currAngle > prevAngle + 5) {
      Keyboard.write('+');
      // MediaKeyboard.press(VOLUME_UP);
      // MediaKeyboard.release();
      prevAngle = currAngle;
    } else if (currAngle < prevAngle - 5) {
      Keyboard.write('-');
      // MediaKeyboard.press(VOLUME_DOWN);
      // MediaKeyboard.release();
      prevAngle = currAngle;
    }
    // if (xQueueReceive(usbQueue, &currentKey, (TickType_t)10)) {
      // if(currentKey == VOLUME_UP)
      // {
        // Keyboard.write('d');
      // }
      // else if (count < 50)
      // {
      //   Keyboard.write(0x00);
      //   sendKey(VOLUME_DOWN);
      // }
      // }
      //  MediaKeyboard.press(currentKey);
      //  vTaskDelay(20);
      //  MediaKeyboard.release();
    // }
      //  delay(20);
      // count++;
      // Keyboard.write(KEY_CAPS_LOCK);
      //  Keyboard.println("PRESSED");
      // Serial.println(uxQueueSpacesAvailable(usbQueue));
      
        // pressed = false;
    // }
    // Serial.println(configCHECK_FOR_STACK_OVERFLOW);
    // }

    vTaskDelay(xDelay);
  }
}

void initUSBTask(UBaseType_t priority)
{
  xTaskCreate(
  TaskUSB
  ,  (const portCHAR *)"USB"
  ,  512  // This stack size can be checked & adjusted by reading the Stack Highwater
  ,  NULL
  ,  priority
  ,  NULL );
}

void sendKey(uint8_t key)
{
  xQueueSend(usbQueue, ( void * ) &key, (TickType_t) 10);
}

#endif // defined(USBCON) && defined(USBD_USE_HID_COMPOSITE)