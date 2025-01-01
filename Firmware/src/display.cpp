#include "display.h"
#include "motor.h"
#include "angleSensor.h"

#include <TFT_eSPI.h>
#include <STM32FreeRTOS.h>

#define TASK_PERIOD_MS 5U

// Create the dial sprite, the dial outer and place scale markers
static void createDialScale(TFT_eSprite* dial, TFT_eSprite* tick, int16_t radius, int16_t start_angle, int16_t end_angle, int16_t increment)
{
  start_angle = start_angle % 360;
  end_angle = end_angle % 360;
  // Clear previous dial
  dial->fillCircle(radius, radius, radius+5, TFT_BLACK);
  // Draw dial outline
  dial->fillCircle(radius, radius, radius, TFT_DARKGREY);  // Draw dial outer

  for (int16_t angle = start_angle; angle <= end_angle; angle += increment) {
    tick->pushRotated(dial, angle); // Sprite is used to make scale markers
  }
}

static void TaskDisplay(void *pvParameters)
{
  // Run every 1ms
  const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;

  TFT_eSPI tft = TFT_eSPI();

  TFT_eSprite dial   = TFT_eSprite(&tft);
  TFT_eSprite tick = TFT_eSprite(&tft);
  TFT_eSprite needle = TFT_eSprite(&tft);

  int16_t radius = 90;
  int16_t tick_offset = 5;
  int16_t tick_height = 6;
  int16_t tick_width = 4;
  int16_t max_radius = 120;

  tft.begin();
  tft.setRotation(1);

  // Clear TFT screen
  tft.fillScreen(TFT_BLACK);

  // Create the dial Sprite
  dial.setColorDepth(8);       // Size is odd (i.e. 91) so there is a centre pixel at 45,45
  dial.createSprite(2*radius+1, 2*radius+1);   // 8bpp requires 91 * 91 = 8281 bytes
  dial.setPivot(radius, radius);       // set pivot in middle of dial Sprite
  dial.fillSprite(TFT_TRANSPARENT);           // Fill with transparent colour

  tick.createSprite(tick_width, tick_height);     // 3 pixels wide, 6 high
  tick.fillSprite(TFT_WHITE);  // Fill with white
  tick.setPivot(1, radius);        //  Set pivot point x to the Sprite centre and y to marker radius

  // Create the needle Sprite
  needle.setColorDepth(8);
  needle.createSprite(11, radius); // create the needle Sprite

  needle.fillSprite(TFT_BLACK);          // Fill with black

  // Define needle pivot point
  uint16_t piv_x = needle.width() / 2;   // x pivot of Sprite (middle)
  uint16_t piv_y = needle.height() - tick_offset - tick_height; // y pivot of Sprite (5 pixels from bottom)
  needle.setPivot(piv_x, piv_y);         // Set pivot point in this Sprite

  // Draw the red needle with a yellow tip
  // Keep needle tip 1 pixel inside dial circle to avoid leaving stray pixels
  needle.fillRect(piv_x - 1, 2, 6, 20, TFT_RED);
  needle.fillRect(piv_x - 1, 2, 6, 5, TFT_YELLOW);

  while(1)
  {
    int16_t angle = getAngleDeg();//getMotorAngleDeg();
    // Update dial
    switch(getMotorState())
    {
      case HOLD:
      {
        createDialScale(&dial, &tick, radius, getMotorTargetAngleDeg(), getMotorTargetAngleDeg(), 1);
        break;
      }
      case SELECT:
      {
        createDialScale(&dial, &tick, radius, -180, 180, 45);
        break;
      }
      case SMALL_SELECT:
      {
        createDialScale(&dial, &tick, radius, -180, 180, 5);
        break;
      }
      case FREE:
      default:
      {
        createDialScale(&dial, &tick, radius, -180, 180, 1);
        break;
      }
    }
    
    // Draw the blank dial in the Sprite, add label and number
    dial.setPivot(radius, radius);
    dial.fillCircle(radius, radius, radius-tick_height, TFT_BLACK);

    dial.setTextDatum(TC_DATUM);              // Draw dial text
    dial.drawString("ANGLE", radius, radius+5, 2);
    dial.drawNumber(angle, radius, radius-20, 4);

    // Push a rotated needle Sprite to the dial Sprite, with black as transparent colour
    needle.pushRotated(&dial, angle, TFT_BLACK); // dial is the destination Sprite

    // Push the resultant dial Sprite to the screen, with transparent colour
    dial.pushSprite(max_radius-radius, max_radius-radius, TFT_TRANSPARENT);
    vTaskDelay(xDelay);
  }
}

void initDisplayTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskDisplay
    ,  (const portCHAR *)"Display"
    ,  1024
    ,  NULL
    ,  priority
    ,  NULL );
}
