#include "angleSensor.h"

#include <Arduino.h>
#include <Tlv493d.h>

Tlv493d sensor = Tlv493d();

void initAngleSensor() {
    // do nothing for motor foc setup, since sensor is already setup
}

float getAngleDeg(void)
{
    return 360 - (atan2(sensor.getY(), sensor.getX()) + PI) * (180/PI);
}

float getAngleRad(void)
{
    return (2 * PI) - (atan2(sensor.getY(), sensor.getX()) + PI);
}

static void TaskAngleSensor(void *pvParameters)
{
    (void) pvParameters;

    // Update sensor every 1ms
    const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

    Wire.setClock(1000000L);
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    sensor.begin(Wire);
    sensor.setAccessMode(Tlv493d::AccessMode_e::FASTMODE);

    while (1)
    {
        sensor.updateData();

        vTaskDelay(xDelay);
    }
}

void initAngleSensorTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskAngleSensor
    ,  (const portCHAR *)"Angle Sensor"
    ,  128
    ,  NULL
    ,  priority
    ,  NULL );
}
