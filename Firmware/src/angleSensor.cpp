#include "angleSensor.h"

#include <Arduino.h>

Tlv493d sensor = Tlv493d();

void initAngleSensor() {
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    sensor.begin(Wire);
}

float getAngleDeg() {
    sensor.updateData();
    return 360 - (atan2(sensor.getY(), sensor.getX()) + PI) * (180/PI);
}

float getAngleRad() {
    sensor.updateData();
    return (2 * PI) - (atan2(sensor.getY(), sensor.getX()) + PI);
}