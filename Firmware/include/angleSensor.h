#ifndef ANGLESENSOR_H
#define ANGLESENSOR_H

#include <Arduino.h>
#include <Tlv493d.h>

void initAngleSensor();
float getAngleDeg();
float getAngleRad();

#endif // ANGLESENSOR_H