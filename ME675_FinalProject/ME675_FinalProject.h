#pragma once
#include <stdint.h>
#include <elapsedMillis.h>

const uint8_t IR_Y_SENSOR_PIN = A0;
const uint8_t IR_X_SENSOR_PIN = A1;

extern elapsedMillis elapsedTime;

void StartPathFollowing();
void LineFollowing(float objectDistanceX, float objectDistanceY);