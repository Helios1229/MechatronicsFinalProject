#pragma once
#include <stdint.h>
#include <Arduino.h>

const uint8_t IR_Y_SENSOR_PIN = A0;
const uint8_t IR_X_SENSOR_PIN = A1;
const uint8_t IR_CLOSE_RANGE_SENSOR_PIN = A2;
const uint8_t US_ECHO_PIN = 36;
const uint8_t US_TRIGGER_PIN = 37;

const double SHORT_RANGE_MULTIPLIER = 2307.9;
const double SHORT_RANGE_POWER = -1.437;
const double LONG_RANGE_X_MULTIPLIER = 4563.7;
const double LONG_RANGE_Y_MULTIPLIER = 5436.8;
const double LONG_RANGE_X_POWER = -0.964;
const double LONG_RANGE_Y_POWER = -0.975;
const int SHORT_RANGE_INVALID_DISTANCE = 0;
const int SHORT_RANGE_MAX_DISTANCE = 10;
const int LONG_RANGE_INVALID_DISTANCE = 0;
const int LONG_RANGE_MIN_DISTANCE = 15;
const int LONG_RANGE_MAX_DISTANCE = 60;
const int AVERAGE_READING_WINDOW = 10;

enum SharpSensorModel { GP2Y0A51SK0F, GP2Y0A60SZLF };
enum DirectionOfIR { X, Y };

void InitializeProximitySensors();
int CalculateIRDistance(SharpSensorModel sensorType, DirectionOfIR direction = DirectionOfIR::X);
int CalculateUltrasonicDistance();