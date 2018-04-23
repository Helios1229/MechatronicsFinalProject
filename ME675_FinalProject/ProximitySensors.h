#pragma once
#include <stdint.h>
#include <Arduino.h>

const uint8_t IR_Y_SENSOR_PIN = A0;
const uint8_t IR_X_SENSOR_PIN = A1;
const uint8_t IR_CLOSE_RANGE_SENSOR_PIN = A2;
const uint8_t US_ECHO_PIN = 36;
const uint8_t US_TRIGGER_PIN = 37;

const double SHORT_RANGE_MULTIPLIER = 4.2354;
const double SHORT_RANGE_POWER = -1.044;
const double LONG_RANGE_MULTIPLIER = 54.879;
const double LONG_RANGE_POWER = -1.378;
const int SHORT_RANGE_INVALID_DISTANCE = 0;
const int SHORT_RANGE_MAX_DISTANCE = 15;
const int LONG_RANGE_INVALID_DISTANCE = 0;
const int LONG_RANGE_MAX_DISTANCE = 150;
const int AVERAGE_READING_WINDOW = 10;

enum SharpSensorModel { GP2Y0A51SK0F, GP2Y0A60SZLF };
enum DirectionOfIR { X, Y };

void InitializeProximitySensors();
int CalculateIRDistance(SharpSensorModel sensorType, DirectionOfIR direction = DirectionOfIR::X);
int CalculateUltrasonicDistance();