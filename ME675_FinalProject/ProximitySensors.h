#pragma once
#include <stdint.h>
#include <Arduino.h>

// Sensor Pin Definitions
const uint8_t LONG_RANGE_IR_X_LEFT_SENSOR_PIN = A0;
const uint8_t LONG_RANGE_IR_Y_LEFT_SENSOR_PIN = A1;
const uint8_t LONG_RANGE_IR_X_RIGHT_SENSOR_PIN = A2;
const uint8_t LONG_RANGE_IR_Y_RIGHT_SENSOR_PIN = A3;
const uint8_t SHORT_RANGE_IR_X_LEFT_SENSOR_PIN = A4;

const uint8_t US_ECHO_PIN = 36;
const uint8_t US_TRIGGER_PIN = 37;

// Left Side Sensors
const double SHORT_RANGE_LEFT_MULTIPLIER = 1952.5;
const double SHORT_RANGE_LEFT_POWER = -1.105;

const double LONG_RANGE_X_LEFT_MULTIPLIER = 4892.9;
const double LONG_RANGE_X_LEFT_POWER = -0.804;

const double LONG_RANGE_Y_LEFT_MULTIPLIER = 4551.1;
const double LONG_RANGE_Y_LEFT_POWER = -0.943;

// Right Side Sensors
const double LONG_RANGE_X_RIGHT_MULTIPLIER = 4563.7;
const double LONG_RANGE_X_RIGHT_POWER = -0.964;

const double LONG_RANGE_Y_RIGHT_MULTIPLIER = 5436.8;
const double LONG_RANGE_Y_RIGHT_POWER = -0.975;

// Sensor Validations
const int SHORT_RANGE_INVALID_DISTANCE = 0;
const int SHORT_RANGE_MAX_DISTANCE = 10;

const int LONG_RANGE_INVALID_DISTANCE = 0;
const int LONG_RANGE_MIN_DISTANCE = 15;
const int LONG_RANGE_MAX_DISTANCE = 60;

const int AVERAGE_READING_WINDOW = 10;

enum DirectionOfLongIR { IR_X_LEFT, IR_Y_LEFT, IR_X_RIGHT, IR_Y_RIGHT };

void InitializeProximitySensors();
int CalculateLongIRDistance(DirectionOfLongIR direction);
int CalculateShortIRDistance();
int CalculateUltrasonicDistance();