#pragma once
#include <stdint.h>
#include <Arduino.h>

// Sensor Pin Definitions
const uint8_t LONG_RANGE_IR_X_LEFT_SENSOR_PIN = A0;
const uint8_t LONG_RANGE_IR_Y_LEFT_SENSOR_PIN = A1;
const uint8_t LONG_RANGE_IR_X_RIGHT_SENSOR_PIN = A2;
const uint8_t LONG_RANGE_IR_Y_RIGHT_SENSOR_PIN = A3;
const uint8_t SHORT_RANGE_IR_X_LEFT_SENSOR_PIN = A4;

const uint8_t DIGITAL_IR_RIGHT3_PIN = 32;	// Right most Digital IR Sensor
const uint8_t DIGITAL_IR_RIGHT2_PIN = 33;
const uint8_t DIGITAL_IR_RIGHT1_PIN = 34;
const uint8_t DIGITAL_IR_MIDDLE_PIN = 35;	// Middle Pin
const uint8_t DIGITAL_IR_LEFT1_PIN = 36;
const uint8_t DIGITAL_IR_LEFT2_PIN = 37;
const uint8_t DIGITAL_IR_LEFT3_PIN = 38;	// Left most Digital IR Sensor

const uint8_t US_Y_ECHO_PIN = 26;
const uint8_t US_Y_TRIGGER_PIN = 27;
const uint8_t US_X_ECHO_PIN = 28;
const uint8_t US_X_TRIGGER_PIN = 29;

// Left Side Sensors
const double SHORT_RANGE_LEFT_MULTIPLIER = 1721.8;
const double SHORT_RANGE_LEFT_POWER = -1.356;

const double LONG_RANGE_X_LEFT_MULTIPLIER = 2774.4;
const double LONG_RANGE_X_LEFT_POWER = -0.541;

const double LONG_RANGE_Y_LEFT_MULTIPLIER = 7763.1;
const double LONG_RANGE_Y_LEFT_POWER = -0.894;

// Right Side Sensors
const double LONG_RANGE_X_RIGHT_MULTIPLIER = 5362.1;
const double LONG_RANGE_X_RIGHT_POWER = -0.757;

const double LONG_RANGE_Y_RIGHT_MULTIPLIER = 5261.4;
const double LONG_RANGE_Y_RIGHT_POWER = -0.755;

// Sensor Validations
const int SHORT_RANGE_INVALID_DISTANCE = 0;
const int SHORT_RANGE_MAX_DISTANCE = 10;

const int LONG_RANGE_INVALID_DISTANCE = 0;
const int LONG_RANGE_MIN_DISTANCE = 10;
const int LONG_RANGE_MAX_DISTANCE = 70;

const int AVERAGE_READING_WINDOW = 10;
const float PERCENT_DIGITAL_VALID_READS = 0.80;

struct DigitalCloseRangeArray
{
	bool isIRleft3Detected;
	bool isIRleft2Detected;
	bool isIRleft1Detected;
	bool isIRmiddleDetected;
	bool isIRright1Detected;
	bool isIRright2Detected;
	bool isIRright3Detected;
};

enum DirectionOfLongIR { IR_X_LEFT, IR_Y_LEFT, IR_X_RIGHT, IR_Y_RIGHT };
enum DirectionOfUS {US_X, US_Y};

void InitializeProximitySensors();
int CalculateLongIRDistance(DirectionOfLongIR direction);
int CalculateShortIRDistance();
DigitalCloseRangeArray CalculateDigitalIRarray();
int CalculateUltrasonicDistance(DirectionOfUS direction);