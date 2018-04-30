#pragma once
#include <stdint.h>
#include "LineFollower.h"
#include "ProximitySensors.h"

const uint8_t LCD_SLAVE_SELECT_PIN = 9;

const unsigned int LCD_UPDATE_INTERVAL = 1000;	// Interval to update the LCD in [ms]
const unsigned int LCD_CHARACTER_WIDTH = 16;	// Number of characters in each row of the LCD
const unsigned int LCD_NUMBER_OF_ROWS = 2;		// Number of rows in the LCD

void LcdDisplayInitialization();
void LcdDisplayLineSensorArray(unsigned char text[]);
void LcdDisplayLineSensors(LineDetectionStructure lineStruct);
void LcdDisplayDigitalSensors(DigitalCloseRangeArray digitalArray);
void LcdDisplayStateAndDistance(char state[], char description[], int distLeftX, int distLeftY, int DistRightX, int DistRightY);
void LcdDisplayTextAndDistance(char state[], char description[], int distance);
void LcdDisplayText(char value[]);
void LcdDisplayText(float value);
void LcdDisplayText(char value1[], float value2);
void LcdDisplayText(float value1, char value2[]);
void LcdDisplayText(float value1, float value2);
void LcdDisplayText(char value1[], char value2[]);
void LcdDisplayText(char value1[], float value2, float value3);