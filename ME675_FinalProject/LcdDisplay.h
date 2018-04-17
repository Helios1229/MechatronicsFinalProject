#pragma once
#include <stdint.h>

const uint8_t LCD_SLAVE_SELECT_PIN = 9;

const unsigned int LCD_UPDATE_INTERVAL = 1000;	// Interval to update the LCD in [ms]
const unsigned int LCD_CHARACTER_WIDTH = 16;	// Number of characters in each row of the LCD
const unsigned int LCD_NUMBER_OF_ROWS = 2;		// Number of rows in the LCD

void LcdDisplayInitialization();
void LcdDisplayLineSensorArray(unsigned char text[]);
void LcdDisplayNumber(float value, int row);
void LcdDisplayNumber(char value[], int row);