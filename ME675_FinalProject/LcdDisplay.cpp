#include "ME675_FinalProject.h" 
#include "LcdDisplay.h"
#include <LiquidCrystal.h>
#include <Arduino.h>

LiquidCrystal lcd(LCD_SLAVE_SELECT_PIN);

void LcdDisplayInitialization()
{
	lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
}

void LcdDisplayLineSensorArray(unsigned char text[])
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(text[0]);
	lcd.setCursor(4, 0);
	lcd.print(text[2]);
	lcd.setCursor(8, 0);
	lcd.print(text[4]);
	lcd.setCursor(12, 0);
	lcd.print(text[6]);

	lcd.setCursor(0, 1);
	lcd.print(text[8]);
	lcd.setCursor(4, 1);
	lcd.print(text[10]);
	lcd.setCursor(8, 1);
	lcd.print(text[12]);
	lcd.setCursor(12, 1);
	lcd.print(text[14]);
}

void LcdDisplayNumber(float value, int row)
{
	lcd.clear();
	lcd.setCursor(0, row);
	lcd.print(value);
}

void LcdDisplayNumber(char value[], int row)
{
	lcd.clear();
	lcd.setCursor(0, row);
	lcd.print(value);
}