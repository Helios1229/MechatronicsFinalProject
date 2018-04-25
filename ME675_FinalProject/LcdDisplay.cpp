#include "LcdDisplay.h"
#include "LineFollower.h"
#include <LiquidCrystal.h>
#include <Arduino.h>

LiquidCrystal lcd(LCD_SLAVE_SELECT_PIN);

void LcdDisplayInitialization()
{
	lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
}

void LcdDisplayLineSensorArray(unsigned char text[])
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(text[14]);
	lcd.setCursor(4, 0);
	lcd.print(text[12]);
	lcd.setCursor(8, 0);
	lcd.print(text[10]);
	lcd.setCursor(12, 0);
	lcd.print(text[8]);
	lcd.setCursor(0, 1);
	lcd.print(text[6]);
	lcd.setCursor(4, 1);
	lcd.print(text[4]);
	lcd.setCursor(8, 1);
	lcd.print(text[2]);
	lcd.setCursor(12, 1);
	lcd.print(text[0]);
}

void LcdDisplayLineSensors(LineDetectionStructure lineStruct)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(lineStruct.Sensor7LineDetected);
	lcd.setCursor(4, 0);
	lcd.print(lineStruct.Sensor6LineDetected);
	lcd.setCursor(8, 0);
	lcd.print(lineStruct.Sensor5LineDetected);
	lcd.setCursor(12, 0);
	lcd.print(lineStruct.Sensor4LineDetected);

	lcd.setCursor(0, 1);
	lcd.print(lineStruct.Sensor3LineDetected);
	lcd.setCursor(4, 1);
	lcd.print(lineStruct.Sensor2LineDetected);
	lcd.setCursor(8, 1);
	lcd.print(lineStruct.Sensor1LineDetected);
	lcd.setCursor(12, 1);
	lcd.print(lineStruct.Sensor0LineDetected);
}

void LcdDisplayStateAndDistance(char state[], char description[], int distLeftX, int distLeftY, int DistRightX, int DistRightY)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(state);
	lcd.setCursor(9, 0);
	lcd.print(description);

	lcd.setCursor(0, 1);
	lcd.print(distLeftX);
	lcd.setCursor(4, 1);
	lcd.print(distLeftY);
	lcd.setCursor(8, 1);
	lcd.print(DistRightX);
	lcd.setCursor(12, 1);
	lcd.print(DistRightY);
}

void LcdDisplayTextAndDistance(char state[], char description[], int distance)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(state);

	lcd.setCursor(0, 1);
	lcd.print(description);

	lcd.setCursor(14, 1);
	lcd.print(distance);
}

void LcdDisplayText(char value[])
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value);
}

void LcdDisplayText(float value)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value);
}

void LcdDisplayText(char value1[], float value2)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(float value1, float value2)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(float value1, char value2[])
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(char value1[], char value2[])
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(char value1[], float value2, float value3)
{
	//lcd.begin(LCD_CHARACTER_WIDTH, LCD_NUMBER_OF_ROWS);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
	lcd.setCursor(7, 1);
	lcd.print(" ");
	lcd.setCursor(8, 1);
	lcd.print(value3);
}