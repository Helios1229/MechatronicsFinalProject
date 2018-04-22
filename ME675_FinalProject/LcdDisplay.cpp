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

void LcdDisplayLineSensors(LineDetectionStructure lineStruct)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(lineStruct.Sensor0LineDetected);
	lcd.setCursor(4, 0);
	lcd.print(lineStruct.Sensor1LineDetected);
	lcd.setCursor(8, 0);
	lcd.print(lineStruct.Sensor2LineDetected);
	lcd.setCursor(12, 0);
	lcd.print(lineStruct.Sensor3LineDetected);

	lcd.setCursor(0, 1);
	lcd.print(lineStruct.Sensor4LineDetected);
	lcd.setCursor(4, 1);
	lcd.print(lineStruct.Sensor5LineDetected);
	lcd.setCursor(8, 1);
	lcd.print(lineStruct.Sensor6LineDetected);
	lcd.setCursor(12, 1);
	lcd.print(lineStruct.Sensor7LineDetected);

}

void LcdDisplayText(char value[])
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value);
}

void LcdDisplayText(char value1[], float value2)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(float value1, char value2[])
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(char value1[], char value2[])
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(value1);
	lcd.setCursor(0, 1);
	lcd.print(value2);
}

void LcdDisplayText(char value1[], float value2, float value3)
{
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