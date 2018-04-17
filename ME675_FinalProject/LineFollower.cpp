#include "LineFollower.h"
#include "LcdDisplay.h"
#include <Arduino.h>
#include <Wire.h>

unsigned char data[16];
unsigned char t;

void LineFollowerInitialization()
{
	// Initialize line follower
	Wire.begin();
}

LineDetectionStructure ProcessLineFollowerInput()
{
	LineDetectionStructure lineDetectionStruct;

	// Process Line Sensor Inputs
	Wire.requestFrom(9, 16);	// request 16 bytes from slave device #9
	while (Wire.available())	// slave may send less than requested
	{
		data[t] = Wire.read();	// receive a byte as character
		if (t < 15)
		{
			t++;
		}
		else
		{
			t = 0;
		}	
	}

	// Build Line Detection Structure
	for (int index = 0; index <= 7; index++)
	{
		switch (index)
		{
			case 0:
				lineDetectionStruct.Sensor0LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 1:
				lineDetectionStruct.Sensor1LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 2:
				lineDetectionStruct.Sensor2LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 3:
				lineDetectionStruct.Sensor3LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 4:
				lineDetectionStruct.Sensor4LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 5:
				lineDetectionStruct.Sensor5LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 6:
				lineDetectionStruct.Sensor6LineDetected = _didSensorDetectLine(data[2 * index]);
				break;

			case 7:
				lineDetectionStruct.Sensor7LineDetected = _didSensorDetectLine(data[2 * index]);
				break;
		}
	}

	//LcdDisplayLineSensorArray(data);
	return lineDetectionStruct;
}

bool _didSensorDetectLine(unsigned char detectionLevel)
{
	if (detectionLevel > BLACK_LINE_THRESHOLD)
	{
		return false;
	}
	else
	{
		return true;
	}
}