/*
 Name:		ME675_Final_Project.ino
 Created:	3/26/2018 7:15:02 PM
 Author:	Jon
*/

#include "ME675_FinalProject.h"
#include "LcdDisplay.h"
#include "LineFollower.h"
#include "StepperControl.h"
#include <Wire.h>
#include "elapsedMillis.h"

bool isLineLost = false;
elapsedMillis elapsedTime;
LineDetectionStructure lineDetectionStruct;

void setup() 
{
	// Initialize devices and motors
	LcdDisplayInitialization();
	LineFollowerInitialization();
	StepperInitialization();
	
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
}


void loop()
{
	lineDetectionStruct = ProcessLineFollowerInput();

	float voltageIR = analogRead(LEFT_IR_SENSOR_PIN);
	float dist = -0.2054*voltageIR + 111.5;
	//LcdDisplayNumber(dist,0);
	delay(50);


	LineFollowingMoveRight();
	if (dist < 25)
	{
		StopMovement();
	}
	else
	{
		if (lineDetectionStruct.Sensor3LineDetected || lineDetectionStruct.Sensor4LineDetected)
		{
			// Tracking Black Line
			LineFollowingMoveCW();
			LcdDisplayNumber("Moving CW", 0);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor2LineDetected)
		{
			// Off to the Left - Move Right
			LineFollowingMoveRight();
			LcdDisplayNumber("Moving RIGHT", 0);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor0LineDetected || lineDetectionStruct.Sensor1LineDetected)
		{
			// Off to the Far Left - Move Far Right
			LineFollowingMoveFarRight();
			LcdDisplayNumber("Moving FAR RIGHT", 0);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor5LineDetected)
		{
			// Off to the Right - Move Left
			LineFollowingMoveLeft();
			LcdDisplayNumber("Moving LEFT", 0);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor6LineDetected || lineDetectionStruct.Sensor7LineDetected)
		{
			// Off to the Far Right - Move Far Left
			LineFollowingMoveFarLeft();
			LcdDisplayNumber("Moving FAR LEFT", 0);
			isLineLost = false;
		}
		else if (!lineDetectionStruct.Sensor1LineDetected &&
			!lineDetectionStruct.Sensor2LineDetected &&
			!lineDetectionStruct.Sensor3LineDetected &&
			!lineDetectionStruct.Sensor4LineDetected &&
			!lineDetectionStruct.Sensor5LineDetected &&
			!lineDetectionStruct.Sensor6LineDetected &&
			!lineDetectionStruct.Sensor7LineDetected)
		{
			if (isLineLost == false)
			{
				// Start timer
				elapsedTime = 0;
			}
			else if (elapsedTime > 250)
			{
				SlowMovement();
				LcdDisplayNumber("SLOW MOVE", 0);
			}
			else if (elapsedTime > 500)
			{
				// Line lost - stop movement
				StopMovement();
				LcdDisplayNumber("STOP MOVE", 0);
			}

			isLineLost = true;
		}
		else
		{
			SlowMovement();
			LcdDisplayNumber("SLOW MOVE", 0);
			isLineLost = false;
		}
	}
}


