/*
 Name:		ME675_Final_Project.ino
 Created:	3/26/2018 7:15:02 PM
 Author:	Jon
*/

#include "ME675_FinalProject.h"
#include "LcdDisplay.h"
#include "LineFollower.h"
#include "StepperControl.h"
#include "IR_Sensor.h"
#include "elapsedMillis.h"
#include <Wire.h>

bool isOnStartingLine = true;
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

	// Pulley DC Motor Control - TODO
	//pinMode(32, OUTPUT);
	//digitalWrite(32, HIGH);
	//pinMode(33, OUTPUT);
	//digitalWrite(33, LOW);
}


void loop()
{
	// Process Line-Reader Input
	lineDetectionStruct = ProcessLineFollowerInput();

	// Process X and Y IR Sensors
	float analogIRY = analogRead(IR_Y_SENSOR_PIN);
	float analogIRX = analogRead(IR_X_SENSOR_PIN);
	float distance_Y = CalculateDistanceFromAnalogInput(analogIRY);
	float distance_X = CalculateDistanceFromAnalogInput(analogIRX);
	delay(50);

	if (isOnStartingLine)
	{
		// Perform Line Following on Starting Path
		StartPathFollowing();
	}
	else
	{
		// Perform Line Following around Main Circle
		LineFollowing(distance_X, distance_Y);
	}
}

void StartPathFollowing()
{
	if (lineDetectionStruct.Sensor2LineDetected
		&& lineDetectionStruct.Sensor3LineDetected
		&& lineDetectionStruct.Sensor4LineDetected
		&& lineDetectionStruct.Sensor5LineDetected
		&& lineDetectionStruct.Sensor6LineDetected)
	{
		// Circle Reached - perform 90 degree CCW Rotation
		LcdDisplayText("START PATH", "Rotate 90 degrees");
		Rotate90CCW();
		isOnStartingLine = false;
	}
	else if (lineDetectionStruct.Sensor1LineDetected
		|| lineDetectionStruct.Sensor2LineDetected)
	{
		LcdDisplayText("START PATH", "MOVE RIGHT");
		LineFollowingMoveRight();
	}
	else if (lineDetectionStruct.Sensor6LineDetected
		|| lineDetectionStruct.Sensor7LineDetected)
	{
		LcdDisplayText("START PATH", "MOVE LEFT");
		LineFollowingMoveLeft();
	}
	else
	{
		LcdDisplayText("START PATH", "MOVE STRAIGHT");
		LineFollowingMoveStraight();
	}
}

void LineFollowing(float objectDistanceX, float objectDistanceY)
{
	if (objectDistanceX < 25.0 || objectDistanceY < 25.0)
	{
		StopMovement();
		LcdDisplayText("OBJECT DETECTED", objectDistanceX, objectDistanceY);
	}
	else
	{
		if (lineDetectionStruct.Sensor2LineDetected
			|| lineDetectionStruct.Sensor3LineDetected
			|| lineDetectionStruct.Sensor4LineDetected
			|| lineDetectionStruct.Sensor5LineDetected)
		{
			// Tracking Black Line
			LineFollowingMoveCW();
			LcdDisplayText("Moving CW", objectDistanceX, objectDistanceY);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor1LineDetected)
		{
			// Off to the Left - Move Right
			LineFollowingMoveRight();
			LcdDisplayText("Moving RIGHT", objectDistanceX, objectDistanceY);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor0LineDetected)
		{
			// Off to the Far Left - Move Far Right
			LineFollowingMoveFarRight();
			LcdDisplayText("Moving FAR RIGHT", objectDistanceX, objectDistanceY);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor6LineDetected)
		{
			// Off to the Right - Move Left
			LineFollowingMoveLeft();
			LcdDisplayText("Moving LEFT", objectDistanceX, objectDistanceY);
			isLineLost = false;
		}
		else if (lineDetectionStruct.Sensor7LineDetected)
		{
			// Off to the Far Right - Move Far Left
			LineFollowingMoveFarLeft();
			LcdDisplayText("Moving FAR LEFT", objectDistanceX, objectDistanceY);
			isLineLost = false;
		}
		else if (!lineDetectionStruct.Sensor1LineDetected
			&& !lineDetectionStruct.Sensor2LineDetected
			&& !lineDetectionStruct.Sensor3LineDetected
			&& !lineDetectionStruct.Sensor4LineDetected
			&& !lineDetectionStruct.Sensor5LineDetected
			&& !lineDetectionStruct.Sensor6LineDetected
			&& !lineDetectionStruct.Sensor7LineDetected)
		{
			if (isLineLost == false)
			{
				// Start timer
				elapsedTime = 0;
			}
			else if (elapsedTime > 250)
			{
				SlowMovement();
				LcdDisplayText("SLOW MOVE", objectDistanceX, objectDistanceY);
			}
			else if (elapsedTime > 500)
			{
				// Line lost - stop movement
				StopMovement();
				LcdDisplayText("STOP MOVE", objectDistanceX, objectDistanceY);
			}

			isLineLost = true;
		}
		else
		{
			SlowMovement();
			LcdDisplayText("SLOW MOVE", objectDistanceX, objectDistanceY);
			isLineLost = false;
		}
	}
}

