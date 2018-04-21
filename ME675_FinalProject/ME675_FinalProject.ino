/*
 Name:		ME675_Final_Project.ino
 Created:	3/26/2018 7:15:02 PM
 Author:	Jon
*/

#include "ME675_FinalProject.h"
#include "LcdDisplay.h"
#include "LineFollower.h"
#include "StepperControl.h"
#include "ProximitySensors.h"
#include "RGB_Sensor.h"
#include "elapsedMillis.h"

bool isOnStartingLine = true;
bool isLineLost = false;
elapsedMillis elapsedTime;

void setup()
{
	// Initialize devices and motors
	LcdDisplayInitialization();
	LineFollowerInitialization();
	StepperInitialization();
	RGBsensorInitialization();
	InitializeProximitySensors();

	Serial.begin(9600);
}


void loop()
{
	//FiniteStateMachineProcess();

}

void FiniteStateMachineProcess()
{
	// Process Line-Reader Input
	LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

	// Process RGB Input
	RGBreadingStructure rgbInput = RGBreadColor();

	// Process X and Y IR Sensors
	float distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
	float distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);
	delay(50);

	if (isOnStartingLine)
	{
		// STATE 1: Perform Line Following on Starting Path
		StartPathFollowing(lineDetectionStruct);
	}
	else
	{
		// STATE 2: Perform Line Following around Main Circle
		LineFollowing(lineDetectionStruct, distance_X, distance_Y);
	}
}

void StartPathFollowing(LineDetectionStructure lineDetectionStruct)
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

void LineFollowing(LineDetectionStructure lineDetectionStruct, float objectDistanceX, float objectDistanceY)
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

