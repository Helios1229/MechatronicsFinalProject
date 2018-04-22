/*/*
 Name:		ME675_Final_Project.ino
 Created:	3/26/2018 7:15:02 PM
 Author:	Jon
*/

#include <LiquidCrystal.h>
#include <Adafruit_TCS34725.h>
#include "ME675_FinalProject.h"
#include "LcdDisplay.h"
#include "LineFollower.h"
#include "StepperControl.h"
#include "ProximitySensors.h"
#include "RGB_Sensor.h"
#include "PulleyControl.h"
#include "elapsedMillis.h"

bool isOnStartingLine = true;
bool isLineLost = false;
bool isMovingToBall = false;
bool isPickingUpBall = false;
elapsedMillis elapsedTime;

// TEST
bool rotate = true;

void setup()
{
	// Initialize devices and motors
	LcdDisplayInitialization();
	LineFollowerInitialization();
	StepperInitialization();
	RGBsensorInitialization();
	InitializeProximitySensors();
	InitializePulleyMotorControl();

	//Serial.begin(9600);
}


void loop()
{
	//FiniteStateMachineProcess();
	if (isPickingUpBall == false)
	{
		LowerPulley();
		PowerOnMagnet();
		delay(2000);
		RaisePulley();
		isPickingUpBall = true;
	}
}

void FiniteStateMachineProcess()
{
	// Process Line-Reader Input
	LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

	if (isOnStartingLine)
	{
		// STATE 1: Perform Line Following on Starting Path
		StartPathFollowing(lineDetectionStruct);
	}
	else
	{
		// Process RGB Input
		RGBreadingStructure rgbInput = RGBreadColor();

		// Process X and Y IR Sensors
		int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
		int distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);

		// STATE 2: Perform Line Following around Main Circle
		if (distance_Y > 30 && isMovingToBall == false)
		{
			LineFollowing(lineDetectionStruct);
		}
		// STATE 3: Ball detected in Y-Direction
		else if (distance_Y < 30 || isMovingToBall == true)
		{
			isMovingToBall = true;

			// STATE 3A: Move towards ball
			if (distance_Y > 10)
			{
				LcdDisplayText("BALL Y - SLOW", distance_Y);
				BallLocateSlowMovement();
			}
			// STATE 3B: Pick up ball
			else
			{
				LcdDisplayText("BALL Y - PICK", distance_Y);
				StopMovement();
				if (isPickingUpBall == false)
				{
					BallPickupRotation();
					LowerPulley();
					PowerOnMagnet();
					delay(2000);
					RaisePulley();
				}
				isMovingToBall = false;
				isPickingUpBall = true;
			}
		}
	}

	// Loop Delay
	delay(25);
}

void StartPathFollowing(LineDetectionStructure lineDetectionStruct)
{
	if (lineDetectionStruct.Sensor2LineDetected
		&& lineDetectionStruct.Sensor3LineDetected
		&& lineDetectionStruct.Sensor4LineDetected
		&& lineDetectionStruct.Sensor5LineDetected)
	{
		// Circle Reached - perform 90 degree CCW Rotation
		LcdDisplayText("START PATH", "Rotate 90 degrees");
		Rotate90CCW();
		isOnStartingLine = false;
	}
	else if (lineDetectionStruct.Sensor0LineDetected
		|| lineDetectionStruct.Sensor1LineDetected
		|| lineDetectionStruct.Sensor2LineDetected)
	{
		LcdDisplayText("START PATH", "MOVE RIGHT");
		LineFollowingMoveRight();
	}
	else if (lineDetectionStruct.Sensor5LineDetected
		|| lineDetectionStruct.Sensor6LineDetected
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

void LineFollowing(LineDetectionStructure lineDetectionStruct)
{
	if (lineDetectionStruct.Sensor2LineDetected
		|| lineDetectionStruct.Sensor3LineDetected
		|| lineDetectionStruct.Sensor4LineDetected
		|| lineDetectionStruct.Sensor5LineDetected)
	{
		// Tracking Black Line
		LineFollowingMoveCW();
		LcdDisplayText("Moving CW");
		isLineLost = false;
	}
	else if (lineDetectionStruct.Sensor1LineDetected)
	{
		// Off to the Left - Move Right
		LineFollowingMoveRight();
		LcdDisplayText("Moving RIGHT");
		isLineLost = false;
	}
	else if (lineDetectionStruct.Sensor0LineDetected)
	{
		// Off to the Far Left - Move Far Right
		LineFollowingMoveFarRight();
		LcdDisplayText("Moving FAR RIGHT");
		isLineLost = false;
	}
	else if (lineDetectionStruct.Sensor6LineDetected)
	{
		// Off to the Right - Move Left
		LineFollowingMoveLeft();
		LcdDisplayText("Moving LEFT");
		isLineLost = false;
	}
	else if (lineDetectionStruct.Sensor7LineDetected)
	{
		// Off to the Far Right - Move Far Left
		LineFollowingMoveFarLeft();
		LcdDisplayText("Moving FAR LEFT");
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
		else if (elapsedTime > 500 && elapsedTime < 2500)
		{
			LineFollowingSlowMovement();
			LcdDisplayText("SLOW MOVE");
		}
		else if (elapsedTime >= 2500)
		{
			// Line lost - stop movement
			StopMovement();
			LcdDisplayText("STOP MOVE");
		}

		isLineLost = true;
	}
	else
	{
		LineFollowingSlowMovement();
		LcdDisplayText("SLOW MOVE");
		isLineLost = false;
	}

}

