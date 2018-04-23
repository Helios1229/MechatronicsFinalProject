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

///////////////////////
// STATE DEFINITIONS //
///////////////////////

// START INITIAL MOVEMENT
const int STATE_0 = 1000;		// Moving from START position to START LINE
const int STATE_1A = 1011;		// Following START LINE - Move Straight
const int STATE_1B = 1012;		// Following START LINE - Move Right (CW)
const int STATE_1C = 1013;		// Following START LINE - Move Left (CCW)
const int STATE_1D = 1014;		// Lost START LINE - Slow Movement
const int STATE_1E = 1015;		// Lost START LINE completely - Stop Movement
const int STATE_2 = 1020;		// Perform 90 degree CCW Rotation

// START CIRCLE-LINE FOLLOWING
const int STATE_3A = 1031;		// Make FarLeft-Hand (CCW) Adjustment
const int STATE_3B = 1032;		// Make Left-Hand (CCW) Adjustment
const int STATE_3C = 1033;		// Follow CW-Path (normal movement)
const int STATE_3D = 1034;		// Make Right-Hand (CW) Adjustment
const int STATE_3E = 1035;		// Make FarRight-Hand (CW) Adjustment
const int STATE_3F = 1036;		// Line lost, Slow Movement
const int STATE_3G = 1037;		// Line lost completely, Stop Movement
const int STATE_3H = 1038;		// Recover line path

// BALL-BEARING DETECTION AND PICKUP
const int STATE_4 = 1040;		// Ball detected in X-Direction, Rotate until detected in Y-Direction
const int STATE_5 = 1050;		// Ball detected in Y-Direction, Move towards it
const int STATE_6A = 1061;		// Ball at Long-Range IR threshold distance, move forward slightly to bring into range of Close-Range IR
const int STATE_6B = 1062;		// Ball at Close-Range IR threshold distance, Rotate CW to align-middle
const int STATE_7 = 1070;		// Move toward ball to align under EM
const int STATE_8 = 1080;		// Ball at threshold distance, lower pulley
const int STATE_9 = 1090;		// Turn on EM and wait small delay
const int STATE_10 = 1100;		// Raise pulley until threshold is reached

const int STATE_11 = 1110;		// TBD!!

int currentFsmState = STATE_0;	// Begin at initial starting state
bool isLineLost = false;		
bool isMagnetPoweringOn = false;
bool isBallBeingDetectedInX = false;
bool isBallBeingDetectedInY = false;
long lineLostTimer, magnetPoweringOnTimer, ballDetectedInXTimer, ballDetectedInYTimer;


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
	_finiteStateMachineProcess();
}


void _finiteStateMachineProcess()
{
	switch (currentFsmState)
	{
		////////////////////////////
		// START INITIAL MOVEMENT //
		////////////////////////////

		case (STATE_0):		// Moving from START position to START LINE
		{
			LcdDisplayText("STATE_0", "MOVETO STARTLINE");
			MoveToStartLine();
			currentFsmState = STATE_1A;
			break;
		}
		case (STATE_1A):	// Following START LINE - Move Straight
		{
			LcdDisplayText("STATE_1A", "STARTLN STRAIGHT");

			// Start moving straight
			StartLineFollowingMoveStraight();

			// Determine Line Adjustment and State Change
			StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
			currentFsmState = _startLineChangeState(adjustment);
			isLineLost = false;
			break;
		}
		case (STATE_1B):	// Following START LINE - Move Right (CW)
		{
			LcdDisplayText("STATE_1B", "STARTLN RIGHT");

			// Adjust movement to the right
			LineFollowingMoveRight();

			// Process Line-Reader Input
			LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

			// Determine Line Adjustment and State Change
			StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
			currentFsmState = _startLineChangeState(adjustment);
			isLineLost = false;
			break;
		}
		case (STATE_1C):	// Following START LINE - Move Left (CCW)
		{
			LcdDisplayText("STATE_1C", "STARTLN LEFT");

			// Adjust movement to the right
			LineFollowingMoveLeft();

			// Process Line-Reader Input
			LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

			// Determine Line Adjustment and State Change
			StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
			currentFsmState = _startLineChangeState(adjustment);
			isLineLost = false;
			break;
		}
		case (STATE_1D):
		{
			LcdDisplayText("STATE_1D", "START LOST SLOW");

			// Start elapsed timer to ensure line is lost
			if (isLineLost == false) { lineLostTimer = millis(); }

			// Wait to begin lost-line mitigation until thresholds are reached
			if ((millis() - lineLostTimer > LINE_LOST_SLOW_TIMEOUT) && (millis() - lineLostTimer < LINE_LOST_STOP_TIMEOUT))
			{
				// Line lost for greater than threshold - Adjust movement slower and straight
				StartLineFollowingSlowMovement();

				// Determine Line Adjustment
				StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
				currentFsmState = _startLineChangeState(adjustment);
			}
			else if (millis() - lineLostTimer > LINE_LOST_STOP_TIMEOUT)
			{
				// Line lost for greater than threshold - Stop movement
				currentFsmState = STATE_1E;
			}

			isLineLost = true;
			break;
		}
		case (STATE_1E):
		{
			LcdDisplayText("STATE_1E", "START LOST STOP");

			// Stop Movement
			StopMovement();
			break;
		}
		case (STATE_2):		// Perform 90 degree CCW Rotation
		{
			LcdDisplayText("STATE_2", "ROTATE 90 CCW");

			// Rotate 90 degrees CCW to being circle-line following
			Rotate90CCW();
			currentFsmState = STATE_3C;
			isLineLost = false;
			break;
		}

		/////////////////////////////////
		// START CIRCLE-LINE FOLLOWING //
		/////////////////////////////////

		case (STATE_3A):	// Make FarLeft-Hand (CCW) Adjustment
		{
			// Scan playing field for ball bearings
			int distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementXandYIRdistance("STATE_3A", distance_X, distance_Y, "MOVE FAR LEFT");

			// Adjust movement to the right
			LineFollowingMoveFarLeft();

			// Determine new state
			currentFsmState = _ballDetectionOrLineAdjustChangeState(distance_X, distance_Y);
			isLineLost = false;
			break;
		}
		case (STATE_3B):	// Make Left-Hand (CCW) Adjustment
		{
			// Scan playing field for ball bearings
			int distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementXandYIRdistance("STATE_3B", distance_X, distance_Y, "MOVE LEFT");

			// Adjust movement to the right
			LineFollowingMoveLeft();

			// Determine new state
			currentFsmState = _ballDetectionOrLineAdjustChangeState(distance_X, distance_Y);
			isLineLost = false;
			break;
		}
		case (STATE_3C):	// Follow CW-Path (normal movement)
		{
			// Scan playing field for ball bearings
			int distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementXandYIRdistance("STATE_3C", distance_X, distance_Y, "MOVE NORMAL CW");

			// Adjust movement to the right
			LineFollowingMoveCW();

			// Determine new state
			currentFsmState = _ballDetectionOrLineAdjustChangeState(distance_X, distance_Y);
			isLineLost = false;
			break;
		}
		case (STATE_3D):	// Make Right-Hand (CW) Adjustment
		{
			// Scan playing field for ball bearings
			int distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementXandYIRdistance("STATE_3D", distance_X, distance_Y, "MOVE RIGHT");

			// Adjust movement to the right
			LineFollowingMoveRight();

			// Determine new state
			currentFsmState = _ballDetectionOrLineAdjustChangeState(distance_X, distance_Y);
			isLineLost = false;
			break;
		}
		case (STATE_3E):	// Make Right-Hand (CW) Adjustment
		{
			// Scan playing field for ball bearings
			int distance_X = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::X);
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementXandYIRdistance("STATE_3E", distance_X, distance_Y, "MOVE FAR RIGHT");

			// Adjust movement to the right
			LineFollowingMoveFarRight();

			// Determine new state
			currentFsmState = _ballDetectionOrLineAdjustChangeState(distance_X, distance_Y);
			isLineLost = false;
			break;
		}
		case (STATE_3F):	// Line lost, Slow Movement
		{
			LcdDisplayText("STATE_3F", "LOST SLOW MOVE");

			// Start elapsed timer to ensure line is lost
			if (isLineLost == false) { lineLostTimer = millis(); }

			// Wait to begin lost-line mitigation until thresholds are reached
			if ((millis() - lineLostTimer > LINE_LOST_SLOW_TIMEOUT) && (millis() - lineLostTimer < LINE_LOST_STOP_TIMEOUT))
			{
				// Line lost for greater than threshold - Adjust movement slower and straight
				LineFollowingSlowMovement();

				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment);
			}
			else if (millis() - lineLostTimer > LINE_LOST_STOP_TIMEOUT)
			{
				// Line lost for greater than threshold - Stop movement
				currentFsmState = STATE_3G;
			}

			isLineLost = true;
			break;
		}
		case (STATE_3G):	// Line lost completely, Stop Movement
		{
			LcdDisplayText("STATE_3G", "LOST STOP MOVE");

			// Stop Movement
			StopMovement();
			break;
		}

		///////////////////////////////////////
		// BALL-BEARING DETECTION AND PICKUP //
		///////////////////////////////////////

		case (STATE_4):		// Ball detected in X-Direction, Rotate 90 degrees CW
		{
			// Rotate slowly CW until ball is detected in the Y-Direction
			RotateSlowCW();
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementYIRdistance("STATE_4", distance_Y, "BALL DETECT IN X");
			if (distance_Y <= MAX_BALL_CLOSE_RANGE_THRESHOLD)	{ currentFsmState = STATE_5; }
			break;
		}
		case (STATE_5):		// Ball detected in Y-Direction, Move towards it
		{
			// Advance very slowly toward ball until threshold is reached
			BallLocateVerySlowMovement();
			int distance_Y = CalculateIRDistance(SharpSensorModel::GP2Y0A60SZLF, DirectionOfIR::Y);
			LcdDisplayMovementYIRdistance("STATE_5", distance_Y, "BALL DETECT IN Y");
			if (distance_Y <= MIN_BALL_DETECTION_THRESHOLD) { currentFsmState = STATE_6A; }
			break;
		}
		case (STATE_6A):	// Ball at Long-Range IR threshold distance, move forward slightly to bring into range of Close-Range IR
		{
			LcdDisplayText("STATE_6A", "ADJUST TO CLOSE");

			// Move slightly past long-range IR threshold to position bearing into close-range IR detection
			AdjustPositionIntoCloseRange();
			currentFsmState = STATE_6B;
			break;
		}
		case (STATE_6B):	// Ball at Close-Range IR threshold distance, Rotate CW to align-middle
		{
			// Rotate slowly CW until ball is aligned with middle of robot
			RotateSlowCW();
			int distance_CR = CalculateIRDistance(SharpSensorModel::GP2Y0A51SK0F);
			LcdDisplayMovementYIRdistance("STATE_6B", distance_CR, "BALL AT CLOSE IR");
			if ((distance_CR <= MAX_BALL_CLOSE_RANGE_THRESHOLD) && (distance_CR != SHORT_RANGE_INVALID_DISTANCE)) { currentFsmState = STATE_7; }
			break;
		}
		case (STATE_7):		// Ball aligned with middle, advance slowly to pick-up threshold
		{
			// Move forward very slowly until ball is at pickup threshold
			BallLocateVerySlowMovement();
			int distance_CR = CalculateIRDistance(SharpSensorModel::GP2Y0A51SK0F);
			LcdDisplayMovementYIRdistance("STATE_7", distance_CR, "BALL ALIGNED");
			if ((distance_CR <= MIN_BALL_CLOSE_RANGE_THRESHOLD) && (distance_CR != SHORT_RANGE_INVALID_DISTANCE)) { currentFsmState = STATE_8; }
			break;
		}
		case (STATE_8):		// Ball at threshold distance, lower pulley
		{
			LcdDisplayText("STATE_8", "LOWERING PULLEY");

			// Stop movement and begin lowering pulley for specified time
			StopMovement();
			LowerPulley();
			currentFsmState = STATE_9;
			break;
		}
		case (STATE_9):		// Turn on EM and wait small delay
		{
			LcdDisplayText("STATE_9", "TURNING ON MAG");

			// Start elapsed timer to ensure magnet has enough time to power on
			if (isMagnetPoweringOn == false) 
			{
				// Turn on Electromagnet
				PowerOnMagnet();
				magnetPoweringOnTimer = millis();
			}
			if (millis() - magnetPoweringOnTimer > MAGNET_POWER_ON_DELAY)
			{
				currentFsmState = STATE_10;
				isMagnetPoweringOn = false;
			}

			isMagnetPoweringOn = true;
			break;
		}
		case (STATE_10):	// Raise pulley until threshold is reached
		{
			LcdDisplayText("STATE_10", "RAISING PULLEY");

			// Raise pulley until correct height is reached
			RaisePulley();
			currentFsmState = STATE_11;
			break;
		}
		case (STATE_11):
		{
			LcdDisplayText("STATE_11", "FINISHED");
			break;
		}
	}

	// Loop Delay
	delay(15);
}

StartLineFollowingAdjustment _adjustStartLineFollowingMovement()
{
	StartLineFollowingAdjustment adjustMovement;

	// Process Line-Reader Input
	LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

	// Check for movement modification
	if (lineDetectionStruct.Sensor2LineDetected
		&& lineDetectionStruct.Sensor3LineDetected
		&& lineDetectionStruct.Sensor4LineDetected
		&& lineDetectionStruct.Sensor5LineDetected)
	{
		adjustMovement = StartLineFollowingAdjustment::ROTATE;
	}
	else if (lineDetectionStruct.Sensor0LineDetected
		|| lineDetectionStruct.Sensor1LineDetected
		|| lineDetectionStruct.Sensor2LineDetected)
	{
		adjustMovement = StartLineFollowingAdjustment::START_RIGHT;
	}
	else if (lineDetectionStruct.Sensor5LineDetected
		|| lineDetectionStruct.Sensor6LineDetected
		|| lineDetectionStruct.Sensor7LineDetected)
	{
		adjustMovement = StartLineFollowingAdjustment::START_LEFT;
	}
	else if(!lineDetectionStruct.Sensor1LineDetected
		&& !lineDetectionStruct.Sensor2LineDetected
		&& !lineDetectionStruct.Sensor3LineDetected
		&& !lineDetectionStruct.Sensor4LineDetected
		&& !lineDetectionStruct.Sensor5LineDetected
		&& !lineDetectionStruct.Sensor6LineDetected
		&& !lineDetectionStruct.Sensor7LineDetected)
	{
		adjustMovement = StartLineFollowingAdjustment::START_LOST;
	}
	else
	{
		adjustMovement = StartLineFollowingAdjustment::START_STRAIGHT;
	}

	return adjustMovement;
}

int _startLineChangeState(StartLineFollowingAdjustment adjustment)
{
	int newState = 0;
	switch (adjustment)
	{
		case (StartLineFollowingAdjustment::START_STRAIGHT):
		{
			newState = STATE_1A;
			break;
		}
		case (StartLineFollowingAdjustment::START_RIGHT):
		{
			newState = STATE_1B;
			break;
		}
		case (StartLineFollowingAdjustment::START_LEFT):
		{
			newState = STATE_1C;
			break;
		}
		case (StartLineFollowingAdjustment::START_LOST):
		{
			newState = STATE_1D;
			break;
		}
		case (StartLineFollowingAdjustment::ROTATE):
		{
			newState = STATE_2;
			break;
		}
		default:
		{
			newState = STATE_1A;
			break;
		}
	}

	return newState;
}

CircleLineFollowingAdjustment _adjustCircleLineFollowingMovement()
{
	CircleLineFollowingAdjustment adjustMovement;

	// Process Line-Reader Input
	LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

	// Check for movement modification
	if (lineDetectionStruct.Sensor7LineDetected)
	{
		// Off to the Far Right - Move Far Left
		adjustMovement = CircleLineFollowingAdjustment::FAR_LEFT;
	}
	else if (lineDetectionStruct.Sensor6LineDetected)
	{
		// Off to the Right - Move Left
		adjustMovement = CircleLineFollowingAdjustment::LEFT;
	}
	else if (lineDetectionStruct.Sensor2LineDetected
		|| lineDetectionStruct.Sensor3LineDetected
		|| lineDetectionStruct.Sensor4LineDetected
		|| lineDetectionStruct.Sensor5LineDetected)
	{
		// Tracking Black Line - Move Normal CW
		adjustMovement = CircleLineFollowingAdjustment::NORMAL_CW;
	}
	else if (lineDetectionStruct.Sensor1LineDetected)
	{
		// Off to the Left - Move Right
		adjustMovement = CircleLineFollowingAdjustment::RIGHT;
	}
	else if (lineDetectionStruct.Sensor0LineDetected)
	{
		// Off to the Far Left - Move FarRight
		adjustMovement = CircleLineFollowingAdjustment::FAR_RIGHT;
	}
	else if (!lineDetectionStruct.Sensor1LineDetected
		&& !lineDetectionStruct.Sensor2LineDetected
		&& !lineDetectionStruct.Sensor3LineDetected
		&& !lineDetectionStruct.Sensor4LineDetected
		&& !lineDetectionStruct.Sensor5LineDetected
		&& !lineDetectionStruct.Sensor6LineDetected
		&& !lineDetectionStruct.Sensor7LineDetected)
	{
		// Line is lost
		adjustMovement = CircleLineFollowingAdjustment::LOST;
	}
	else
	{
		// No conditions satisfied, assume line is lost
		adjustMovement = CircleLineFollowingAdjustment::LOST;
	}

	return adjustMovement;
}

int _circleLineChangeState(CircleLineFollowingAdjustment adjustment)
{
	int newState = 0;
	switch (adjustment)
	{
		case CircleLineFollowingAdjustment::FAR_LEFT:
		{
			newState = STATE_3A;
			break;
		}
		case CircleLineFollowingAdjustment::LEFT:
		{
			newState = STATE_3B;
			break;
		}
		case CircleLineFollowingAdjustment::NORMAL_CW:
		{
			newState = STATE_3C;
			break;
		}
		case CircleLineFollowingAdjustment::RIGHT:
		{
			newState = STATE_3D;
			break;
		}
		case CircleLineFollowingAdjustment::FAR_RIGHT:
		{
			newState = STATE_3E;
			break;
		}
		case CircleLineFollowingAdjustment::LOST:
		{
			newState = STATE_3F;
			break;
		}
	}

	return newState;
}

int _ballDetectionOrLineAdjustChangeState(int xDistanceDetection, int yDistanceDetection)
{
	int newState = 0;
	if ((xDistanceDetection <= MAX_BALL_DETECTION_THRESHOLD) 
		&& ((xDistanceDetection < INVALID_RANGE_X_MIN) 
		|| (xDistanceDetection > INVALID_RANGE_X_MAX)))
	{
		// If first detection in this direction, begin persistence timer
		if (isBallBeingDetectedInX == false) { ballDetectedInXTimer = millis(); }

		// Check if specified persistence has elapsed, otherwise continue line-following
		if (millis() - ballDetectedInXTimer > DETECTION_PERSISTENCE)
		{
			// Ball bearing detected to the side of robot
			newState = STATE_4;
			isBallBeingDetectedInX = false;
		}
		else
		{
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			newState = _circleLineChangeState(adjustment);
		}
		isBallBeingDetectedInX = true;
	}
	else if (yDistanceDetection <= MAX_BALL_DETECTION_THRESHOLD)
	{
		// If first detection in this direction, begin persistence timer
		if (isBallBeingDetectedInY == false) { ballDetectedInYTimer = millis(); }

		// Check if specified persistence has elapsed, otherwise continue line-following
		if (millis() - ballDetectedInYTimer > DETECTION_PERSISTENCE)
		{
			// Ball bearing detected in front of robot
			newState = STATE_5;
			isBallBeingDetectedInY = false;
		}
		else
		{
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			newState = _circleLineChangeState(adjustment);
		}
		isBallBeingDetectedInY = true;
	}
	else
	{
		// Determine Line Adjustment
		CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
		newState = _circleLineChangeState(adjustment);
		isBallBeingDetectedInX = false;
		isBallBeingDetectedInY = false;
	}
	
	return newState;
}


