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

// BALL-BEARING DETECTION
const int STATE_4A = 1041;		// Possible X Left Side Detection, Stop and wait to confirm valid detect
const int STATE_4B = 1042;		// Possible X Right Side Detection, Stop and wait to confirm valid detect
const int STATE_4C = 1043;		// Possible Y Left Side Detection, Stop and wait to confirm valid detect
const int STATE_4D = 1044;		// Possible Y Right Side Detection, Stop and wait to confirm valid detect

const int STATE_5A = 1051;		// Confirmed X Left Side Detection, Rotate 135 degrees CCW
const int STATE_5B = 1052;		// Confirmed Y Left Side Detection, Rotate 45 degrees CCW
const int STATE_5C = 1053;		// Confirmed X Right Side Detection, Rotate 135 degrees CW
const int STATE_5D = 1054;		// Confirmed Y Right Side Detection, Rotate 45 degrees CW

// BALL-BEARING PICKUP
const int STATE_6 = 1060;		// Move Forward until Short Range X Left Side Detects

const int STATE_7 = 1070;		// Move toward ball to align under EM
const int STATE_8 = 1080;		// Ball at threshold distance, lower pulley
const int STATE_9 = 1090;		// Turn on EM and wait small delay
const int STATE_10 = 1100;		// Raise pulley until threshold is reached

const int STATE_11 = 1110;		// TBD!!



// Global Variables
int currentFsmState = STATE_0;					// Begin at initial starting state
int currentLongRangeLeftXDistance = 0;			// Initialize long-range left-x detection distance
int currentLongRangeLeftYDistance = 0;			// Initialize long-range left-y detection distance
int currentLongRangeRightXDistance = 0;			// Initialize long-range right-x detection distance
int currentLongRangeRightYDistance = 0;			// Initialize long-range right-y detection distance
int shortRangeLeftXDistance = 0;				// Initialize short-range left-x detection distance

int confirmedBearingDetectedDistance = 0;		// Initialize confirmed bearing distance 

long lineLostTimer, magnetPoweringOnTimer,		// Initialize timers
	possibleBallDetectionInLeftXTimer, possibleBallDetectionInLeftYTimer,
	possibleBallDetectionInRightXTimer, possibleBallDetectionInRightYTimer;

bool isPossibleBallBeingDetectedInLeftX = false;
bool isPossibleBallBeingDetectedInLeftY = false;
bool isPossibleBallBeingDetectedInRightX = false;
bool isPossibleBallBeingDetectedInRightY = false;

bool isLineLost = false;
bool isMagnetPoweringOn = false;

void setup()
{
	// Initialize devices and motors
	LcdDisplayInitialization();
	LineFollowerInitialization();
	StepperInitialization();
	RGBsensorInitialization();
	InitializeProximitySensors();
	InitializePulleyMotorControl();

	pinMode(37, INPUT);
	//Serial.begin(9600);
}


void loop()
{
	int digital = digitalRead(37);
	LcdDisplayText(digital);

	delay(50);
	//_finiteStateMachineProcess();
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
			StartLineFollowingMoveStraight();
			delay(1000);
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
			_readIRdetectiondistance();
			LcdDisplayStateAndDistance("STATE_3A", "FAR LEFT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the right
			LineFollowingMoveFarLeft();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(false);
			isLineLost = false;
			break;
		}
		case (STATE_3B):	// Make Left-Hand (CCW) Adjustment
		{
			// Scan playing field for ball bearings
			_readIRdetectiondistance();
			LcdDisplayStateAndDistance("STATE_3B", "LEFT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the right
			LineFollowingMoveLeft();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(false);
			isLineLost = false;
			break;
		}
		case (STATE_3C):	// Follow CW-Path (normal movement)
		{
			// Scan playing field for ball bearings
			_readIRdetectiondistance();
			LcdDisplayStateAndDistance("STATE_3C", "NORM CW",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the right
			LineFollowingMoveCW();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(false);
			isLineLost = false;
			break;
		}
		case (STATE_3D):	// Make Right-Hand (CW) Adjustment
		{
			// Scan playing field for ball bearings
			_readIRdetectiondistance();
			LcdDisplayStateAndDistance("STATE_3B", "RIGHT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the right
			LineFollowingMoveRight();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(false);
			isLineLost = false;
			break;
		}
		case (STATE_3E):	// Make Right-Hand (CW) Adjustment
		{
			// Scan playing field for ball bearings
			_readIRdetectiondistance();
			LcdDisplayStateAndDistance("STATE_3B", "FAR RIGHT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the right
			LineFollowingMoveFarRight();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(false);
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

		////////////////////////////
		// BALL-BEARING DETECTION //
		////////////////////////////

		case (STATE_4A):	// Possible Left-X Detection, Slow and wait to confirm valid detect
		{
			currentLongRangeLeftXDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_X_LEFT);
			LcdDisplayTextAndDistance("STATE_4A", "POSS XLEFT", currentLongRangeLeftXDistance);

			// Slow robot down and allow for continued reading
			BallDetectVerySlowMovement();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(true);
			break;
		}
		case (STATE_4B):	// Possible Left-Y Detection, Slow and wait to confirm valid detect
		{
			currentLongRangeLeftYDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_Y_LEFT);
			LcdDisplayTextAndDistance("STATE_4B", "POSS YLEFT", currentLongRangeLeftYDistance);

			// Slow robot down and allow for continued reading
			BallDetectVerySlowMovement();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(true);
			break;
		}
		case (STATE_4C):	// Possible Right-X Detection, Slow and wait to confirm valid detect
		{
			currentLongRangeRightXDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_X_RIGHT);
			LcdDisplayTextAndDistance("STATE_4C", "POSS XRIGHT", currentLongRangeRightXDistance);

			// Slow robot down and allow for continued reading
			BallDetectVerySlowMovement();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(true);
			break;
		}
		case (STATE_4D):	// Possible Right-Y Detection, Slow and wait to confirm valid detect
		{
			currentLongRangeRightYDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_Y_RIGHT);
			LcdDisplayTextAndDistance("STATE_4D", "POSS YRIGHT", currentLongRangeRightYDistance);

			// Slow robot down and allow for continued reading
			BallDetectVerySlowMovement();

			// Determine new state
			currentFsmState = _BallDetectionStateChange(true);
			break;
		}
		case (STATE_5A):	// Confirmed Left-X Detection, Rotate 135 degrees CCW
		{
			// Rotate 135 degrees CCW
			LcdDisplayText("STATE_5A", "ROTATE 135 CCW");
			Rotate135CCW();
			currentFsmState = STATE_6;
			break;
		}
		case (STATE_5B):	// Confirmed Left-Y Detection, Rotate 45 degrees CCW
		{
			// Rotate 45 degrees CCW
			LcdDisplayText("STATE_5B", "ROTATE 45 CCW");
			Rotate135CCW();
			currentFsmState = STATE_6;
			break;
		}
		case (STATE_5C):	// Confirmed Right-X Detection, Rotate 135 degrees CW
		{
			// Rotate 135 degrees CW
			LcdDisplayText("STATE_5C", "ROTATE 135 CW");
			Rotate135CW();
			currentFsmState = STATE_6;
			break;
		}
		case (STATE_5D):	// Confirmed Right-Y Detection, Rotate 45 degrees CW
		{
			// Rotate 45 degrees CCW
			LcdDisplayText("STATE_5D", "ROTATE 45 CW");
			Rotate135CCW();
			currentFsmState = STATE_6;
			break;
		}

		/////////////////////////
		// BALL-BEARING PICKUP //
		/////////////////////////

		case (STATE_6):
		{
			shortRangeLeftXDistance = CalculateShortIRDistance();
			LcdDisplayTextAndDistance("STATE_6", "WAIT SHT RNG", shortRangeLeftXDistance);

			// Continue advancing forward until the ball reaches the short range detector
			while (shortRangeLeftXDistance > SHORT_RANGE_DETECT_THRESHOLD)
			{
				BallLocateVerySlowMovement();
			}

			currentFsmState = STATE_7;
			break;
		}
		case (STATE_7):		// Ball aligned with middle, advance slowly to pick-up threshold
		{
			LcdDisplayText("STATE_7", "STOPPED");
			StopMovement();
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

void _readIRdetectiondistance()
{
	// Scan playing field for ball bearings
	currentLongRangeLeftXDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_X_LEFT);
	currentLongRangeLeftYDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_Y_LEFT);
	currentLongRangeRightXDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_X_RIGHT);
	currentLongRangeRightYDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_Y_RIGHT);
}

int _BallDetectionStateChange(bool isBallDetectionBeingConfirmed)
{
	int newState = 0;
	int detectionPersistence = 0;

	// Determine persistence time to use
	if (isBallDetectionBeingConfirmed) { detectionPersistence = INITIAL_DETECTION_PERSISTENCE; }
	else { detectionPersistence = CONFIRM_DETECTION_PERSISTENCE; }

	// Check for Left-X Detection
	if ((currentLongRangeLeftXDistance <= MAX_BALL_DETECTION_THRESHOLD) 
		&& (currentLongRangeLeftXDistance >= MIN_BALL_DETECTION_THRESHOLD))
	{
		if (isPossibleBallBeingDetectedInLeftX == false)
		{ 
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInLeftXTimer = millis();
			isPossibleBallBeingDetectedInLeftX = true;
			// Clear other detection persistence timers
			isPossibleBallBeingDetectedInLeftY = false;
			isPossibleBallBeingDetectedInRightX = false;
			isPossibleBallBeingDetectedInRightY = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInLeftXTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_4A; }
			else { newState = STATE_5A; }
			confirmedBearingDetectedDistance = currentLongRangeLeftXDistance;
			isPossibleBallBeingDetectedInLeftX = false;
		}
		else
		{
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			newState = _circleLineChangeState(adjustment);
		}
	}
	// Check for Left-Y Detection
	else if ((currentLongRangeLeftYDistance <= MAX_BALL_DETECTION_THRESHOLD)
		&& currentLongRangeLeftYDistance >= MIN_BALL_DETECTION_THRESHOLD)
	{
		if (isPossibleBallBeingDetectedInLeftY == false)
		{ 
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInLeftYTimer = millis();
			isPossibleBallBeingDetectedInLeftY = true;
			// Clear other detection persistence timers
			isPossibleBallBeingDetectedInLeftX = false;
			isPossibleBallBeingDetectedInRightX = false;
			isPossibleBallBeingDetectedInRightY = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInLeftYTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_4B; }
			else { newState = STATE_5B; }
			confirmedBearingDetectedDistance = currentLongRangeLeftYDistance;
			isPossibleBallBeingDetectedInLeftY = false;
		}
		else
		{
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			newState = _circleLineChangeState(adjustment);
		}
	}
	// Check for Right-X Detection
	else if ((currentLongRangeRightXDistance <= MAX_BALL_DETECTION_THRESHOLD)
		&& (currentLongRangeRightXDistance >= MIN_BALL_DETECTION_THRESHOLD))
	{
		if (isPossibleBallBeingDetectedInRightX == false)
		{
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInRightXTimer = millis();
			isPossibleBallBeingDetectedInRightX = true;
			// Clear other detection persistence timers
			isPossibleBallBeingDetectedInLeftX = false;
			isPossibleBallBeingDetectedInLeftY = false;
			isPossibleBallBeingDetectedInRightY = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInRightXTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_4C; }
			else { newState = STATE_5C; }
			confirmedBearingDetectedDistance = currentLongRangeRightXDistance;
			isPossibleBallBeingDetectedInRightX = false;
		}
		else
		{
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			newState = _circleLineChangeState(adjustment);
		}
	}
	// Check for Right-Y Detection
	else if ((currentLongRangeRightYDistance <= MAX_BALL_DETECTION_THRESHOLD)
		&& (currentLongRangeRightYDistance >= MIN_BALL_DETECTION_THRESHOLD))
	{
		if (isPossibleBallBeingDetectedInRightY == false)
		{
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInRightYTimer = millis();
			isPossibleBallBeingDetectedInRightY = true;
			// Clear other detection persistence timers
			isPossibleBallBeingDetectedInLeftX = false;
			isPossibleBallBeingDetectedInLeftY = false;
			isPossibleBallBeingDetectedInRightX = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInRightYTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_4D; }
			else { newState = STATE_5D; }
			confirmedBearingDetectedDistance = currentLongRangeRightYDistance;
			isPossibleBallBeingDetectedInRightY = false;
		}
		else
		{
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			newState = _circleLineChangeState(adjustment);
		}
	}
	// No possible detections currently active, continue line following
	else
	{
		// Determine Line Adjustment
		CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
		newState = _circleLineChangeState(adjustment);
		// Clear all detection persistence timers
		isPossibleBallBeingDetectedInLeftX = false;
		isPossibleBallBeingDetectedInLeftY = false;
		isPossibleBallBeingDetectedInRightX = false;
		isPossibleBallBeingDetectedInRightY = false;
	}
	
	return newState;
}
