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
const int STATE_4A = 1041;		// Possible X Left Side Detection, Slow and wait to confirm valid detect
const int STATE_4B = 1042;		// Possible X Right Side Detection, Slow and wait to confirm valid detect
const int STATE_4C = 1043;		// Possible Y Left Side Detection, Slow and wait to confirm valid detect
const int STATE_4D = 1044;		// Possible Y Right Side Detection, Slow and wait to confirm valid detect

const int STATE_5A = 1051;		// Confirmed X Left Side Detection, Rotate 135 degrees CCW
const int STATE_5B = 1052;		// Confirmed Y Left Side Detection, Rotate 45 degrees CCW
const int STATE_5C = 1053;		// Confirmed X Right Side Detection, Rotate 135 degrees CW
const int STATE_5D = 1054;		// Confirmed Y Right Side Detection, Rotate 45 degrees CW

// BALL-BEARING PICKUP
const int STATE_6A = 1061;		// Move Forward until Short Range X Left Side Detects
const int STATE_6B = 1062;		// False detection occurred, reverse until black line is found
const int STATE_7A = 1071;		// Read in digital IR Array
const int STATE_7B = 1072;		// Left sensors active, rotate CCW until middle detects
const int STATE_7C = 1073;		// Right sensors active, rotate CW until middle detects
const int STATE_7D = 1074;		// No sensors active, move forward until something detects
const int STATE_7E = 1075;		// Middle sensor active, reverse until ball is underneath magnet
const int STATE_8 = 1080;		// Ball at threshold distance, lower pulley
const int STATE_9 = 1090;		// Turn on EM and wait small delay
const int STATE_10 = 1100;		// Raise pulley until threshold is reached

// Z-BOT LOCATION AND DROPOFF
const int STATE_11 = 1110;		// Rotate 180 degrees
const int STATE_12 = 1120;		// Move forward until the black line is found
const int STATE_13A = 1131;		// If ball was found in the circle, rotate 90 degrees CW to continue line following
const int STATE_13B = 1132;		// If ball was found outside the circle, rotate 90 degrees CCW to continue line following
const int STATE_14A = 1141;		// Make FarLeft-Hand (CCW) Adjustment while searching for Z-Bot
const int STATE_14B = 1142;		// Make Left-Hand (CCW) Adjustment while searching for Z-Bot
const int STATE_14C = 1143;		// Follow CW-Path (normal movement) while searching for Z-Bot
const int STATE_14D = 1144;		// Make Right-Hand (CW) Adjustment while searching for Z-Bot
const int STATE_14E = 1145;		// Make FarRight-Hand (CW) Adjustment while searching for Z-Bot
const int STATE_14F = 1146;		// Line lost, Slow Movement while searching for Z-Bot
const int STATE_14G = 1147;		// Line lost completely, Stop Movement while searching for Z-Bot
const int STATE_14H = 1148;		// Recover line path while searching for Z-Bot
const int STATE_15 = 1150;		// Z-Bot detected, rotate 90CCW
const int STATE_16A = 1161;		// Move straight to Z-Bot
const int STATE_16B = 1162;		// Right move adjustment to Z-Bot
const int STATE_16C = 1163;		// Left move adjustment to Z-Bot
const int STATE_16D = 1164;		// Lost Line to Z-Bot, Slow Movement
const int STATE_16E = 1165;		// Line lost completely to Z-Bot, Stop movement
const int STATE_17 = 1170;		// Z-Bot Reached, deposit ball



// Global Variables
int currentFsmState = STATE_0;					// Begin at initial starting state
int currentLongRangeLeftXDistance = 0;			// Initialize long-range left-x detection distance
int currentLongRangeLeftYDistance = 0;			// Initialize long-range left-y detection distance
int currentLongRangeRightXDistance = 0;			// Initialize long-range right-x detection distance
int currentLongRangeRightYDistance = 0;			// Initialize long-range right-y detection distance
int shortRangeLeftXDistance = 0;				// Initialize short-range left-x detection distance

int ultrasonicLeftXDistance = 0;				// Initialize ultrasonic left-x detection distance
int ultrasonicLeftYDistance = 0;				// Initialize ultrasonic left-y detection distance

int confirmedBearingDetectedDistance = 0;		// Initialize confirmed bearing distance 

long lineLostTimer, magnetPoweringOnTimer,		// Initialize timers
	possibleBallDetectionInLeftXTimer, possibleBallDetectionInLeftYTimer,
	possibleBallDetectionInRightXTimer, possibleBallDetectionInRightYTimer, falseDetectionTimer;

bool isPossibleBallBeingDetectedInLeftX = false;
bool isPossibleBallBeingDetectedInLeftY = false;
bool isPossibleBallBeingDetectedInRightX = false;
bool isPossibleBallBeingDetectedInRightY = false;
bool isCheckingForFalseDetection = false;
bool isBallLocatedInsideTheCircle = false;

bool isLineLost = false;
bool isMagnetPoweringOn = false;

// Competition Checklist
bool testZbotSafety = false;
bool testRobotMovement = false;
bool testLiftingMechanism = false;
bool testSizeDetection = false;
bool isTestComplete = false;


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
			currentFsmState = _startLineChangeState(adjustment,false);
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
			currentFsmState = _startLineChangeState(adjustment,false);
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
			currentFsmState = _startLineChangeState(adjustment,false);
			isLineLost = false;
			break;
		}
		case (STATE_1D):	// Lost START LINE - Slow Movement
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
				currentFsmState = _startLineChangeState(adjustment,false);
			}
			else if (millis() - lineLostTimer > LINE_LOST_STOP_TIMEOUT)
			{
				// Line lost for greater than threshold - Stop movement
				currentFsmState = STATE_1E;
			}

			isLineLost = true;
			break;
		}
		case (STATE_1E):	// Lost START LINE completely - Stop Movement
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
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayStateAndDistance("STATE_3A", "FAR LEFT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the far left
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
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayStateAndDistance("STATE_3B", "LEFT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the left
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
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayStateAndDistance("STATE_3C", "NORM CW",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Follow normal CW movement
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
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
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
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayStateAndDistance("STATE_3B", "FAR RIGHT",
				currentLongRangeLeftXDistance, currentLongRangeLeftYDistance, currentLongRangeRightXDistance, currentLongRangeRightYDistance);

			// Adjust movement to the far right
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
				currentFsmState = _circleLineChangeState(adjustment,false);
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
			//currentLongRangeLeftYDistance = 0;
			//currentLongRangeRightXDistance = 0;
			//currentLongRangeRightYDistance = 0;
			
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
			//currentLongRangeLeftXDistance = 0;
			//currentLongRangeRightXDistance = 0;
			//currentLongRangeRightYDistance = 0;
			
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
			//currentLongRangeLeftXDistance = 0;
			//currentLongRangeLeftYDistance = 0;
			//currentLongRangeRightYDistance = 0;
			
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
			//currentLongRangeLeftXDistance = 0;
			//currentLongRangeLeftYDistance = 0;
			//currentLongRangeRightXDistance = 0;
			
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
			isBallLocatedInsideTheCircle = false;
			currentFsmState = STATE_6A;
			break;
		}
		case (STATE_5B):	// Confirmed Left-Y Detection, Rotate 45 degrees CCW
		{
			// Rotate 45 degrees CCW
			LcdDisplayText("STATE_5B", "ROTATE 45 CCW");
			Rotate45CCW();
			isBallLocatedInsideTheCircle = false;
			currentFsmState = STATE_6A;
			break;
		}
		case (STATE_5C):	// Confirmed Right-X Detection, Rotate 135 degrees CW
		{
			// Rotate 135 degrees CW
			LcdDisplayText("STATE_5C", "ROTATE 135 CW");
			Rotate135CW();
			isBallLocatedInsideTheCircle = true;
			currentFsmState = STATE_6A;
			break;
		}
		case (STATE_5D):	// Confirmed Right-Y Detection, Rotate 45 degrees CW
		{
			// Rotate 45 degrees CCW
			LcdDisplayText("STATE_5D", "ROTATE 45 CW");
			Rotate45CW();
			isBallLocatedInsideTheCircle = true;
			currentFsmState = STATE_6A;
			break;
		}

		/////////////////////////
		// BALL-BEARING PICKUP //
		/////////////////////////

		case (STATE_6A):
		{
			if (isCheckingForFalseDetection == false)
			{
				falseDetectionTimer = millis();
				isCheckingForFalseDetection = true;
			}

			if (millis() - falseDetectionTimer > FALSE_DETECTION_TIMEOUT) { currentFsmState = STATE_6B; }
			else
			{
				shortRangeLeftXDistance = CalculateShortIRDistance();
				DigitalCloseRangeArray digitalArray = CalculateDigitalIRarray();
				LcdDisplayTextAndDistance("STATE_6A", "WAIT SHRT RG", shortRangeLeftXDistance);

				if (((shortRangeLeftXDistance > SHORT_RANGE_MAX_THRESHOLD) || (shortRangeLeftXDistance < SHORT_RANGE_DETECT_THRESHOLD))
					&& !digitalArray.isIRleft3Detected
					&& !digitalArray.isIRleft2Detected
					&& !digitalArray.isIRleft1Detected
					&& !digitalArray.isIRmiddleDetected
					&& !digitalArray.isIRright1Detected
					&& !digitalArray.isIRright2Detected
					&& !digitalArray.isIRright3Detected)
				{
					// Continue advancing forward until the ball reaches the short range detector
					BallLocateSlowMovement();
					shortRangeLeftXDistance = CalculateShortIRDistance();
					digitalArray = CalculateDigitalIRarray();
				}
				else
				{
					currentFsmState = STATE_7A;
					isCheckingForFalseDetection = false;
				}

			}
			break;
		}
		case (STATE_6B):	// False detection occurred, reverse until black line is found
		{
			LcdDisplayText("STATE_6B", "REVERSE!");

			if (isCheckingForFalseDetection == true)
			{
				falseDetectionTimer = millis();
				isCheckingForFalseDetection = false;
			}

			while (millis() - falseDetectionTimer < (FALSE_DETECTION_TIMEOUT+1750)) { ReverseMovement();;}
			currentFsmState = STATE_3C;

			break;
		}
		case (STATE_7A):	// Read in digital IR Array
		{
			LcdDisplayText("STATE_7A", "READ DIGITAL ARR");

			// Stop movement and read in current digital array
			DigitalSensorDetectVerySlowMovement();
			DigitalCloseRangeArray digitalArray = CalculateDigitalIRarray();

			// Determine next state from current active sensors
			if ((digitalArray.isIRleft3Detected
				|| digitalArray.isIRleft2Detected
				|| digitalArray.isIRleft1Detected)
				&& !digitalArray.isIRmiddleDetected)
			{
				currentFsmState = STATE_7B;
			}
			else if ((digitalArray.isIRright3Detected
				|| digitalArray.isIRright2Detected
				|| digitalArray.isIRright1Detected)
				&& !digitalArray.isIRmiddleDetected)
			{
				currentFsmState = STATE_7C;
			}
			else if (!digitalArray.isIRright3Detected
				&& !digitalArray.isIRright2Detected
				&& !digitalArray.isIRright1Detected
				&& !digitalArray.isIRmiddleDetected
				&& !digitalArray.isIRleft1Detected
				&& !digitalArray.isIRleft2Detected
				&& !digitalArray.isIRleft3Detected)
			{
				currentFsmState = STATE_7D;
			}
			else
			{
				currentFsmState = STATE_7E;
			}

			//LcdDisplayDigitalSensors(digitalArray);
			break;
		}
		case (STATE_7B):	// Left sensors active, rotate CW until middle detects
		{
			LcdDisplayText("STATE_7B", "ROTATE CCW");
			DigitalCloseRangeArray digitalArray = CalculateDigitalIRarray();
			while (!digitalArray.isIRmiddleDetected)
			{
				RotateSlowCCW();
				digitalArray = CalculateDigitalIRarray();
			}
			currentFsmState = STATE_7E;
			break;
		}
		case (STATE_7C):	// Right sensors active, rotate CCW until middle detects
		{
			LcdDisplayText("STATE_7C", "ROTATE CW");
			DigitalCloseRangeArray digitalArray = CalculateDigitalIRarray();
			while (!digitalArray.isIRmiddleDetected)
			{
				RotateSlowCW();
				digitalArray = CalculateDigitalIRarray();
			}
			currentFsmState = STATE_7E;
			break;
		}
		case (STATE_7D):	// No sensors active, move forward until something detects
		{
			LcdDisplayText("STATE_7D", "MOVE FORWARD");
			AdjustPositionIntoCloseRange();

			DigitalCloseRangeArray digitalArray = CalculateDigitalIRarray();
			if ((digitalArray.isIRleft3Detected
				|| digitalArray.isIRleft2Detected
				|| digitalArray.isIRleft1Detected)
				&& !digitalArray.isIRmiddleDetected)
			{
				currentFsmState = STATE_7B;
			}
			else if ((digitalArray.isIRright3Detected
				|| digitalArray.isIRright2Detected
				|| digitalArray.isIRright1Detected)
				&& !digitalArray.isIRmiddleDetected)
			{
				currentFsmState = STATE_7C;
			}
			else if (digitalArray.isIRmiddleDetected)
			{
				currentFsmState = STATE_7E;
			}
			else
			{
				currentFsmState = STATE_7D;
			}
			break;
		}
		case (STATE_7E):	// Middle sensor active, reverse until ball is underneath magnet
		{
			DigitalCloseRangeArray digitalArray = CalculateDigitalIRarray();
			if (digitalArray.isIRmiddleDetected 
				&& (!digitalArray.isIRright1Detected || !digitalArray.isIRleft1Detected))
			{
				LcdDisplayText("STATE_7E", "MEDIUM DETECTED");
				delay(2000);
			}
			else if(digitalArray.isIRmiddleDetected 
				&& digitalArray.isIRright1Detected 
				&& digitalArray.isIRleft1Detected)
			{
				LcdDisplayText("STATE_7E", "MEDIUM DETECTED");
				delay(2000);
			}
			else if (digitalArray.isIRmiddleDetected
				&& digitalArray.isIRright1Detected
				&& digitalArray.isIRright2Detected
				&& digitalArray.isIRleft1Detected
				&& digitalArray.isIRleft2Detected)
			{
				LcdDisplayText("STATE_7E", "LARGE DETECTED");
				delay(2000);
			}
			else
			{
				LcdDisplayText("STATE_7E", "MEDIUM DETECTED");
				delay(2000);
			}

			ReverseUnderMagnet();
			currentFsmState = STATE_8;
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

		////////////////////////////////
		// Z-BOT LOCATION AND DROPOFF //
		////////////////////////////////

		case (STATE_11):	// Clear Screen
		{
			LcdDisplayInitialization();
			delay(25);
			LcdDisplayText("STATE_11", "CLEAR SCREEN");
			//Rotate180();
			ReverseMovement();
			currentFsmState = STATE_12;
			break;
		}
		case (STATE_12):	// Move forward until the black line is found
		{
			LcdDisplayText("STATE_12", "REVERSE TO LINE");

			// Process Line-Reader Input
			LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();
			while (!lineDetectionStruct.Sensor2LineDetected &&
				!lineDetectionStruct.Sensor3LineDetected &&
				!lineDetectionStruct.Sensor4LineDetected &&
				!lineDetectionStruct.Sensor5LineDetected)
			{
				ReverseMovement();
				lineDetectionStruct = ProcessLineFollowerInput();
			}

			if(isBallLocatedInsideTheCircle){ currentFsmState = STATE_13B; }
			else{ currentFsmState = STATE_13A; }
			break;
		}
		case(STATE_13A):	// If ball was found in the circle, rotate 45 degrees CW to continue line following
		{
			LcdDisplayText("STATE_13A", "ROTATE 45CW");
			Rotate45CW();
			
			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			currentFsmState = _circleLineChangeState(adjustment, true);
			break;
		}
		case(STATE_13B):	// If ball was found outside the circle, rotate 45 degrees CCW to continue line following
		{
			LcdDisplayText("STATE_13B", "ROTATE 90CCW");

			Rotate45CCW();

			// Determine Line Adjustment
			CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
			currentFsmState = _circleLineChangeState(adjustment,true);

			break;
		}
		case(STATE_14A):	// Make FarLeft-Hand (CCW) Adjustment while searching for Z-Bot
		{
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayTextAndDistance("STATE_14A", "ZS-FAR LEFT", ultrasonicLeftXDistance);

			// Adjust movement to the far left
			LineFollowingMoveFarLeft();

			if (ultrasonicLeftXDistance < ZBOT_DETECTION_DISTANCE) { currentFsmState = STATE_15; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case(STATE_14B):	// Make Left-Hand (CCW) Adjustment while searching for Z-Bot
		{
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayTextAndDistance("STATE_14B", "ZS-LEFT", ultrasonicLeftXDistance);

			// Adjust movement to the left
			LineFollowingMoveLeft();

			if (ultrasonicLeftXDistance < ZBOT_DETECTION_DISTANCE) { currentFsmState = STATE_15; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case(STATE_14C):	// Follow CW-Path (normal movement) while searching for Z-Bot
		{
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayTextAndDistance("STATE_14C", "ZS-NORMAL CW", ultrasonicLeftXDistance);

			// Normal line following movement
			LineFollowingMoveCW();

			if (ultrasonicLeftXDistance < ZBOT_DETECTION_DISTANCE) { currentFsmState = STATE_15; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case(STATE_14D):	// Make Right-Hand (CW) Adjustment while searching for Z-Bot
		{
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayTextAndDistance("STATE_14D", "ZS-RIGHT", ultrasonicLeftXDistance);

			// Adjust movement to the right
			LineFollowingMoveRight();

			if (ultrasonicLeftXDistance < ZBOT_DETECTION_DISTANCE) { currentFsmState = STATE_15; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case(STATE_14E):	// Make FarRight-Hand (CW) Adjustment while searching for Z-Bot
		{
			ultrasonicLeftXDistance = CalculateUltrasonicDistance(DirectionOfUS::US_X);
			LcdDisplayTextAndDistance("STATE_14E", "ZS-FAR RIGHT", ultrasonicLeftXDistance);

			// Adjust movement to the far right
			LineFollowingMoveFarRight();

			if (ultrasonicLeftXDistance < ZBOT_DETECTION_DISTANCE) { currentFsmState = STATE_15; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case (STATE_14F):	// Line lost, Slow Movement while searching for Z-Bot
		{
			LcdDisplayText("STATE_14F", "ZS-LOST SLOW");

			// Start elapsed timer to ensure line is lost
			if (isLineLost == false) { lineLostTimer = millis(); }

			// Wait to begin lost-line mitigation until thresholds are reached
			if ((millis() - lineLostTimer > LINE_LOST_SLOW_TIMEOUT) && (millis() - lineLostTimer < LINE_LOST_STOP_TIMEOUT))
			{
				// Line lost for greater than threshold - Adjust movement slower and straight
				LineFollowingSlowMovement();

				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				currentFsmState = _circleLineChangeState(adjustment, false);
			}
			else if (millis() - lineLostTimer > LINE_LOST_STOP_TIMEOUT)
			{
				// Line lost for greater than threshold - Stop movement
				currentFsmState = STATE_14G;
			}

			isLineLost = true;
			break;
		}
		case (STATE_14G):	// Line lost completely, Stop Movement while searching for Z-Bot
		{
			LcdDisplayText("STATE_14G", "ZS-LOST STOP");

			// Stop Movement
			StopMovement();
			break;
		}
		case (STATE_15):	// Z-Bot detected, rotate 90CCW
		{
			LcdDisplayText("STATE_15", "ZFOUND ROT90CCW");
			Rotate90CCW();
			currentFsmState = STATE_16A;
			break;
		}
		case (STATE_16A):	// Move straight to Z-Bot
		{
			ultrasonicLeftYDistance = CalculateUltrasonicDistance(DirectionOfUS::US_Y);
			LcdDisplayTextAndDistance("STATE_16A", "MOVE ST TO Z", ultrasonicLeftYDistance);

			// Start moving straight
			StartLineFollowingMoveStraight();

			if (ultrasonicLeftYDistance < ZBOT_DEPOSIT_DISTANCE) { currentFsmState = STATE_17; }
			else 
			{
				// Determine Line Adjustment and State Change
				StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
				currentFsmState = _startLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case (STATE_16B):	// Right move adjustment to Z-Bot
		{
			ultrasonicLeftYDistance = CalculateUltrasonicDistance(DirectionOfUS::US_Y);
			LcdDisplayTextAndDistance("STATE_16B", "MOVE RIGHT TO Z", ultrasonicLeftYDistance);

			// Adjust movement to the right
			LineFollowingMoveRight();

			if (ultrasonicLeftYDistance < ZBOT_DEPOSIT_DISTANCE) { currentFsmState = STATE_17; }
			else
			{
				// Process Line-Reader Input
				LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

				// Determine Line Adjustment and State Change
				StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
				currentFsmState = _startLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case (STATE_16C):	// Left move adjustment to Z-Bot
		{
			ultrasonicLeftYDistance = CalculateUltrasonicDistance(DirectionOfUS::US_Y);
			LcdDisplayTextAndDistance("STATE_16C", "MOVE LEFT TO Z", ultrasonicLeftYDistance);

			// Adjust movement to the right
			LineFollowingMoveLeft();

			if (ultrasonicLeftYDistance < ZBOT_DEPOSIT_DISTANCE) { currentFsmState = STATE_17; }
			else
			{
				// Process Line-Reader Input
				LineDetectionStructure lineDetectionStruct = ProcessLineFollowerInput();

				// Determine Line Adjustment and State Change
				StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
				currentFsmState = _startLineChangeState(adjustment, true);
			}
			isLineLost = false;
			break;
		}
		case (STATE_16D):	// Lost Line to Z-Bot, Slow Movement
		{
			LcdDisplayText("STATE_16D", "LOST TO Z SLOW");

			// Start elapsed timer to ensure line is lost
			if (isLineLost == false) { lineLostTimer = millis(); }

			// Wait to begin lost-line mitigation until thresholds are reached
			if ((millis() - lineLostTimer > LINE_LOST_SLOW_TIMEOUT) && (millis() - lineLostTimer < LINE_LOST_STOP_TIMEOUT))
			{
				// Line lost for greater than threshold - Adjust movement slower and straight
				StartLineFollowingSlowMovement();

				// Determine Line Adjustment
				StartLineFollowingAdjustment adjustment = _adjustStartLineFollowingMovement();
				currentFsmState = _startLineChangeState(adjustment, true);
			}
			else if (millis() - lineLostTimer > LINE_LOST_STOP_TIMEOUT)
			{
				// Line lost for greater than threshold - Stop movement
				currentFsmState = STATE_16E;
			}

			isLineLost = true;
			break;
		}
		case (STATE_16E):	// Line lost completely to Z-Bot, Stop movement
		{
			LcdDisplayText("STATE_16E", "LOSE TO Z STOP");

			// Stop Movement
			StopMovement();
			break;
		}
		case (STATE_17):	// Z-Bot Reached, deposit ball
		{
			StopMovement();
			PowerOffMagnet();
			break;
		}

	}

	// Loop Delay
	delay(25);
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

int _startLineChangeState(StartLineFollowingAdjustment adjustment, bool isZbotBeingDetected)
{
	int newState = 0;
	switch (adjustment)
	{
		case (StartLineFollowingAdjustment::START_STRAIGHT):
		{
			if (isZbotBeingDetected) { newState = STATE_16A; }
			else{ newState = STATE_1A; }
			break;
		}
		case (StartLineFollowingAdjustment::START_RIGHT):
		{
			if (isZbotBeingDetected) { newState = STATE_16B; }
			else { newState = STATE_1B; }
			break;
		}
		case (StartLineFollowingAdjustment::START_LEFT):
		{
			if (isZbotBeingDetected) { newState = STATE_16C; }
			else { newState = STATE_1C; }
			break;
		}
		case (StartLineFollowingAdjustment::START_LOST):
		{
			if (isZbotBeingDetected) { newState = STATE_16D; }
			else { newState = STATE_1D; }
			break;
		}
		case (StartLineFollowingAdjustment::ROTATE):
		{
			if (isZbotBeingDetected) { newState = STATE_16A; }
			else { newState = STATE_2; }
			break;
		}
		default:
		{
			if (isZbotBeingDetected) { newState = STATE_16A; }
			else { newState = STATE_1A; }
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

int _circleLineChangeState(CircleLineFollowingAdjustment adjustment, bool isZbotBeingDetected)
{
	int newState = 0;
	switch (adjustment)
	{
		case CircleLineFollowingAdjustment::FAR_LEFT:
		{
			if (isZbotBeingDetected) { newState = STATE_14A; }
			else{ newState = STATE_3A; }
			break;
		}
		case CircleLineFollowingAdjustment::LEFT:
		{
			if (isZbotBeingDetected) { newState = STATE_14B; }
			else{ newState = STATE_3B; }
			break;
		}
		case CircleLineFollowingAdjustment::NORMAL_CW:
		{
			if (isZbotBeingDetected) { newState = STATE_14C; }
			else { newState = STATE_3C; }
			break;
		}
		case CircleLineFollowingAdjustment::RIGHT:
		{
			if (isZbotBeingDetected) { newState = STATE_14D; }
			else { newState = STATE_3D; }

			break;
		}
		case CircleLineFollowingAdjustment::FAR_RIGHT:
		{
			if (isZbotBeingDetected) { newState = STATE_14E; }
			else { newState = STATE_3E; }
			break;
		}
		case CircleLineFollowingAdjustment::LOST:
		{
			if (isZbotBeingDetected) { newState = STATE_14F; }
			else { newState = STATE_3F; }
			break;
		}
	}

	return newState;
}

void _readIRdetectiondistance()
{
	// Scan playing field for ball bearings
	//currentLongRangeLeftXDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_X_LEFT);
	currentLongRangeLeftYDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_Y_LEFT);
	//currentLongRangeRightXDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_X_RIGHT);
	currentLongRangeRightYDistance = CalculateLongIRDistance(DirectionOfLongIR::IR_Y_RIGHT);
}

int _BallDetectionStateChange(bool isBallDetectionBeingConfirmed)
{
	int newState = 0;
	int detectionPersistence = 0;

	// Determine persistence time to use
	if (isBallDetectionBeingConfirmed) { detectionPersistence = CONFIRM_DETECTION_PERSISTENCE; }
	else { detectionPersistence = INITIAL_DETECTION_PERSISTENCE; }

	// Check for Left-X Detection
	if ((currentLongRangeLeftXDistance <= MAX_BALL_DETECTION_THRESHOLD) 
		&& (currentLongRangeLeftXDistance >= MIN_BALL_DETECTION_THRESHOLD)
		&& (ultrasonicLeftXDistance > ZBOT_DETECTION_DISTANCE))
	{
		if (isPossibleBallBeingDetectedInLeftX == false)
		{ 
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInLeftXTimer = millis();
			isPossibleBallBeingDetectedInLeftX = true;
			// Clear other detection persistence timers
			//isPossibleBallBeingDetectedInLeftY = false;
			//isPossibleBallBeingDetectedInRightX = false;
			//isPossibleBallBeingDetectedInRightY = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInLeftXTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_5A; }
			else { newState = STATE_4A; }
			confirmedBearingDetectedDistance = currentLongRangeLeftXDistance;
			isPossibleBallBeingDetectedInLeftX = false;
		}
		else
		{
			if (isBallDetectionBeingConfirmed) { newState = STATE_4A; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				newState = _circleLineChangeState(adjustment, false);
			}
		}
	}
	// Check for Left-Y Detection
	else if ((currentLongRangeLeftYDistance <= MAX_BALL_DETECTION_THRESHOLD)
		&& (currentLongRangeLeftYDistance >= MIN_BALL_DETECTION_THRESHOLD)
		&& (ultrasonicLeftXDistance > ZBOT_DETECTION_DISTANCE))
	{
		if (isPossibleBallBeingDetectedInLeftY == false)
		{ 
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInLeftYTimer = millis();
			isPossibleBallBeingDetectedInLeftY = true;
			// Clear other detection persistence timers
			//isPossibleBallBeingDetectedInLeftX = false;
			//isPossibleBallBeingDetectedInRightX = false;
			//isPossibleBallBeingDetectedInRightY = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInLeftYTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_5B; }
			else { newState = STATE_4B; }
			confirmedBearingDetectedDistance = currentLongRangeLeftYDistance;
			isPossibleBallBeingDetectedInLeftY = false;
		}
		else
		{
			if (isBallDetectionBeingConfirmed) { newState = STATE_4B; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				newState = _circleLineChangeState(adjustment, false);
			}
		}
	}
	// Check for Right-X Detection
	else if ((currentLongRangeRightXDistance <= MAX_BALL_DETECTION_THRESHOLD)
		&& (currentLongRangeRightXDistance >= MIN_BALL_DETECTION_THRESHOLD)
		&& (ultrasonicLeftXDistance > ZBOT_DETECTION_DISTANCE))
	{
		if (isPossibleBallBeingDetectedInRightX == false)
		{
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInRightXTimer = millis();
			isPossibleBallBeingDetectedInRightX = true;
			// Clear other detection persistence timers
			//isPossibleBallBeingDetectedInLeftX = false;
			//isPossibleBallBeingDetectedInLeftY = false;
			//isPossibleBallBeingDetectedInRightY = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInRightXTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_5C; }
			else { newState = STATE_4C; }
			confirmedBearingDetectedDistance = currentLongRangeRightXDistance;
			isPossibleBallBeingDetectedInRightX = false;
		}
		else
		{
			if (isBallDetectionBeingConfirmed) { newState = STATE_4C; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				newState = _circleLineChangeState(adjustment, false);
			}
		}
	}
	// Check for Right-Y Detection
	else if ((currentLongRangeRightYDistance <= MAX_BALL_DETECTION_THRESHOLD)
		&& (currentLongRangeRightYDistance >= MIN_BALL_DETECTION_THRESHOLD)
		&& (ultrasonicLeftXDistance > ZBOT_DETECTION_DISTANCE))
	{
		if (isPossibleBallBeingDetectedInRightY == false)
		{
			// Possible detection, continue evaluating until initial persistence is met
			possibleBallDetectionInRightYTimer = millis();
			isPossibleBallBeingDetectedInRightY = true;
			// Clear other detection persistence timers
			//isPossibleBallBeingDetectedInLeftX = false;
			//isPossibleBallBeingDetectedInLeftY = false;
			//isPossibleBallBeingDetectedInRightX = false;
		}

		// Check if initial persistence has elapsed, otherwise continue line-following
		if (millis() - possibleBallDetectionInRightYTimer > detectionPersistence)
		{
			// Move to validation or confirmation state
			if (isBallDetectionBeingConfirmed) { newState = STATE_5D; }
			else { newState = STATE_4D; }
			confirmedBearingDetectedDistance = currentLongRangeRightYDistance;
			isPossibleBallBeingDetectedInRightY = false;
		}
		else
		{
			if (isBallDetectionBeingConfirmed) { newState = STATE_4D; }
			else
			{
				// Determine Line Adjustment
				CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
				newState = _circleLineChangeState(adjustment,false);
			}
		}
	}
	// No possible detections currently active, continue line following
	else
	{
		// Determine Line Adjustment
		CircleLineFollowingAdjustment adjustment = _adjustCircleLineFollowingMovement();
		newState = _circleLineChangeState(adjustment, false);
		// Clear all detection persistence timers
		isPossibleBallBeingDetectedInLeftX = false;
		isPossibleBallBeingDetectedInLeftY = false;
		isPossibleBallBeingDetectedInRightX = false;
		isPossibleBallBeingDetectedInRightY = false;
	}
	
	return newState;
}
