#pragma once
#include <stdint.h>

const int LINE_LOST_SLOW_TIMEOUT = 650;				// Timeout period in milliseconds to start slow movement
const int LINE_LOST_STOP_TIMEOUT = 2500;			// Timeout period in milliseconds to stop movement

const int MIN_BALL_DETECTION_THRESHOLD = 15;		// Minimum distance to advance towards a detected ball bearing
const int MAX_BALL_DETECTION_THRESHOLD = 35;		// Maximum ball-bearing detection threshold

const int SHORT_RANGE_DETECT_THRESHOLD = 4;			// Minimum valid distance to register a ball bearing at the short range IR

const int INITIAL_DETECTION_PERSISTENCE = 100;		// Initial period in milliseconds to detect a ball bearing
const int CONFIRM_DETECTION_PERSISTENCE = 1000;	// Validation period in milliseconds to confirm a ball bearing

const int MAGNET_POWER_ON_DELAY = 500;				// Period in milliseconds to power on magnet and lift ball

enum StartLineFollowingAdjustment { START_LEFT, START_STRAIGHT, START_RIGHT, START_LOST, ROTATE };
enum CircleLineFollowingAdjustment { FAR_LEFT, LEFT, NORMAL_CW, RIGHT, FAR_RIGHT, LOST };

void _finiteStateMachineProcess();
StartLineFollowingAdjustment _adjustStartLineFollowingMovement();
int _startLineChangeState(StartLineFollowingAdjustment adjustment);
CircleLineFollowingAdjustment _adjustCircleLineFollowingMovement();
int _circleLineChangeState(CircleLineFollowingAdjustment adjustment);
void _readIRdetectiondistance();
int _BallDetectionStateChange(bool isBallDetectionBeingConfirmed);
