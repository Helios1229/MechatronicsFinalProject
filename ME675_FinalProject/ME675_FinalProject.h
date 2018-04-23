#pragma once
#include <stdint.h>

const int LINE_LOST_SLOW_TIMEOUT = 500;			// Timeout period in milliseconds to start slow movement
const int LINE_LOST_STOP_TIMEOUT = 2500;		// Timeout period in milliseconds to stop movement
const int MAX_BALL_DETECTION_THRESHOLD = 30;	// Maximum distance to register a detected ball bearing
const int MIN_BALL_DETECTION_THRESHOLD = 11;	// Minimum distance to advance towards a detected ball bearing
const int MAX_BALL_CLOSE_RANGE_THRESHOLD = 15;	// Maximum distance to register a ball bearing with close range IR
const int MIN_BALL_CLOSE_RANGE_THRESHOLD = 6;	// Minimum distance to start ball bearing pick-up process
const int MAGNET_POWER_ON_DELAY = 500;			// Period in milliseconds to power on magnet and lift ball
const int DETECTION_PERSISTENCE = 50000;			// Period in milliseconds to register a detected bearing

enum StartLineFollowingAdjustment { START_LEFT, START_STRAIGHT, START_RIGHT, START_LOST, ROTATE };
enum CircleLineFollowingAdjustment { FAR_LEFT, LEFT, NORMAL_CW, RIGHT, FAR_RIGHT, LOST };

void _finiteStateMachineProcess();
StartLineFollowingAdjustment _adjustStartLineFollowingMovement();
int _startLineChangeState(StartLineFollowingAdjustment adjustment);
CircleLineFollowingAdjustment _adjustCircleLineFollowingMovement();
int _circleLineChangeState(CircleLineFollowingAdjustment adjustment);
int _ballDetectionOrLineAdjustChangeState(int xDistanceDetection, int yDistanceDetection);