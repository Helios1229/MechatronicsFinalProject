#pragma once
#include <stdint.h>
#include <AMIS30543.h>

const uint8_t RIGHT_STEPPER_DIR_PIN = 2;
const uint8_t RIGHT_STEPPER_STEP_PIN = 3;
const uint8_t RIGHT_STEPPER_SLAVE_SELECT_PIN = 4;
const uint8_t LEFT_STEPPER_DIR_PIN = 5;
const uint8_t LEFT_STEPPER_STEP_PIN = 6;
const uint8_t LEFT_STEPPER_SLAVE_SELECT_PIN = 7;

const int DEFAULT_STEPPER_CURRENT = 1700;								// Default Current Limit [mA]
const int PWM_DUTY_CYCLE_DEFAULT = 250;									// Default Duty Cycle to provide to stepping pin
const AMIS30543::stepMode DEFAULT_STEP_SIZE = AMIS30543::MicroStep8;	// Microsteps corresponding to one full step
const int STEP_PULSE_WIDTH = 3;											// Number of microseconds to hold each step pulse
const int DIRECTION_PULSE_WIDTH = 1;									// Number of microseconds to hold each direction pulse
const int POST_STEP_DELAY_MICROSEC = 500;								// Number of microseconds to delay after stepping
const int START_MOVEMENT_PERIOD = 2000;									// Number of milliseconds to move to starting line
const bool RIGHT_STEPPER_FORWARD_DIRECTION = 0;							// Boolean value corresponding to right stepper forward movement
const bool LEFT_STEPPER_FORWARD_DIRECTION = 1;							// Boolean value corresponding to left stepper forward movement

void StepperInitialization();
void MoveToStartLine();
void StartLineFollowingMoveStraight();
void StartLineFollowingSlowMovement();
void LineFollowingMoveCW();
void LineFollowingMoveRight();
void LineFollowingMoveFarRight();
void LineFollowingMoveLeft();
void LineFollowingMoveFarLeft();
void LineFollowingSlowMovement();
void BallLocateVerySlowMovement();
void AdjustPositionIntoCloseRange();
void StopMovement();
void Rotate90CW();
void Rotate90CCW();
void RotateSlowCW();
void _setDirection(uint8_t dirPin, bool dir);
void _setStepping(uint8_t stepPin, AMIS30543::stepMode stepSize);
void _setPWM(int pwmDutyCycle);

