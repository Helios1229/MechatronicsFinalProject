#include "ME675_FinalProject.h"
#include "StepperControl.h"
#include <AMIS30543.h>
#include <SPI.h>
#include <AccelStepper.h>


static AMIS30543::stepMode selectedRightStepperStepSize = DEFAULT_STEP_SIZE;
static AMIS30543::stepMode selectedLeftStepperStepSize = DEFAULT_STEP_SIZE;
AMIS30543 rightStepperMotor;
AMIS30543 leftStepperMotor;

void StepperInitialization()
{
	// Initialize stepper drivers
	SPI.begin();
	rightStepperMotor.init(RIGHT_STEPPER_SLAVE_SELECT_PIN);
	leftStepperMotor.init(LEFT_STEPPER_SLAVE_SELECT_PIN);

	// Drive the NXT/STEP and DIR pins low initially.
	digitalWrite(RIGHT_STEPPER_STEP_PIN, LOW);
	pinMode(RIGHT_STEPPER_STEP_PIN, OUTPUT);
	digitalWrite(RIGHT_STEPPER_DIR_PIN, LOW);
	pinMode(RIGHT_STEPPER_DIR_PIN, OUTPUT);

	digitalWrite(LEFT_STEPPER_STEP_PIN, LOW);
	pinMode(LEFT_STEPPER_STEP_PIN, OUTPUT);
	digitalWrite(LEFT_STEPPER_DIR_PIN, LOW);
	pinMode(LEFT_STEPPER_DIR_PIN, OUTPUT);

	// Give the drivers some time to power up.
	delay(1);

	// Reset the drivers to their default settings.
	rightStepperMotor.resetSettings();
	leftStepperMotor.resetSettings();

	// Set the stepper current limit. 
	rightStepperMotor.setCurrentMilliamps(DEFAULT_STEPPER_CURRENT);
	leftStepperMotor.setCurrentMilliamps(DEFAULT_STEPPER_CURRENT);

	// Set the number of microsteps that correspond to one full step.
	rightStepperMotor.setStepMode(DEFAULT_STEP_SIZE);
	leftStepperMotor.setStepMode(DEFAULT_STEP_SIZE);

	// Enable the motor outputs.
	rightStepperMotor.enableDriver();
	leftStepperMotor.enableDriver(); 
}

void LineFollowingMoveStraight()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void LineFollowingMoveCW()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep4);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void LineFollowingMoveRight()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep16);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void LineFollowingMoveFarRight()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep32);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void LineFollowingMoveLeft()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep16);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void LineFollowingMoveFarLeft()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep8);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep32);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void SlowMovement()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 250);
	analogWrite(LEFT_STEPPER_STEP_PIN, 250);
	// Step the right motor
	_setStepping(RIGHT_STEPPER_STEP_PIN, AMIS30543::MicroStep16);
	_setDirection(RIGHT_STEPPER_DIR_PIN, RIGHT_STEPPER_FORWARD_DIRECTION);
	//_step(RIGHT_STEPPER_STEP_PIN);
	delay(1);

	// Step the left motor
	_setStepping(LEFT_STEPPER_STEP_PIN, AMIS30543::MicroStep16);
	_setDirection(LEFT_STEPPER_DIR_PIN, LEFT_STEPPER_FORWARD_DIRECTION);
	//_step(LEFT_STEPPER_STEP_PIN);
	delay(1);
}

void StopMovement()
{
	analogWrite(RIGHT_STEPPER_STEP_PIN, 0);
	analogWrite(LEFT_STEPPER_STEP_PIN, 0);
}

// Sends a pulse on the NXT/STEP pin to tell the driver to take
// one step, and also delays to control the speed of the motor.
void _step(uint8_t stepPin)
{
	// The NXT/STEP minimum high pulse width is 2 microseconds.
	digitalWrite(stepPin, HIGH);
	delayMicroseconds(STEP_PULSE_WIDTH);
	digitalWrite(stepPin, LOW);
	delayMicroseconds(STEP_PULSE_WIDTH);

	// The delay here controls the stepper motor's speed.  You can
	// increase the delay to make the stepper motor go slower.  If
	// you decrease the delay, the stepper motor will go fast, but
	// there is a limit to how fast it can go before it starts
	// missing steps.
	delayMicroseconds(POST_STEP_DELAY_MICROSEC);
}

// Writes a high or low value to the direction pin to specify
// what direction to turn the motor.
void _setDirection(uint8_t dirPin, bool dir)
{
	// The NXT/STEP pin must not change for at least 0.5
	// microseconds before and after changing the DIR pin.
	delayMicroseconds(DIRECTION_PULSE_WIDTH);
	digitalWrite(dirPin, dir);
	delayMicroseconds(DIRECTION_PULSE_WIDTH);
}

void _setStepping(uint8_t stepPin, AMIS30543::stepMode stepSize)
{
	switch (stepPin)
	{
	case RIGHT_STEPPER_STEP_PIN:
		if (stepSize != selectedRightStepperStepSize)
		{
			rightStepperMotor.setStepMode(stepSize);
			selectedRightStepperStepSize = stepSize;
		}
		break;

	case LEFT_STEPPER_STEP_PIN:
		if (stepSize != selectedLeftStepperStepSize)
		{
			leftStepperMotor.setStepMode(stepSize);
			selectedLeftStepperStepSize = stepSize;
		}
		break;

	default:
		// Error - Invalid Pin
		break;
	}
}