#include <Arduino.h>
#include "PulleyControl.h"
#include "ProximitySensors.h"

volatile bool raisePulley = false;

void InitializePulleyMotorControl()
{
	pinMode(RAISE_PULLEY_PIN, OUTPUT);
	pinMode(LOWER_PULLEY_PIN, OUTPUT);
	pinMode(POWER_MAGNET_PIN, OUTPUT);
	pinMode(LEFT_SIDE_LIMIT_SWITCH_PIN, INPUT);
	pinMode(RIGHT_SIDE_LIMIT_SWITCH_PIN, INPUT);

	attachInterrupt(digitalPinToInterrupt(LEFT_SIDE_LIMIT_SWITCH_PIN), StopRaisingPulley, HIGH);
	attachInterrupt(digitalPinToInterrupt(RIGHT_SIDE_LIMIT_SWITCH_PIN), StopRaisingPulley, HIGH);

	digitalWrite(RAISE_PULLEY_PIN, LOW);
	digitalWrite(LOWER_PULLEY_PIN, LOW);
	digitalWrite(POWER_MAGNET_PIN, LOW);
}

void LowerPulley()
{
	unsigned long currentTime = millis();
	while (millis() - currentTime < PULLEY_LOWER_PERIOD)
	{
		digitalWrite(LOWER_PULLEY_PIN, HIGH);
	}
	digitalWrite(LOWER_PULLEY_PIN, LOW);
}

void RaisePulley()
{
	raisePulley = true;
	while (raisePulley)
	{
		digitalWrite(RAISE_PULLEY_PIN, HIGH);
	}
	digitalWrite(RAISE_PULLEY_PIN, LOW);
}

void StopRaisingPulley()
{
	raisePulley = false;
}

void PowerOnMagnet()
{
	digitalWrite(POWER_MAGNET_PIN, HIGH);
}

void PowerOffMagnet()
{
	digitalWrite(POWER_MAGNET_PIN, LOW);
}