#include <Arduino.h>
#include "PulleyControl.h"
#include "ProximitySensors.h"

void InitializePulleyMotorControl()
{
	pinMode(RAISE_PULLEY_PIN, OUTPUT);
	pinMode(LOWER_PULLEY_PIN, OUTPUT);
	pinMode(POWER_MAGNET_PIN, OUTPUT);

	digitalWrite(RAISE_PULLEY_PIN, LOW);
	digitalWrite(LOWER_PULLEY_PIN, LOW);
	digitalWrite(POWER_MAGNET_PIN, LOW);
}

void LowerPulley()
{
	unsigned long currentTime = millis();
	while (millis() - currentTime < 5700)
	{
		digitalWrite(LOWER_PULLEY_PIN, HIGH);
	}
	digitalWrite(LOWER_PULLEY_PIN, LOW);
}

void RaisePulley()
{
	bool isBearingHolsterPresent = false;
	while (isBearingHolsterPresent == false)
	{
		digitalWrite(RAISE_PULLEY_PIN, HIGH);
		isBearingHolsterPresent = IsBearingHolsterPresent();
	}
	digitalWrite(RAISE_PULLEY_PIN, LOW);
}

void PowerOnMagnet()
{
	digitalWrite(POWER_MAGNET_PIN, HIGH);
}

void PowerOffMagnet()
{
	digitalWrite(POWER_MAGNET_PIN, LOW);
}