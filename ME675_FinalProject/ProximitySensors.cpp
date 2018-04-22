#include <math.h>
#include "ProximitySensors.h"
#include "Utilities.h"

void InitializeProximitySensors()
{
	pinMode(IR_OBJECT_DETECTION_SENSOR_PIN, INPUT);
	pinMode(US_ECHO_PIN, INPUT);
	pinMode(US_TRIGGER_PIN, OUTPUT);
	digitalWrite(US_TRIGGER_PIN, LOW);
}

int CalculateIRDistance(SharpSensorModel sensorType, DirectionOfIR direction)
{
	double voltage;
	int distance;

	switch (sensorType)
	{
	case SharpSensorModel::GP2Y0A51SK0F:
	{
		double analogCloseRangeIR = analogRead(IR_CLOSE_RANGE_SENSOR_PIN);
		voltage = ConvertAnalogInToVoltage(analogCloseRangeIR);
		distance = SHORT_RANGE_MULTIPLIER * pow(voltage, SHORT_RANGE_POWER);

		// If the measured distance is outside the measurement range, set equal to INVALID
		if (distance > SHORT_RANGE_MAX_DISTANCE) { distance = SHORT_RANGE_INVALID_DISTANCE; }
		break;
	}
	case SharpSensorModel::GP2Y0A60SZLF:
	{
		switch (direction)
		{
		case DirectionOfIR::X:
		{
			double analogLongRangeXIR = analogRead(IR_X_SENSOR_PIN);
			voltage = ConvertAnalogInToVoltage(analogLongRangeXIR);
			distance = LONG_RANGE_MULTIPLIER * pow(voltage, LONG_RANGE_POWER);
			break;
		}
		case DirectionOfIR::Y:
		{
			double analogLongRangeYIR = analogRead(IR_Y_SENSOR_PIN);
			voltage = ConvertAnalogInToVoltage(analogLongRangeYIR);
			distance = LONG_RANGE_MULTIPLIER * pow(voltage, LONG_RANGE_POWER);
			break;
		}
		}
	}
	}

	return distance;
}

int CalculateUltrasonicDistance()
{
	// Trigger the US Sensor for a reading
	digitalWrite(US_TRIGGER_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(US_TRIGGER_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(US_TRIGGER_PIN, LOW);

	// Measure the reflected signal for distance calculation
	long duration = pulseIn(US_ECHO_PIN, HIGH);
	int distance = duration * 0.034 / 2;
	return distance;
}

bool IsBearingHolsterPresent()
{
	bool isHolsterLifted = false;
	int detectionCounts = 0;
	double startTime = millis();

	while (millis() - startTime < OBJECT_DETECTION_SAMPLE_PERIOD)
	{
		int isObjectDetected = digitalRead(IR_OBJECT_DETECTION_SENSOR_PIN);
		if (isObjectDetected == LOW) { detectionCounts++; }
	}

	if (detectionCounts > HOLSTER_LIFTED_THRESHOLD) { isHolsterLifted = true; }
	return isHolsterLifted;
}
