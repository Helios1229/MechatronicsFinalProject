#include <math.h>
#include "ProximitySensors.h"
#include "Utilities.h"

int irReadingsCloseRange[AVERAGE_READING_WINDOW];
int irReadingsLongRangeX[AVERAGE_READING_WINDOW];
int irReadingsLongRangeY[AVERAGE_READING_WINDOW];
int totalCloseRange = 0;
int totalLongRangeX = 0;
int totalLongRangeY = 0;
int averageCloseRange = 0;
int averageLongRangeX = 0;
int averageLongRangeY = 0;
int currentReadingIndexCloseRange = 0;
int currentReadingIndexLongRangeX = 0;
int currentReadingIndexLongRangeY = 0;

void InitializeProximitySensors()
{
	pinMode(IR_OBJECT_DETECTION_SENSOR_PIN, INPUT);
	pinMode(US_ECHO_PIN, INPUT);
	pinMode(US_TRIGGER_PIN, OUTPUT);
	digitalWrite(US_TRIGGER_PIN, LOW);

	// Initialize all running averages to 0
	for (int index = 0; index < AVERAGE_READING_WINDOW; index++)
	{
		irReadingsCloseRange[index] = 0;
		irReadingsLongRangeX[index] = 0;
		irReadingsLongRangeY[index] = 0;
	}
}

int CalculateIRDistance(SharpSensorModel sensorType, DirectionOfIR direction)
{
	double voltage;
	int distance;

	switch (sensorType)
	{
		case SharpSensorModel::GP2Y0A51SK0F:
		{
			// Subtract previous reading
			totalCloseRange = totalCloseRange - irReadingsCloseRange[currentReadingIndexCloseRange];

			// Read current value from the sensor
			double analogCloseRangeIR = analogRead(IR_CLOSE_RANGE_SENSOR_PIN);
			voltage = ConvertAnalogInToVoltage(analogCloseRangeIR);
			distance = SHORT_RANGE_MULTIPLIER * pow(voltage, SHORT_RANGE_POWER);
			if (distance > SHORT_RANGE_MAX_DISTANCE) { distance = SHORT_RANGE_INVALID_DISTANCE; }
			irReadingsCloseRange[currentReadingIndexCloseRange] = distance;

			// Add the reading to the total
			totalCloseRange = totalCloseRange + irReadingsCloseRange[currentReadingIndexCloseRange];

			// Move to the next read position
			currentReadingIndexCloseRange = currentReadingIndexCloseRange + 1;

			// Check if the end of the reading array has been reached
			if (currentReadingIndexCloseRange >= AVERAGE_READING_WINDOW) { currentReadingIndexCloseRange = 0; }

			// Return the filtered value
			averageCloseRange = totalCloseRange / AVERAGE_READING_WINDOW;

			return averageCloseRange;
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

	while (millis() - startTime < HOLSTER_DETECTION_SAMPLE_PERIOD)
	{
		int isObjectDetected = digitalRead(IR_OBJECT_DETECTION_SENSOR_PIN);
		if (isObjectDetected == LOW) { detectionCounts++; }
	}

	if (detectionCounts > HOLSTER_LIFTED_THRESHOLD) { isHolsterLifted = true; }
	return isHolsterLifted;
}
