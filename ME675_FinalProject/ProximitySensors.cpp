#include <math.h>
#include "ProximitySensors.h"
#include "Utilities.h"

int irReadingsLongRangeLeftX[AVERAGE_READING_WINDOW];
int irReadingsLongRangeLeftY[AVERAGE_READING_WINDOW];
int irReadingsLongRangeRightX[AVERAGE_READING_WINDOW];
int irReadingsLongRangeRightY[AVERAGE_READING_WINDOW];
int irReadingsShortRangeLeftX[AVERAGE_READING_WINDOW];

int totalLongRangeLeftX = 0;
int totalLongRangeLeftY = 0;
int totalLongRangeRightX = 0;
int totalLongRangeRightY = 0;
int totalCloseRangeLeftX = 0;

int currentIndexLongRangeLeftX = 0;
int currentIndexLongRangeLeftY = 0;
int currentIndexLongRangeRightX = 0;
int currentIndexLongRangeRightY = 0;
int currentIndexShortRangeLeftX = 0;

int averageDistance = 0;

void InitializeProximitySensors()
{
	pinMode(US_ECHO_PIN, INPUT);
	pinMode(US_TRIGGER_PIN, OUTPUT);

	digitalWrite(US_TRIGGER_PIN, LOW);

	// Initialize all running averages to 0
	for (int index = 0; index < AVERAGE_READING_WINDOW; index++)
	{
		irReadingsLongRangeLeftX[index] = 0;
		irReadingsLongRangeLeftY[index] = 0;
		irReadingsLongRangeRightX[index] = 0;
		irReadingsLongRangeRightY[index] = 0;
		irReadingsShortRangeLeftX[index] = 0;
	}
}

int CalculateLongIRDistance(DirectionOfLongIR direction)
{
	int distance = 0;
	switch (direction)
	{
		case (DirectionOfLongIR::IR_X_LEFT):
		{
			// Subtract previous reading
			totalLongRangeLeftX = totalLongRangeLeftX - irReadingsLongRangeLeftX[currentIndexLongRangeLeftX];

			// Read current value from the sensor
			double analogLongRangeXIR = analogRead(LONG_RANGE_IR_X_LEFT_SENSOR_PIN);
			distance = LONG_RANGE_X_LEFT_MULTIPLIER * pow(analogLongRangeXIR, LONG_RANGE_X_LEFT_POWER);

			// Validity Checks
			if (distance > LONG_RANGE_MAX_DISTANCE) { distance = LONG_RANGE_MAX_DISTANCE; }
			else if(distance < LONG_RANGE_MIN_DISTANCE) { distance = LONG_RANGE_INVALID_DISTANCE; }
			irReadingsLongRangeLeftX[currentIndexLongRangeLeftX] = distance;

			// Add the reading to the total
			totalLongRangeLeftX = totalLongRangeLeftX + irReadingsLongRangeLeftX[currentIndexLongRangeLeftX];

			// Move index to next read position
			currentIndexLongRangeLeftX = currentIndexLongRangeLeftX + 1;

			// Check if the end of the reading array has been reached
			if (currentIndexLongRangeLeftX >= AVERAGE_READING_WINDOW) { currentIndexLongRangeLeftX = 0; }

			// Return the filtered value
			averageDistance = totalLongRangeLeftX / AVERAGE_READING_WINDOW;
			break;
		}
		case DirectionOfLongIR::IR_Y_LEFT:
		{
			// Subtract previous reading
			totalLongRangeLeftY = totalLongRangeLeftY - irReadingsLongRangeLeftY[currentIndexLongRangeLeftY];

			// Read current value from the sensor
			double analogLongRangeYIR = analogRead(LONG_RANGE_IR_Y_LEFT_SENSOR_PIN);
			distance = LONG_RANGE_Y_LEFT_MULTIPLIER * pow(analogLongRangeYIR, LONG_RANGE_Y_LEFT_POWER);

			// Validity Checks
			if (distance > LONG_RANGE_MAX_DISTANCE) { distance = LONG_RANGE_MAX_DISTANCE; }
			else if (distance < LONG_RANGE_MIN_DISTANCE) { distance = LONG_RANGE_INVALID_DISTANCE; }
			irReadingsLongRangeLeftY[currentIndexLongRangeLeftY] = distance;

			// Add the reading to the total
			totalLongRangeLeftY = totalLongRangeLeftY + irReadingsLongRangeLeftY[currentIndexLongRangeLeftY];

			// Move index to next read position
			currentIndexLongRangeLeftY = currentIndexLongRangeLeftY + 1;

			// Check if the end of the reading array has been reached
			if (currentIndexLongRangeLeftY >= AVERAGE_READING_WINDOW) { currentIndexLongRangeLeftY = 0; }

			// Return the filtered value
			averageDistance = totalLongRangeLeftY / AVERAGE_READING_WINDOW;
			break;
		}
	}

	return averageDistance;
}

int CalculateShortIRDistance()
{
	int distance = 0;

	// Subtract previous reading
	totalCloseRangeLeftX = totalCloseRangeLeftX - irReadingsShortRangeLeftX[currentIndexShortRangeLeftX];

	// Read current value from the sensor
	double analogCloseRangeIR = analogRead(SHORT_RANGE_IR_X_LEFT_SENSOR_PIN);
	distance = SHORT_RANGE_LEFT_MULTIPLIER * pow(analogCloseRangeIR, SHORT_RANGE_LEFT_POWER);
	if ((distance > SHORT_RANGE_MAX_DISTANCE) || (distance < 0)) { distance = SHORT_RANGE_INVALID_DISTANCE; }
	irReadingsShortRangeLeftX[currentIndexShortRangeLeftX] = distance;

	// Add the reading to the total
	totalCloseRangeLeftX = totalCloseRangeLeftX + irReadingsShortRangeLeftX[currentIndexShortRangeLeftX];

	// Move index to the next read position
	currentIndexShortRangeLeftX = currentIndexShortRangeLeftX + 1;

	// Check if the end of the reading array has been reached
	if (currentIndexShortRangeLeftX >= AVERAGE_READING_WINDOW) { currentIndexShortRangeLeftX = 0; }

	// Return the filtered value
	averageDistance = totalCloseRangeLeftX / AVERAGE_READING_WINDOW;

	return averageDistance;
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
