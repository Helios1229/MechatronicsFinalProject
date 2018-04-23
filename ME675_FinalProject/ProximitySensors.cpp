#include <math.h>
#include "ProximitySensors.h"
#include "Utilities.h"

int irReadingsCloseRange[AVERAGE_READING_WINDOW];
int irReadingsLongRangeX[AVERAGE_READING_WINDOW];
int irReadingsLongRangeY[AVERAGE_READING_WINDOW];
int totalCloseRange = 0;
int totalLongRangeX = 0;
int totalLongRangeY = 0;
int currentReadingIndexCloseRange = 0;
int currentReadingIndexLongRangeX = 0;
int currentReadingIndexLongRangeY = 0;
int averageDistance = 0;

void InitializeProximitySensors()
{
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
	int distance = 0;

	switch (sensorType)
	{
		case SharpSensorModel::GP2Y0A51SK0F:
		{
			// Subtract previous reading
			totalCloseRange = totalCloseRange - irReadingsCloseRange[currentReadingIndexCloseRange];

			// Read current value from the sensor
			double analogCloseRangeIR = analogRead(IR_CLOSE_RANGE_SENSOR_PIN);
			distance = SHORT_RANGE_QUAD_COEFF * pow(analogCloseRangeIR, 2) + SHORT_RANGE_LINEAR_COEFF * analogCloseRangeIR + SHORT_RANGE_OFFSET;
			if ((distance > SHORT_RANGE_MAX_DISTANCE) || (distance < 0)) { distance = SHORT_RANGE_INVALID_DISTANCE; }
			irReadingsCloseRange[currentReadingIndexCloseRange] = distance;

			// Add the reading to the total
			totalCloseRange = totalCloseRange + irReadingsCloseRange[currentReadingIndexCloseRange];

			// Move index to the next read position
			currentReadingIndexCloseRange = currentReadingIndexCloseRange + 1;

			// Check if the end of the reading array has been reached
			if (currentReadingIndexCloseRange >= AVERAGE_READING_WINDOW) { currentReadingIndexCloseRange = 0; }

			// Return the filtered value
			averageDistance = totalCloseRange / AVERAGE_READING_WINDOW;
			break;
		}
		case SharpSensorModel::GP2Y0A60SZLF:
		{
			switch (direction)
			{
				case DirectionOfIR::X:
				{
					// Subtract previous reading
					totalLongRangeX = totalLongRangeX - irReadingsLongRangeX[currentReadingIndexLongRangeX];

					// Read current value from the sensor
					double analogLongRangeXIR = analogRead(IR_X_SENSOR_PIN);
					distance = LONG_RANGE_X_MULTIPLIER * pow(analogLongRangeXIR, LONG_RANGE_X_POWER);
					if ((distance > LONG_RANGE_MAX_DISTANCE) || (distance < 0)) { distance = LONG_RANGE_INVALID_DISTANCE; }
					irReadingsLongRangeX[currentReadingIndexLongRangeX] = distance;

					// Add the reading to the total
					totalLongRangeX = totalLongRangeX + irReadingsLongRangeX[currentReadingIndexLongRangeX];

					// Move index to next read position
					currentReadingIndexLongRangeX = currentReadingIndexLongRangeX + 1;

					// Check if the end of the reading array has been reached
					if (currentReadingIndexLongRangeX >= AVERAGE_READING_WINDOW) { currentReadingIndexLongRangeX = 0; }

					// Return the filtered value
					averageDistance = totalLongRangeX / AVERAGE_READING_WINDOW;
					break;
				}
				case DirectionOfIR::Y:
				{
					// Subtract previous reading
					totalLongRangeY = totalLongRangeY - irReadingsLongRangeY[currentReadingIndexLongRangeY];

					// Read current value from the sensor
					double analogLongRangeYIR = analogRead(IR_Y_SENSOR_PIN);
					distance = LONG_RANGE_Y_MULTIPLIER * pow(analogLongRangeYIR, LONG_RANGE_Y_POWER);
					if ((distance > LONG_RANGE_MAX_DISTANCE) || (distance < 0)) { distance = LONG_RANGE_INVALID_DISTANCE; }
					irReadingsLongRangeY[currentReadingIndexLongRangeY] = distance;

					// Add the reading to the total
					totalLongRangeY = totalLongRangeY + irReadingsLongRangeY[currentReadingIndexLongRangeY];

					// Move index to next read position
					currentReadingIndexLongRangeY = currentReadingIndexLongRangeY + 1;

					// Check if the end of the reading array has been reached
					if (currentReadingIndexLongRangeY >= AVERAGE_READING_WINDOW) { currentReadingIndexLongRangeY = 0; }

					// Return the filtered value
					averageDistance = totalLongRangeY / AVERAGE_READING_WINDOW;
					break;
				}
			}
			break;
		}
	}

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
