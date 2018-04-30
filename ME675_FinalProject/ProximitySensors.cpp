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

void InitializeProximitySensors()
{
	pinMode(US_X_ECHO_PIN, INPUT);
	pinMode(US_X_TRIGGER_PIN, OUTPUT);
	pinMode(US_Y_ECHO_PIN, INPUT);
	pinMode(US_Y_TRIGGER_PIN, OUTPUT);

	pinMode(DIGITAL_IR_LEFT3_PIN, INPUT);
	pinMode(DIGITAL_IR_LEFT2_PIN, INPUT);
	pinMode(DIGITAL_IR_LEFT1_PIN, INPUT);
	pinMode(DIGITAL_IR_MIDDLE_PIN, INPUT);
	pinMode(DIGITAL_IR_RIGHT1_PIN, INPUT);
	pinMode(DIGITAL_IR_RIGHT2_PIN, INPUT);
	pinMode(DIGITAL_IR_RIGHT3_PIN, INPUT);

	digitalWrite(US_X_TRIGGER_PIN, LOW);
	digitalWrite(US_Y_TRIGGER_PIN, LOW);

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
	int averageDistance = 0;
	double analogLongRangeIR = 0;
	switch (direction)
	{
		case (DirectionOfLongIR::IR_X_LEFT):
		{
			// Subtract previous reading
			totalLongRangeLeftX = totalLongRangeLeftX - irReadingsLongRangeLeftX[currentIndexLongRangeLeftX];

			// Read current value from the sensor
			analogLongRangeIR = analogRead(LONG_RANGE_IR_X_LEFT_SENSOR_PIN);
			distance = LONG_RANGE_X_LEFT_MULTIPLIER * pow(analogLongRangeIR, LONG_RANGE_X_LEFT_POWER);

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
			analogLongRangeIR = analogRead(LONG_RANGE_IR_Y_LEFT_SENSOR_PIN);
			distance = LONG_RANGE_Y_LEFT_MULTIPLIER * pow(analogLongRangeIR, LONG_RANGE_Y_LEFT_POWER);

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
		case (DirectionOfLongIR::IR_X_RIGHT):
		{
			// Subtract previous reading
			totalLongRangeRightX = totalLongRangeRightX - irReadingsLongRangeRightX[currentIndexLongRangeRightX];

			// Read current value from the sensor
			analogLongRangeIR = analogRead(LONG_RANGE_IR_X_RIGHT_SENSOR_PIN);
			distance = LONG_RANGE_X_RIGHT_MULTIPLIER * pow(analogLongRangeIR, LONG_RANGE_X_RIGHT_POWER);

			// Validity Checks
			if (distance > LONG_RANGE_MAX_DISTANCE) { distance = LONG_RANGE_MAX_DISTANCE; }
			else if (distance < LONG_RANGE_MIN_DISTANCE) { distance = LONG_RANGE_INVALID_DISTANCE; }
			irReadingsLongRangeRightX[currentIndexLongRangeRightX] = distance;

			// Add the reading to the total
			totalLongRangeRightX = totalLongRangeRightX + irReadingsLongRangeRightX[currentIndexLongRangeRightX];

			// Move index to next read position
			currentIndexLongRangeRightX = currentIndexLongRangeRightX + 1;

			// Check if the end of the reading array has been reached
			if (currentIndexLongRangeRightX >= AVERAGE_READING_WINDOW) { currentIndexLongRangeRightX = 0; }

			// Return the filtered value
			averageDistance = totalLongRangeRightX / AVERAGE_READING_WINDOW;
			break;
		}
		case (DirectionOfLongIR::IR_Y_RIGHT):
		{
			// Subtract previous reading
			totalLongRangeRightY = totalLongRangeRightY - irReadingsLongRangeRightY[currentIndexLongRangeRightY];

			// Read current value from the sensor
			analogLongRangeIR = analogRead(LONG_RANGE_IR_Y_RIGHT_SENSOR_PIN);
			distance = LONG_RANGE_Y_RIGHT_MULTIPLIER * pow(analogLongRangeIR, LONG_RANGE_Y_RIGHT_POWER);

			// Validity Checks
			if (distance > LONG_RANGE_MAX_DISTANCE) { distance = LONG_RANGE_MAX_DISTANCE; }
			else if (distance < LONG_RANGE_MIN_DISTANCE) { distance = LONG_RANGE_INVALID_DISTANCE; }
			irReadingsLongRangeRightY[currentIndexLongRangeRightY] = distance;

			// Add the reading to the total
			totalLongRangeRightY = totalLongRangeRightY + irReadingsLongRangeRightY[currentIndexLongRangeRightY];

			// Move index to next read position
			currentIndexLongRangeRightY = currentIndexLongRangeRightY + 1;

			// Check if the end of the reading array has been reached
			if (currentIndexLongRangeRightY >= AVERAGE_READING_WINDOW) { currentIndexLongRangeRightY = 0; }

			// Return the filtered value
			averageDistance = totalLongRangeRightY / AVERAGE_READING_WINDOW;
			break;
		}
	}

	return averageDistance;
}

int CalculateShortIRDistance()
{
	int distance = 0;
	int averageDistance = 0;

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

DigitalCloseRangeArray CalculateDigitalIRarray()
{
	int count = 0;
	int digitalLeft3 = 0;
	int digitalLeft2 = 0;
	int digitalLeft1 = 0;
	int digitalMiddle = 0;
	int digitalRight1 = 0;
	int digitalRight2 = 0;
	int digitalRight3 = 0;

	DigitalCloseRangeArray digitalIRarray;
	unsigned long currentTime = millis();
	while (millis() - currentTime < 25)
	{
		if (digitalRead(DIGITAL_IR_LEFT3_PIN)==LOW) { digitalLeft3 = digitalLeft3 + 1; }
		if (digitalRead(DIGITAL_IR_LEFT2_PIN)==LOW) { digitalLeft2 = digitalLeft2 + 1; }
		if (digitalRead(DIGITAL_IR_LEFT1_PIN)==LOW) { digitalLeft1 = digitalLeft1 + 1; }
		if (digitalRead(DIGITAL_IR_MIDDLE_PIN)==LOW) { digitalMiddle = digitalMiddle + 1; }
		if (digitalRead(DIGITAL_IR_RIGHT1_PIN)==LOW) { digitalRight1 = digitalRight1 + 1; }
		if (digitalRead(DIGITAL_IR_RIGHT2_PIN)==LOW) { digitalRight2 = digitalRight2 + 1; }
		if (digitalRead(DIGITAL_IR_RIGHT3_PIN)==LOW) { digitalRight3 = digitalRight3 + 1; }
		count = count + 1;
	}

	if ((float)digitalLeft3 / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRleft3Detected = true; }
	else { digitalIRarray.isIRleft3Detected = false; }
	if ((float)digitalLeft2 / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRleft2Detected = true; }
	else { digitalIRarray.isIRleft2Detected = false; }
	if ((float)digitalLeft1 / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRleft1Detected = true; }
	else { digitalIRarray.isIRleft1Detected = false; }
	if ((float)digitalMiddle / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRmiddleDetected = true; }
	else { digitalIRarray.isIRmiddleDetected = false; }
	if ((float)digitalRight1 / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRright1Detected = true; }
	else { digitalIRarray.isIRright1Detected = false; }
	if ((float)digitalRight2 / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRright2Detected = true; }
	else { digitalIRarray.isIRright2Detected = false; }
	if ((float)digitalRight3 / (float)count > PERCENT_DIGITAL_VALID_READS) { digitalIRarray.isIRright3Detected = true; }
	else { digitalIRarray.isIRright3Detected = false; }

	return digitalIRarray;
}

int CalculateUltrasonicDistance(DirectionOfUS direction)
{
	int distance = 0;
	long duration = 0;
	switch (direction)
	{
		case (DirectionOfUS::US_X):
		{
			digitalWrite(US_X_TRIGGER_PIN, LOW);
			delayMicroseconds(2);
			digitalWrite(US_X_TRIGGER_PIN, HIGH);
			delayMicroseconds(10);
			digitalWrite(US_X_TRIGGER_PIN, LOW);

			// Measure the reflected signal for distance calculation
			duration = pulseIn(US_X_ECHO_PIN, HIGH);
			distance = duration * 0.034 / 2;
			break;
		}
		case (DirectionOfUS::US_Y):
		{
			digitalWrite(US_Y_TRIGGER_PIN, LOW);
			delayMicroseconds(2);
			digitalWrite(US_Y_TRIGGER_PIN, HIGH);
			delayMicroseconds(10);
			digitalWrite(US_Y_TRIGGER_PIN, LOW);

			// Measure the reflected signal for distance calculation
			duration = pulseIn(US_Y_ECHO_PIN, HIGH);
			distance = duration * 0.034 / 2;
			break;
		}
	}

	// Trigger the US Sensor for a reading
	return distance;
}
