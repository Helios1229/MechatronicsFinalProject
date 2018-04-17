#pragma once
#include <stdint.h>

const unsigned int BLACK_LINE_THRESHOLD = 170;

struct LineDetectionStructure
{
	bool Sensor7LineDetected;
	bool Sensor6LineDetected;
	bool Sensor5LineDetected;
	bool Sensor4LineDetected;
	bool Sensor3LineDetected;
	bool Sensor2LineDetected;
	bool Sensor1LineDetected;
	bool Sensor0LineDetected;
};

void LineFollowerInitialization();
LineDetectionStructure ProcessLineFollowerInput();
bool _didSensorDetectLine(unsigned char detectionLevel);