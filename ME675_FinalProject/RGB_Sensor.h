#pragma once

const int RED_LINE_THRESHOLD = 75;

struct RGBreadingStructure
{
	float Red;
	float Green;
	float Blue;
};

void RGBsensorInitialization();
RGBreadingStructure RGBreadColor();
bool IsRedLinePresent();