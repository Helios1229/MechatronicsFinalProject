#include "Utilities.h"

double ConvertAnalogInToVoltage(double analogInput)
{
	double voltage = analogInput * INPUT_BITS_PER_VOLTAGE;
	return voltage;
}