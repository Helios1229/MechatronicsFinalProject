#include <math.h>

double CalculateDistanceFromAnalogInput(double analogIR)
{
	double voltage = (analogIR / 1023)*5.0;
	double distance = 54.879*pow(voltage, -1.378);
	return distance;
}