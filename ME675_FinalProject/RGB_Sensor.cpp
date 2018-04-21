#include <wire.h>
#include <Adafruit_TCS34725.h>
#include "RGB_Sensor.h"

uint16_t lux;
uint8_t r, g, b, c;
RGBreadingStructure rgbReadData;
Adafruit_TCS34725 rgbSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

void RGBsensorInitialization()
{
	rgbSensor.begin();
}

RGBreadingStructure RGBreadColor()
{
	lux = rgbSensor.getRGB(&r, &g, &b);
	rgbReadData = { *(&r), *(&g), *(&b)};
	return rgbReadData;
}

bool IsRedLinePresent()
{
	bool isRedLinePresent = false;

	lux = rgbSensor.getRGB(&r, &g, &b);
	rgbReadData = { *(&r), *(&g), *(&b) };
	if (rgbReadData.Red >= RED_LINE_THRESHOLD)
	{
		isRedLinePresent = true;
	}
	return isRedLinePresent;
}