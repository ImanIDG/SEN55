#include "MH411D.h"

MH411D::MH411D()
{
}

void MH411D::begin(Stream * serial, char scale)
{
	_serial = serial;
	_scale = scale;
}

char MH411D::getScale()
{
	return _scale;
}

void MH411D::setScale(char scale)
{
	#if DEBUG > 0
		Serial.print("Set sensor scale to: ");
		Serial.println(scale);
	#endif
	_scale = scale;
}

int MH411D::computeGasConcentration(uint8_t byte2, uint8_t byte3)
{
    //Same computation process as mh411D

    int result = WinsenSensors::computeGasConcentration(byte2, byte3);
	float pdata = result/100.0;

	#if DEBUG > 0
		Serial.print("result: ");
		Serial.println(result);
		Serial.print("percent: ");
		Serial.print(pdata, 2);
		Serial.println("%");
		Serial.print("ppm: ");
		Serial.println(result);

	#endif
    //proceed decimal value if needed
    return result ;

}