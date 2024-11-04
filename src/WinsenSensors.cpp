/**
 * A nice library that handle MH410D/MH411D sensors from winsen sensors
 *
 * name     : MH411D
 * Author   : Guillaume CHARDIN
 * Version  : 0.0.1
 * Date     : 13/02/2024
 * url      : https://github.com/guillaumechardin/MH411D
 */
 
#include "WinsenSensors.h"
 

WinsenSensors::WinsenSensors()
{
}


void WinsenSensors::begin(Stream * serial)
{
  _serial = serial;
}

int WinsenSensors::getGasConcentration()
{
	return _gasConcentration;
}

int WinsenSensors::startMeasure()
{
	#if DEBUG > 0
		Serial.println("Request sensor on CO2 concentration");
	#endif
	
	//request co2 concentration value  from sensor command 0x86
	uint8_t request[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00};
    request[8] = getChecksum(request);
	//ask sensor to get co2 data
	sendSensorData(request, 9);	
	
	// process sensor reply
	return getSensorData();
}

uint8_t WinsenSensors::getChecksum(uint8_t *data)
{
	#if DEBUG > 0
	Serial.println("Computing cheksum");
	for (int i=0;i<8;i++)
		{
			Serial.print(data[i],HEX);
			Serial.print(",");
		}
	#endif

	uint8_t  checksum;
	for( int i = 1; i < 8; i++)
	{
		checksum += data[i];
	}
	
	checksum = 0xff - checksum;
	checksum += 1;
	#if DEBUG > 0
		Serial.print("Checksum value:");
		Serial.println(checksum,HEX);
	#endif
	return checksum;
}

void WinsenSensors::sendSensorData(uint8_t *data, uint8_t size)
{
	#if DEBUG > 0
		Serial.print("Send data on Serial:");
		for (int i=0; i<=8 ;i++)
		{
			Serial.print(data[i],HEX);
			Serial.print(",");
		}
		Serial.println("");
	#endif
	_serial->write(data, size);
	_serial->flush();
}

int WinsenSensors::getSensorData()
{
	#if DEBUG > 0
		Serial.println("Get sensor data...");
	#endif

	uint32_t startTime = millis();
	int i = 0;
	uint8_t incomingBytes[9] = {0xff};
	//limit to 9 bytes received as sensor message is 9 bytes.
	while (i <= 8)
	{
		#if DEBUG > 0
			Serial.print("wait...");
			Serial.println(i);
		#endif

		if ( _serial->available() )   //get uart data
		{
			incomingBytes[i] = _serial->read();
			i++;
		}
		else if (millis() - startTime > UARTTIMEOUTMS) //check if sensor replied in a short time
		{
			#if DEBUG > 0
				Serial.println("Data transfert timeout");
			#endif
			return UARTTIMEOUT_FAILED;
		}
		else
		{
			#if DEBUG > 0
				Serial.println("Still no data from serial...");
			#endif
			delay(10);
		}
	}
	
	//all sensor data received
	#if DEBUG > 0
		Serial.print("Data acquisition complete: ");
		for (int i=0; i<=8 ;i++)
		{
			//Serial.print(i,HEX);
			Serial.print(incomingBytes[i], HEX);
			Serial.print("_");
		}
		Serial.println("");
	#endif

	if (incomingBytes[8] != getChecksum(incomingBytes) )
	{
		#if DEBUG > 0
			Serial.println("Checksum error");
		#endif
		_gasConcentration = -1;
		return CHECKSUM_FAILED;
	}
	else
	{
		#if DEBUG > 0
			Serial.println("Checksum OK");
		#endif
 
		_gasConcentration = computeGasConcentration(incomingBytes[2], incomingBytes[3]);
		return OPERATION_SUCCESS;
	}
}

void WinsenSensors::setCalibrationZeroPoint()
{	
	#if DEBUG > 0
		Serial.println("Calibrating sensor method:Zero");
	#endif
	uint8_t request[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
	sendSensorData(request, 9);	
}

void WinsenSensors::setCalibrationSpanPoint(uint8_t spanHighPos, uint8_t spanLowPos)
{
	#if DEBUG > 0
		Serial.println("Calibrating sensor method:Span");
		Serial.print("Span high:");
		Serial.print(spanHighPos,DEC);
		Serial.print("Span2:");
		Serial.print(spanLowPos,DEC);
	#endif
	uint8_t request[9] = {0xFF, 0x01, 0x88, spanHighPos, spanLowPos, 0x00, 0x00, 0x00};
	request[8] = getChecksum(request);
	sendSensorData(request, 9);
}

int WinsenSensors::computeGasConcentration(uint8_t byte2, uint8_t byte3)
{
	#if DEBUG > 0
		Serial.print("Computed data from sensor: ");
		Serial.print("byte2:");
		Serial.print(byte2);
		Serial.print(" byte3:");
		Serial.println(byte3);
	#endif

	int value = byte2 * 256 + byte3;
	#if DEBUG > 0
		Serial.print(value);
		Serial.print(" ");

	#endif
	return value;
}


