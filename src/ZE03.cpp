/**
 * A nice library that handle ZE03xx sensors from winsen sensors.
 * ZE03xx is inherithed from main library : MH411D 
 *
 * name     : MH411D
 * Author   : Guillaume CHARDIN
 * Version  : 0.0.1
 * Date     : 17/02/2024
 * url      : https://github.com/guillaumechardin/MH411D
 */
#include "ZE03.h"


ZE03::ZE03()
{

}

int ZE03::begin(Stream * serial, uint8_t sensorCode, uint8_t commMode)
{
    _serial = serial;
    _commMode = commMode;
    
    //get sensor code and resolution
    for (int i=0 ; i<=16 ; i++)
    {
        if(sensorsList[i].code == sensorCode)
        {
            _code = sensorCode;
            strncpy(_type, sensorsList[i].name, 4);
            _resolution = sensorsList[i].resolution;
            break;
        }
    }
    #if DEBUG > 0
        Serial.print("Sensor full name: ");
        Serial.print(SENSORMODEL);
        Serial.print("-");
        Serial.println(_type);
        Serial.print("Resolution:");
        Serial.println(_resolution);
        Serial.print("Communication mode: ");
        if (_commMode ==  MODE_QA)
        {
            Serial.println("Q&A");
        }
        else
        {
            Serial.println("ACTIVE UPLOAD");
        }
    #endif

    if(_code == 0)
    {
        return OPERATION_FAILED;
    }
    else
    {
        return OPERATION_SUCCESS;
    }

    
}

uint8_t ZE03::getChecksum(unsigned char *i,unsigned char ln)
{
    unsigned char j,tempq=0;
    i += 1;
    for( j=0; j<(ln-2) ; j++)
    {
        tempq += *i;
        i++;
    }

    tempq=(~tempq)+1;
    return(tempq);
}


int ZE03::computeGasConcentration(uint8_t byte2, uint8_t byte3)
{
    //Same computation process as mh411D
    int result = WinsenSensors::computeGasConcentration(byte2, byte3);

    //proceed decimal value if needed
    return result;

}
