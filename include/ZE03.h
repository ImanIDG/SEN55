#pragma once
/*
 * ZE03 class is based on main MH411D class.
 */
#include "WinsenSensors.h" 
#include <string.h>

#ifndef ZE03_h
#define ZE03_h
#endif

#define SENSOR_NH3   0x02
#define SENSOR_H2S   0x03
#define SENSOR_CO    0x04
#define SENSOR_O2    0x05
#define SENSOR_H2    0x06
#define SENSOR_C2H   0x08
#define SENSOR_HCH   0x17
#define SENSOR_O3    0x2A
#define SENSOR_SO2   0x2B
#define SENSOR_NO    0x2C
#define SENSOR_HC    0x2E
//#define SENSOR_HC    0x2F
#define SENSOR_CL    0x31
#define SENSOR_HF    0x33
#define SENSOR_ET    0x34
#define SENSOR_PH    0x45
#define SENSOR_DG    0x46

#define MODE_QA         0x04
#define MODE_ACTIVEUP   0x03



struct Sensor {
   const char name[4];
   uint8_t code;
   uint8_t resolution;
};

Sensor sensorsList[] = {
    {"NH3", 0x02, 0},         //0	
    {"H2S", 0x03, 0},         //1	
    {"CO", 0x04, 0},         //2	
    {"O2", 0x05, 1},         //3	
    {"H2", 0x06, 0},         //4	
    {"C2H", 0x08, 1},         //5	
    {"HCH", 0x17, 1},         //6	
    {"O3", 0x2A, 1},         //7	
    {"SO2", 0x2B, 1},         //8	
    {"NO", 0x2C, 1},         //9	
    {"HC", 0x2E, 1},         //10	
    {"HC", 0x2F, 1},         //11	
    {"CL", 0x31, 1},         //12	
    {"HF", 0x33, 1},         //13	
    {"ET", 0x34, 1},         //14	
    {"PH", 0x45, 1},         //15	
    {"DG", 0x46, 2},         //16	


};

class ZE03 : public WinsenSensors
{
	public:
		ZE03();
		/**
		 * Start sensor initialisation
		 * @param Stream  reference to access this already instanciated Serial Stream
		 * @param uint8_t sensor code : 
         *     SENSOR_NH3 | SENSOR_H2S | SENSOR_CO | SENSOR_O2 |  
         *     SENSOR_H2 |  SENSOR_C2H |  SENSOR_HCH |  SENSOR_O3 |  
         *     SENSOR_SO2 |  SENSOR_NO |  SENSOR_HC |  SENSOR_HC |
         *     SENSOR_CL | SENSOR_HF |  SENSOR_ET |  SENSOR_PH |  SENSOR_DG 
         * @param uint8_t sensor communication mode Q&A or ActiveUpload MODE_QA | MODE_ACTIVEUP
		 * @return int OPERATION_SUCCESS | OPERATION_FAILED
		 */
		int begin(Stream * serial, uint8_t sensorCode, uint8_t commMode);

        /**
         * @var Sensor model name
         */
        static constexpr char SENSORMODEL[] = "ZE03";

	protected:
        /**
		 * @var uint8_t Sensor model as human readable 4 char name like CO2, H2, O3...
		 */
        char _type[4] = {0};

        /**
		 * @var uint8_t Sensor code see datasheet for more details
		 */
        uint8_t _code = 0;

        /**
		 * @var uint8_t Sensor code see datasheet for more details
		 */
        uint8_t _resolution = 0;

        /**
		 * @var uint8_t Sensor mode can be Q&A or Active Upload (send data every sec);
		 */
        uint8_t _commMode = 0;

		/**
		 * compute checksum from user request or sensor data
		 * @param pointer to uint8_t
		 * @return int
		 */
		uint8_t getChecksum(uint8_t * data, uint8_t length);

		/**
		 * compute O2 concentration
		 * @param  uint8_t Concentration high-position
		 * @param  uint8_t Concentration low-position
		 * @return int gas concentration
		 */
		int computeGasConcentration(uint8_t byte2, uint8_t byte3);		
};