#pragma once
/**
 * A nice library that handle MH410D/MH411D sensors from winsen sensors
 *
 * name     : MH411D
 * Author   : Guillaume CHARDIN
 * Version  : 0.0.1
 * Date     : 13/02/2024
 * url      : https://github.com/guillaumechardin/MH411D
 */
#define WINSENSENSORS_h


#include "Arduino.h"
#include <string.h>

#define VERSION 		"0.0.1"
#define DEBUG 			1

class WinsenSensors
{
	public:

    	/**
		 * @var define a bootup time wait to let the sensor stabilize and get ready to use
		 */
		static const int BOOTUPDELAYMS = 7000;

		//default constructor
		WinsenSensors();
		
		/**
		 * Start sensor initialisation
		 * @param Stream A reference to access already instanciated Serial Stream
         * @param scale MH411D : scale to be used 0-5000ppm  =>  [p] | 0-5% => [%]
		 * @return void
		 */
		void begin(Stream * serial);
        void begin(Stream * serial, char scale);
        
		
		/**
		 * getGasConcentration return CO2 concentration, commonly return result in ppm
		 * (see specific sensor datasheet for more informations)
		 * 
		 * @return int on success or OPERATION_FAILED on failure
		 */
		int getGasConcentration();
		
		/**
		 * Set sensor calibration on Zero point.
		 * @return void
		 */
		void setCalibrationZeroPoint();
		
		/**
		 * Set sensor calibration level to a user defined span points
		 * @arg spanHighPos a ppm value (sure ?)
		 * @arg spanLowPos a ppm value (sure ?)
		 * @return void
		 */
		void setCalibrationSpanPoint(uint8_t spanHighPos, uint8_t spanLowPos);
		
		/**
		 * Start measure process on sensor.
		 * @return int operation status.  OPERATION_SUCCESS | UARTTIMEOUTFAILED | CHECKSUMFAILED 
		 *
		 */
		int startMeasure();
		

	protected:
		/**
		 * @var Stream pointer to an instanciated arduino Serial
		 */
		Stream *_serial = nullptr;
		
		/**
		 * @var int representing measure of gas concentration
		 */
		int _gasConcentration = -1;

        /**
         * @var const char* Sensor model name
         */
        static constexpr char SENSORMODEL[] = "";

        //some more specific var for return values
        static const int OPERATION_SUCCESS  = 1 ;
        static const int OPERATION_FAILED   = -1 ;
        static const int CHECKSUM_FAILED    = -10 ;
        static const int UARTTIMEOUT_FAILED	= -20 ;
		
		/**
		 * Define sensor scale to be used. Either UNITS_PPM (for range 0-5000ppm) or UNITS_PERCENT (0-50%).
         * Avoid changing this during operations.
		 * See sensor datasheet for more details.
		 * @return void
		 */
		//void setScale(int scale);
		
		/**
		 * Get data from sensor
		 * @param uint8_t 9 bytes array to store reply from serial
		 * @return int 
		 */
		int getSensorData();
		
		/**
		 * send data to sensor
		 * @param uint8_t 9 bytes array of data to send to sensor on serial
		 * @param int usually 9 as sensor way for 9 bytes
		 * @return void 
		 */
		void sendSensorData(uint8_t *data, uint8_t length);
		
		/**
		 * compute checksum from user request or sensor data
		 * @param uint8_t pointer to usual 8 bytes data set
		 * @return int
		 */
		uint8_t getChecksum(uint8_t * data);
		 
		 /**
		  * compute co2 concentration depending units
		  * @param  uint8_t Concentration high-position
		  * @param  uint8_t Concentration low-position
		  * @return int gas concentration
		  */
		virtual int computeGasConcentration (uint8_t byte2, uint8_t byte3);
		
        /**
         *  @var const int hardcoded timeout when comunication with sensor is too long
         */
        static const int UARTTIMEOUTMS = 5000;


		
	
};


