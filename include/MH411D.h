
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

#ifndef MH411D_h
#define MH411D_h
#endif

#include "WinsenSensors.h"

class MH411D : public WinsenSensors
{
	public:
	/**
	 * Default empty constructor
	 */
	MH411D();

	/**
	 * Start sensor initialisation
	 * @param Stream A reference to access already instanciated Serial Stream
	 * @param scale A char either 'p' or '%'. Default is [p] (ppm range).
	 * @return void
	 */
	void begin(Stream * serial, char scale);


	/**
	 * Get current used scale for the running program
	 * @return char either 'p' | '%'
	 */
	char getScale();

	/**
	 * Get current used scale for the running program
	 * @param char  Could be [%] => or [p] => (range 0-5000 ppm).
	 * @return void
	 */
	void setScale(char scale);

	/**
	 * @var const char* Sensor model name
	 */
	static constexpr char SENSORMODEL[] = "MH411D";

	private:
	/**
	 * @var char sensor scale to use. Could be [%] => or [p] => (range 0-5000 ppm).
	 * See sensor datasheet for more details
	 */
	char _scale;

	/**
	 * compute O2 concentration in ppm or percent depending on sensor
	 * @param  uint8_t Concentration high-position
	 * @param  uint8_t Concentration low-position
	 * @return int gas concentration
	 */
	int computeGasConcentration(uint8_t byte2, uint8_t byte3) override;
};