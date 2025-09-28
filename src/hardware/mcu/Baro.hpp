//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef BARO_HPP
#define BARO_HPP
#include <cstdint>
#include <Wire.h>

namespace imu {

	typedef struct {
		uint16_t dig_T1;
		uint16_t dig_P1;
		int16_t dig_T2;
		int16_t dig_T3;
		int16_t dig_P2;
		int16_t dig_P3;
		int16_t dig_P4;
		int16_t dig_P5;
		int16_t dig_P6;
		int16_t dig_P7;
		int16_t dig_P8;
		int16_t dig_P9;
	} SensorCalibrationValues;

class Baro {
public:
	Baro(TwoWire &wire);
	float measure();

	float groundAltitude();

private:
	void m_init();
	void m_calibrate();
	float m_measureAltitude();
private:
	TwoWire &m_wire;
	float m_altitudeBaroStartUp;
	float m_groundAltidute;



};

} // imu

#endif //BARO_HPP
