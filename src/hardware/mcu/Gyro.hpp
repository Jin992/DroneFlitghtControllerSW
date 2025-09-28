//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef GYRO_HPP
#define GYRO_HPP
#include "Wire.h"
#include "ImuTypes.hpp"

namespace imu {

class Gyro {
public:
	Gyro(TwoWire &wire);
	GyroData measure();

private:
	void m_init();
	void m_calibrate();
	GyroData m_pollGyro();

private:
	TwoWire &m_wire;
	GyroData m_calibrationAdjust;
};

} // imu

#endif //GYRO_HPP
