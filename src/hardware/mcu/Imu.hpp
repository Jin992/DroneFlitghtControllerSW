//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef IMU_HPP
#define IMU_HPP
#include "Accel.hpp"
#include "ImuTypes.hpp"
#include "Gyro.hpp"
#include "Baro.hpp"
namespace imu {

class Imu {
public:
	Imu(TwoWire &wire);
	ImuData measure();

	bool isCalibrated();

private:
	Gyro m_gyro;
	Accel m_accel;
	Baro m_baro;
};

} // imu

#endif //IMU_HPP
