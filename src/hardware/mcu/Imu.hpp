//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef IMU_HPP
#define IMU_HPP
#include "Accel.hpp"
#include "ImuTypes.hpp"
#include "Gyro.hpp"
namespace imu {

class Imu {
public:
	Imu(TwoWire &wire);
	ImuData measure();

	bool isCalibrated();

private:
	Gyro m_gyro;
	Accel m_accel;
};

} // imu

#endif //IMU_HPP
