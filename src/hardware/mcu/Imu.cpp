//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "Imu.hpp"

namespace imu {
	Imu::Imu(TwoWire &wire)
	: m_gyro(wire), m_accel(wire) {
	}

	ImuData Imu::measure() {
		ImuData data;
		data.gyro = m_gyro.measure();
		data.accel = m_accel.measure();
		return data;
	}

	bool Imu::isCalibrated() {
		auto data = measure();
		return data.gyro.roll != 0.0;
	}
} // imu