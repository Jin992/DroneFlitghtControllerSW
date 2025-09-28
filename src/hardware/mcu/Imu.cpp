//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "Imu.hpp"

namespace imu {
	Imu::Imu(TwoWire &wire)
	: m_gyro(wire), m_accel(wire), m_baro(wire) {
	}

	ImuData Imu::measure() {
		ImuData data;
	//	uint32_t gyro_time = micros();
		data.gyro = m_gyro.measure();
	//	uint32_t gyro_time2 = micros();
	//	uint32_t accel_time = micros();
		data.accel = m_accel.measure();
	//	uint32_t accel_time2 = micros();
	//	uint32_t baro_time = micros();
		data.altitudeCm = m_baro.measure();
	//	uint32_t baro_time2 = micros();
		// Serial.printf("Cyro Time: %d Acc Time: %d Baro Timew: %d\n", gyro_time2 - gyro_time, accel_time2 - accel_time, baro_time2 - baro_time);
		data.groundAltitude = m_baro.groundAltitude();
		return data;
	}

	bool Imu::isCalibrated() {
		auto data = measure();
		return data.gyro.roll != 0.0;
	}
} // imu