//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef IMUTYPES_HPP
#define IMUTYPES_HPP

#include <iostream>

namespace imu {
	struct GyroData {
		float roll;
		float pitch;
		float yaw;

		// Addition operator
		GyroData operator+(const GyroData& other) const {
			return {roll + other.roll, pitch + other.pitch, yaw + other.yaw};
		}

		// Subtraction operator
		GyroData operator-(const GyroData& other) const {
			return {roll - other.roll, pitch - other.pitch, yaw - other.yaw};
		}

		// Addition assignment operator
		GyroData& operator+=(const GyroData& other) {
			roll += other.roll;
			pitch += other.pitch;
			yaw += other.yaw;
			return *this;
		}

		// Subtraction assignment operator
		GyroData& operator-=(const GyroData& other) {
			roll -= other.roll;
			pitch -= other.pitch;
			yaw -= other.yaw;
			return *this;
		}

		// Division assignment operator for int
		GyroData& operator/=(int value) {
			roll /= value;
			pitch /= value;
			yaw /= value;
			return *this;
		}

		// Stream insertion operator (friend function)
		friend std::ostream& operator<<(std::ostream& os, const GyroData& gyro) {
			os << "GyroData(roll: " << gyro.roll << ", pitch: " << gyro.pitch << ", yaw: " << gyro.yaw << ")";
			return os;
		}
	};

	typedef struct {
		float angleRoll;
		float anglePitch;
	} AccelData;

	typedef struct {
		GyroData gyro;
		AccelData accel;
	} ImuData;

}

#endif //IMUTYPES_HPP
