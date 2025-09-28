//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "Accel.hpp"
#include <cmath>

namespace imu {
	Accel::Accel(TwoWire &wire)
	: m_wire(wire), m_verticalVelocity(0)
	{}

	void Accel::m_init() {
		m_wire.beginTransmission(0x68);
		m_wire.write(0x1A);
		m_wire.write(0x05);
		m_wire.endTransmission();
		m_wire.beginTransmission(0x68);
		m_wire.write(0x1C);
		m_wire.write(0x10);
		m_wire.endTransmission();
	}

	AccelData Accel::measure() {
		m_wire.beginTransmission(0x68);
		m_wire.write(0x3B);
		m_wire.endTransmission();
		m_wire.requestFrom(0x68,6);

		const int16_t AccXLSB = m_wire.read() << 8 | m_wire.read();
		const int16_t AccYLSB = m_wire.read() << 8 | m_wire.read();
		const int16_t AccZLSB = m_wire.read() << 8 | m_wire.read();

		const float AccX = static_cast<float>(AccXLSB) / 4096;
		const float AccY = static_cast<float>(AccYLSB) / 4096;
		const float AccZ = static_cast<float>(AccZLSB) / 4096;

		const float anglePitch = static_cast<float>(-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180));
		const float angleRoll = static_cast<float>(atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180));
		auto verticalVelocity = calculateVerticalVelocity(anglePitch, angleRoll, AccX, AccY, AccZ);

		return { angleRoll, anglePitch, verticalVelocity};
	}

	AccelVerticalVelocityData Accel::calculateVerticalVelocity(float anglePitch, float angleRoll, float accX, float accY, float accZ) {
		float accZInertial = -sin(anglePitch * (3.142 / 180)) * accX
							+ cos(anglePitch * (3.142 / 180))
							* sin(angleRoll * (3.142 / 180)) * accY
							+ cos(anglePitch * (3.142 / 180))
							* cos(angleRoll * (3.142 / 180)) * accZ;

		// convert the acceleration to cm/s2
		accZInertial = (accZInertial - 1) * 9.81 * 100;
		m_verticalVelocity = m_verticalVelocity + accZInertial * 0.004;
		return {m_verticalVelocity, accZInertial};
	};
} // imu