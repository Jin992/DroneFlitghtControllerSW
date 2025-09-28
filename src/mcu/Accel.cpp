//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "Accel.hpp"
#include <cmath>

namespace imu {
	Accel::Accel(TwoWire &wire)
	: m_wire(wire)
	{}

	AccelData Accel::measure() {
		m_wire.beginTransmission(0x68);
		m_wire.write(0x1A);
		m_wire.write(0x05);
		m_wire.endTransmission();
		m_wire.beginTransmission(0x68);
		m_wire.write(0x1C);
		m_wire.write(0x10);
		m_wire.endTransmission();
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

		return { static_cast<float>(atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180))
				  ,static_cast<float>(-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180))
		};
	}
} // imu