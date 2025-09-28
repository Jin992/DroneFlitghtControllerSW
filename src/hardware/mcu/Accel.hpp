//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef ACCEL_HPP
#define ACCEL_HPP
#include "ImuTypes.hpp"
#include "Wire.h"
namespace imu {

class Accel {
public:
	Accel(TwoWire &wire);
	AccelData measure();
private:
	void m_init();
	AccelVerticalVelocityData calculateVerticalVelocity(float anglePitch, float angleRoll, float accX, float accY, float accZ);
private:
	TwoWire &m_wire;
	float m_verticalVelocity;
};

} // imu

#endif //ACCEL_HPP
