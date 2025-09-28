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
	TwoWire &m_wire;
};

} // imu

#endif //ACCEL_HPP
