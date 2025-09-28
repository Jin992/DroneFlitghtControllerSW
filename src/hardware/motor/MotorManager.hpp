//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef MOTORMANAGER_HPP
#define MOTORMANAGER_HPP
#include <array>
#include <hardware/motor/Motor.hpp>

namespace motor {

class MotorManager {
public:
	MotorManager(int motor1Pin,int motor2Pin,int motor3Pin,int motor4Pin);
	void adjustMotors(float inputThrottle, float inputRoll, float inputPith, float inputYaw);
	void stopMotors();

private:
	std::array<float, 4> calculateMotorThrust(float inputThrottle, float inputRoll, float inputPith, float inputYaw);

private:
	std::array<Motor, 4> m_motors;

};

} // motor

#endif //MOTORMANAGER_HPP
