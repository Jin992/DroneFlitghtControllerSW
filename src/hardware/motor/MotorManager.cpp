//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "hardware/motor/MotorManager.hpp"

#include <core_pins.h>

#define THROTTLE_CUTOFF 1000
#define THROTTLE_IDLE 1180
#define THROTTLE_MAX 2000
#define THROTTLE_INPUT_MAX 1800
#define MOTOR_PWM_RESOLUTION 12

namespace motor {
	MotorManager::MotorManager(int motor1Pin, int motor2Pin, int motor3Pin, int motor4Pin)
	: m_motors{motor1Pin, motor2Pin, motor3Pin, motor4Pin}
	{
		analogWriteResolution(MOTOR_PWM_RESOLUTION);
	}

	void MotorManager::adjustMotors(float inputThrottle, float inputRoll, float inputPith, float inputYaw) {
		auto motorThrust = calculateMotorThrust(inputThrottle, inputRoll, inputPith, inputYaw);
		for (std::size_t i = 0; i < m_motors.size(); ++i) {
			m_motors[i].setPwm(motorThrust[i]);
		}
	}

	void MotorManager::stopMotors() {
		for (auto& motor : m_motors) {
			motor.setPwm(THROTTLE_CUTOFF);
		}
	}

	std::array<float, 4> MotorManager::calculateMotorThrust(float inputThrottle, float inputRoll, float inputPith, float inputYaw) {
		std::array<float, 4> motorsInput = {0};

		if (inputThrottle > THROTTLE_INPUT_MAX)
			inputThrottle = THROTTLE_INPUT_MAX;

		motorsInput[0] = (inputThrottle - inputRoll - inputPith - inputYaw);
		motorsInput[1] = (inputThrottle - inputRoll + inputPith + inputYaw);
		motorsInput[2] = (inputThrottle + inputRoll + inputPith - inputYaw);
		motorsInput[3] = (inputThrottle + inputRoll - inputPith + inputYaw);

		for (auto& input: motorsInput) {
			if (input > THROTTLE_MAX)
				input = THROTTLE_MAX -1;
		}

		for (auto& input: motorsInput) {
			if (input < THROTTLE_IDLE)
				input = THROTTLE_IDLE;
		}

		return motorsInput;
	}
} // motor