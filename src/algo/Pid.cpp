//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "Pid.hpp"

namespace algo {
	const float cycleDuration = 0.004;

	PID::PID(PIDConfig config)
	: m_config(config)
	, m_prevError(0)
	, m_prevIterm(0)
	{}


	float PID::calculate(float error) {
		const float Pterm = m_config.P * error;

		float Iterm = m_prevIterm + m_config.I * (error + m_prevError) * cycleDuration / 2;
		if (Iterm > 400) {
			Iterm = 400;
		} else if (Iterm < -400) {
			Iterm = -400;
		}

		const float Dterm = m_config.D * (error - m_prevError) / cycleDuration;
		float PIDOutput = Pterm + Iterm + Dterm;
		if (PIDOutput > 400) {
			PIDOutput = 400;
		} else if (PIDOutput < -400) {
			PIDOutput = -400;
		}
		m_prevError = error;
		m_prevIterm = Iterm;

		return  PIDOutput;
	}

	void PID::reset() {
		m_prevError = 0;
		m_prevIterm = 0;
	}
} // algo