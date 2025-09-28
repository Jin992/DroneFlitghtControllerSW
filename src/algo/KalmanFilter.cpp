//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "algo/KalmanFilter.hpp"

namespace algo {

	KalmanFilter::KalmanFilter()
	: m_state(0)\
	, m_uncertainty(2*2)
	{}

	float KalmanFilter::calculateAngle1d(float input, float measurement) {
		m_state = m_state + 0.004 * input;
		m_uncertainty = m_uncertainty + 0.004 * 0.004 * 4 * 4;
		const float kalmanGain = m_uncertainty * 1 / (1 * m_uncertainty + 3 * 3);
		m_state = m_state + kalmanGain * (measurement - m_state);
		m_uncertainty = (1 - kalmanGain) * m_uncertainty;

		return m_state;
	}
} // algo