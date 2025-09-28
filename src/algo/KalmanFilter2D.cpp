//
// Created by Yevhen Arteshchuk on 28.09.2025.
//

#include "KalmanFilter2D.hpp"

namespace algo {
	KalmanFilter2d::KalmanFilter2d()
	: m_F{1, 0.004, 0, 1}
	, m_G{0.5 * 0.004 * 0.004, 0.004}
	, m_P{0, 0, 0, 0}
	, m_Q(m_G * ~m_G * 10 * 10)
	, m_S{0, 0}
	, m_H{1, 0}
	, m_I{1, 0, 0, 1}
	, m_R{30 * 30}
	{}

	Kalman2dVerticalVelAltData KalmanFilter2d::calculate(float baroAlt, float accZInertial) {
		m_Acc = {accZInertial};
		m_S = m_F * m_S + m_G * m_Acc;
		m_P = m_F * m_P * ~m_F + m_Q;
		m_L = m_H * m_P * ~m_H + m_R;
		m_K = m_P * ~m_H * Invert(m_L);
		m_M = {baroAlt};
		m_S = m_S + m_K * (m_M - m_H * m_S);
		float altKalman = m_S(0,0);
		float velKalman = m_S(1,0);
		m_P = (m_I - m_K * m_H) * m_P;
		return {velKalman, altKalman};
	}
} // algo