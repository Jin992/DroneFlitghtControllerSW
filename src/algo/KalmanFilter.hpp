//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

namespace algo {

class KalmanFilter {
public:
	KalmanFilter();
	float calculateAngle1d(float input, float measurement);

private:
	float m_state;
	float m_uncertainty;
};

} // algo

#endif //KALMANFILTER_HPP
