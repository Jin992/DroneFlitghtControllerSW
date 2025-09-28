//
// Created by Yevhen Arteshchuk on 28.09.2025.
//

#ifndef KALMANFILTER2D_HPP
#define KALMANFILTER2D_HPP

#include <BasicLinearAlgebra.h>

namespace algo {

	typedef struct {
		float velocity;
		float altitude;
	} Kalman2dVerticalVelAltData;

class KalmanFilter2d {
public:
	KalmanFilter2d();
	Kalman2dVerticalVelAltData calculate(float baroAlt, float accZinertial);


private:
	BLA::Matrix<2,2> m_F;
	BLA::Matrix<2,1> m_G;
	BLA::Matrix<2,2> m_P;
	BLA::Matrix<2,2> m_Q;
	BLA::Matrix<2,1> m_S;
	BLA::Matrix<1,2> m_H;
	BLA::Matrix<2,2> m_I;
	BLA::Matrix<1,1> m_R;
	BLA::Matrix<1,1> m_Acc;
	BLA::Matrix<2,1> m_K;
	BLA::Matrix<1,1> m_L;
	BLA::Matrix<1,1> m_M;

};

} // algo

#endif //KALMANFILTER2D_HPP
