//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP
#include "algo/Pid.hpp"
#include "algo/KalmanFilter.hpp"
#include "hardware/Rate.hpp"
#include "hardware/receiver/ElrsReceiver.hpp"
#include "hardware/motor/MotorManager.hpp"
#include "hardware/mcu/Imu.hpp"

namespace algo {
	typedef struct {
		float roll;
		float pitch;
		float yaw;
	} FCInput;

class FlightController {
public:
	FlightController();
	FCInput calculateMotorThrust(imu::ImuData &imuData, ControlState &controlState);
	void reset();
	void runOnce();

private:
	KalmanFilter m_kalmanFilterRoll;
	KalmanFilter m_kalmanFilterPitch;

	PID m_pidAngleRoll = PID({2,0,0});
	PID m_pidAnglePitch = PID({2,0,0});
	PID m_pidRatePitch = PID({0.45,3.5,0.025});
	PID m_pidRateRoll = PID({0.45,3.5,0.025});
	PID m_pidRateYaw = PID({2,12,0});

	ElrsReceiver			m_receiver;
	motor::MotorManager		m_motorMgr;
	imu::Imu				m_imuSensor;
	uint32_t				m_loopTimer;
	ControlState			m_rcControlInput;
};

} // algo

#endif //FLIGHTCONTROLLER_HPP
