//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP
#include "FlightControllerState.hpp"
#include "MavlinkHandler.hpp"
#include "algo/Pid.hpp"
#include "algo/KalmanFilter.hpp"
#include "algo/KalmanFilter2D.hpp"
#include "hardware/Rate.hpp"
#include "hardware/motor/MotorManager.hpp"
#include "hardware/mcu/Imu.hpp"
#include "hardware/receiver/ElrsMavlinkReceiver.hpp"

namespace fc {
class FlightController {
public:
	FlightController();
	FCInput calculatePositionInSpace(imu::ImuData &imuData, ControlState &controlState);
	void reset();
	void runOnce();

private:
	FlightControllerState m_state;
	algo::KalmanFilter m_kalmanFilterRoll;
	algo::KalmanFilter m_kalmanFilterPitch;
	algo::KalmanFilter2d m_kamlanFilterVerticalVelAlt;

	algo::PID m_pidAngleRoll = algo::PID({2,0,0});
	algo::PID m_pidAnglePitch = algo::PID({2,0,0});
	algo::PID m_pidRatePitch = algo::PID({0.40,3.5,0.025});
	algo::PID m_pidRateRoll = algo::PID({0.40,3.5,0.025});
	algo::PID m_pidRateYaw = algo::PID({2,12,0});
	algo::PID m_pidVerticalåVelocity = algo::PID({3.5,0.0015,0.01});

	ElrsMavlinkReceiver			m_receiver;
	motor::MotorManager			m_motorMgr;
	imu::Imu					m_imuSensor;
	uint32_t					m_loopTimer;
	ControlState				m_rcControlInput;
	MavlinkHandler				m_mavlinkHandler;
};

} // algo

#endif //FLIGHTCONTROLLER_HPP
