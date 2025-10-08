//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "FlightController.hpp"
#include "hardware/Led.hpp"
#include <cmath>

namespace fc {
	FlightController::FlightController()
	: m_state{.mode=MAV_MODE_MANUAL_DISARMED, .rate ={0,0,0}}
	, m_receiver(Serial2)
	, m_motorMgr(1,2,3,4)
	, m_imuSensor(Wire)
	, m_loopTimer{micros()}
	, m_rcControlInput{false, 0, {0.0, 0.0, 0.0}}
	{
		Led::statusRed(true);
		if (m_imuSensor.isCalibrated()) {
			Led::statusRed(false);
			Led::statusGreen(true);
		}
	}


	FCInput FlightController::calculatePositionInSpace(imu::ImuData &imuData, ControlState &rcControlInput) {
		// vertical velocity mode, experimentsl, not working now
		//auto kalmanVerticalVelAltr = m_kamlanFilterVerticalVelAlt.calculate(imuData.altitudeCm, imuData.accel.verticalVelocity.accZInertial);
		//const float errorVelocityVertical = 0.3 * (rcControlInput.throttle - 1500) - kalmanVerticalVelAltr.velocity;
		//rcControlInput.throttle = m_pidVerticalVelocity.calculate(errorVelocityVertical) + 1500;

		const float kalmanAngleRoll = m_kalmanFilterRoll.calculateAngle1d(imuData.gyro.roll, imuData.accel.angleRoll);
		const float kalmanAnglePitch = m_kalmanFilterPitch.calculateAngle1d(imuData.gyro.pitch, imuData.accel.anglePitch);

		const float ErrorAngleRoll = rcControlInput.rate.roll - kalmanAngleRoll;
		const float ErrorAnglePitch = rcControlInput.rate.pitch - kalmanAnglePitch;

		const float DesiredRateRoll = m_pidAngleRoll.calculate(ErrorAngleRoll);
		const float DesiredRatePitch = m_pidAnglePitch.calculate(ErrorAnglePitch);

		const float ErrorRateRoll = DesiredRateRoll - imuData.gyro.roll;
		const float ErrorRatePitch = DesiredRatePitch - imuData.gyro.pitch;
		const float ErrorRateYaw = rcControlInput.rate.yaw - imuData.gyro.yaw;

		return {m_pidRateRoll.calculate(ErrorRateRoll),
				 m_pidRatePitch.calculate(ErrorRatePitch),
				  m_pidRateYaw.calculate(ErrorRateYaw)
		};
	}

	void FlightController::reset() {
		m_pidAngleRoll.reset();
		m_pidAnglePitch.reset();
		m_pidRateRoll.reset();
		m_pidRatePitch.reset();
		m_pidRateYaw.reset();
	}


	void FlightController::runOnce() {
		std::vector<mavlink_message_t> response;

		m_state.imuData = m_imuSensor.measure();

		// Calculate attitude and quaternion before processing control

		auto rxData = m_receiver.poll();
		if (rxData) {
			response = m_mavlinkHandler.handleRx(rxData.value(), m_state);
		}

		m_state.rate = calculatePositionInSpace(m_state.imuData, m_state.rcControl);
		if (m_state.rcControl.arm == false) {
			m_motorMgr.stopMotors();
			reset();
			Led::statusArm(false);
		} else {
			Led::statusArm(true);
			m_motorMgr.adjustMotors(m_state.rcControl.throttle, m_state.rate.roll, m_state.rate.pitch, m_state.rate.yaw);
		}
		for (const auto&m: response)
			m_receiver.send(m);

		auto timeSpent = micros() - m_loopTimer;
		//Serial.printf("Time spent %d\n", timeSpent);
		while (timeSpent < 4000){}
		m_loopTimer = micros();
	}
} // algo