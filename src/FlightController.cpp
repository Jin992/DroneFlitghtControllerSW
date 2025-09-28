//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "FlightController.hpp"
#include "hardware/Led.hpp"

namespace algo {
	FlightController::FlightController()
	: m_receiver(Serial4)
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

	FCInput FlightController::calculateMotorThrust(imu::ImuData &imuData, ControlState &rcControlInput) {

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
		auto imuData = m_imuSensor.measure();
		auto rxData = m_receiver.poll();
		if (rxData) {
			m_rcControlInput = ElrsReceiver::convertElrsReceiverDataToControlData(*rxData);
		}

		auto fcMotorInput = calculateMotorThrust(imuData, m_rcControlInput);
		if (m_rcControlInput.arm == false) {
			m_motorMgr.stopMotors();
			reset();
			Led::statusArm(false);
		} else {
			Led::statusArm(true);
			m_motorMgr.adjustMotors(m_rcControlInput.throttle, fcMotorInput.roll, fcMotorInput.pitch, fcMotorInput.yaw);
		}

		auto timeSpent = micros() - m_loopTimer;
		//Serial.printf("Time spent %d\n", timeSpent);
		while (timeSpent < 4000){}
		m_loopTimer = micros();
	}
} // algo