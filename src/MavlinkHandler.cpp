//
// Created by Yevhen Arteshchuk on 07.10.2025.
//

#include "MavlinkHandler.hpp"
#include "MAVLink.h"
#include <core_pins.h>
#include <wiring.h>


namespace fc {
	std::vector<mavlink_message_t> MavlinkHandler::handleRx(const mavlink_message_t &msg, fc::FlightControllerState &state) {
		std::vector<mavlink_message_t> msgsToSend;
		auto hb = m_produceHeartBeat(state);
		if (hb.has_value()) {
			msgsToSend.push_back(hb.value());
		}
		auto res = m_handleInput(msg, state);
		if (res.has_value()) {
			msgsToSend.push_back(res.value());
		}

		auto att = m_attitudeQuaterion(state);
		if (att.has_value()) {
			msgsToSend.push_back(att.value());
		}
		return  msgsToSend;
	}

	std::optional<mavlink_message_t> MavlinkHandler::m_produceHeartBeat(FlightControllerState &state) {
		static uint32_t hbTimer = micros();
		if (micros() - hbTimer > 1000000) {
			mavlink_message_t msg;
			mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR,
				MAV_AUTOPILOT_GENERIC, state.mode, 0, MAV_STATE_ACTIVE);
			hbTimer = micros();
			return msg;
		}
		return std::nullopt;
	}

	static void eulerToQuat(float roll, float pitch, float yaw,
						float &w, float &x, float &y, float &z) {
		float cr = cosf(roll * 0.5f);
		float sr = sinf(roll * 0.5f);
		float cp = cosf(pitch * 0.5f);
		float sp = sinf(pitch * 0.5f);
		float cy = cosf(yaw * 0.5f);
		float sy = sinf(yaw * 0.5f);

		w = cr * cp * cy + sr * sp * sy;
		x = sr * cp * cy - cr * sp * sy;
		y = cr * sp * cy + sr * cp * sy;
		z = cr * cp * sy - sr * sp * cy;
	}


	std::optional<mavlink_message_t> MavlinkHandler::m_attitudeQuaterion(FlightControllerState &state) {
		static uint32_t hbTimer = micros();
		if (micros() - hbTimer > 200000) {
			mavlink_message_t msg;

			// Use the calculated quaternion from the flight controller state
			float qw = state.quaternion.w;
			float qx = state.quaternion.x;
			float qy = state.quaternion.y;
			float qz = state.quaternion.z;

			// Use actual gyro rates (body frame angular velocities)
			float rollspeed = state.imuData.gyro.roll;   // body roll rate rad/s
			float pitchspeed = state.imuData.gyro.pitch; // body pitch rate rad/s
			float yawspeed = state.imuData.gyro.yaw;     // body yaw rate rad/s

			mavlink_msg_attitude_quaternion_pack(
				1, MAV_COMP_ID_AUTOPILOT1, &msg,
				millis(),
				qw, qx, qy, qz,
				rollspeed, pitchspeed, yawspeed,
				{0}  // repr_offset_q identity
			);

			hbTimer = micros();
			return msg;
		}
		return std::nullopt;
	}

	namespace {
		float normalize(int value, float old_min, float old_max, float new_min, float new_max) {
			return new_min + (new_max - new_min) * ((value - old_min) / (old_max - old_min));
		}
	}

	std::optional<mavlink_message_t> MavlinkHandler::m_handleInput(const mavlink_message_t &msg, FlightControllerState &state) {
		switch(msg.msgid) {
		 	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT
		 	{
		 		// Get all fields in payload (into global_position)
		 		//mavlink_msg_global_position_int_decode(&msg, &global_position);
		 		break;
		 	}
		 	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		 	{
		 		mavlink_rc_channels_override_t data;
		 		// Get all fields in payload (into rc_channels)
		 		mavlink_msg_rc_channels_override_decode(&msg, &data);

		 		if (data.chan5_raw > 1000) {
		 			state.rcControl.arm = true;
		 			state.mode = MAV_MODE_MANUAL_ARMED;
		 		} else {
		 			state.rcControl.arm = false;
		 			state.mode = MAV_MODE_MANUAL_DISARMED;
		 		}

		 		state.rcControl.throttle = normalize(data.chan3_raw, 174, 1811, 1000, 2000);
		 		state.rcControl.rate.roll = 0.1 * (normalize(data.chan1_raw, 174, 1811, 1000, 2000) - 1500);
		 		state.rcControl.rate.pitch = 0.1 * (normalize(data.chan2_raw, 174, 1811, 1000, 2000) - 1500);
		 		state.rcControl.rate.yaw = 0.15 * (normalize(data.chan4_raw, 174, 1811, 1000, 2000) - 1500);
		 		//Serial.printf("RC chan1: %d chan2: %d chan3: %d chan4: %d\n", data.chan1_raw, data.chan2_raw, data.chan3_raw, data.chan4_raw);
		 		break;
		 	}

		 	case MAVLINK_MSG_ID_GPS_STATUS:
		 	{
		 		// Get just one field from payload
		 		//visible_sats = mavlink_msg_gps_status_get_satellites_visible(&msg);
		 		break;
		 	}
		 	case MAVLINK_MSG_ID_ATTITUDE: {
		 		break;
		 	}
			case  MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {

				break;
			}
		 	default:
				break;
		}
		return std::nullopt;
	}
}
