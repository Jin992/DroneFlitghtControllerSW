//
// Created by Yevhen Arteshchuk on 07.10.2025.
//

#ifndef TEENSY4_FLIGHTCONTROLLERSTATE_HPP
#define TEENSY4_FLIGHTCONTROLLERSTATE_HPP

#include "MAVLink.h"
#include "hardware/Rate.hpp"
#include "hardware/mcu/ImuTypes.hpp"

namespace fc {
	typedef struct {
		float roll;
		float pitch;
		float yaw;
	} FCInput;

	struct FlightControllerState {
		MAV_MODE		mode;
		FCInput			rate;
		ControlState	rcControl;
		imu::ImuData	imuData;
	};
} // fc

#endif //TEENSY4_FLIGHTCONTROLLERSTATE_HPP