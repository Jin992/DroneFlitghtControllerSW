//
// Created by Yevhen Arteshchuk on 07.10.2025.
//

#ifndef TEENSY4_MAVLINKHANDLER_HPP
#define TEENSY4_MAVLINKHANDLER_HPP
#include <optional>
#include <vector>

#include "mavlink/mavlink_types.h"
#include "FlightControllerState.hpp"

namespace fc {
	class MavlinkHandler {
	public:
		std::vector<mavlink_message_t> handleRx(const mavlink_message_t &msg, FlightControllerState &state);
	private:
		std::optional<mavlink_message_t> m_produceHeartBeat(FlightControllerState &state);
		std::optional<mavlink_message_t> m_attitudeQuaterion(FlightControllerState &state);
		std::optional<mavlink_message_t> m_handleInput(const mavlink_message_t &msg, FlightControllerState &state);
	};
} // receiver

#endif //TEENSY4_MAVLINKHANDLER_HPP