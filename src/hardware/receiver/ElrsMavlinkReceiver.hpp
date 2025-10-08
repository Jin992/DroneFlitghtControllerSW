//
// Created by Yevhen Arteshchuk on 06.10.2025.
//

#ifndef TEENSY4_ELRSMAVLINKRECEIVER_HPP
#define TEENSY4_ELRSMAVLINKRECEIVER_HPP

#include "MAVLink.h"
#include <HardwareSerial.h>
#include <optional>

class ElrsMavlinkReceiver {
public:
	explicit ElrsMavlinkReceiver(HardwareSerialIMXRT &serial);
	std::optional<mavlink_message_t> poll() const;
	void send(const mavlink_message_t &msg);

private:
	HardwareSerialIMXRT &m_serialRef;
	uint8_t inBuffer[MAVLINK_MAX_PAYLOAD_LEN]{};
};


#endif //TEENSY4_ELRSMAVLINKRECEIVER_HPP