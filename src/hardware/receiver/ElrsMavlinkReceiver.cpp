//
// Created by Yevhen Arteshchuk on 06.10.2025.
//

#include "ElrsMavlinkReceiver.hpp"
#include <usb_serial.h>

ElrsMavlinkReceiver::ElrsMavlinkReceiver(HardwareSerialIMXRT &serial)
	: m_serialRef(serial)
{
	m_serialRef.begin(460800);
}

std::optional<mavlink_message_t> ElrsMavlinkReceiver::poll() const {
	 mavlink_status_t status;
	 mavlink_message_t msg;
	 int chan = MAVLINK_COMM_0;

	 while (m_serialRef.available() > 0) {
	 	uint8_t byte = m_serialRef.read();
	 	if (mavlink_parse_char(chan, byte, &msg, &status))
	 	{
	 		//Serial.printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
	 		return msg;
	 	}
	}


	return std::nullopt;
}

void ElrsMavlinkReceiver::send(const mavlink_message_t &msg) {
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
		const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

		for (int i = 0; i < len; i++) {
			m_serialRef.write(buffer[i]);
		}
}

