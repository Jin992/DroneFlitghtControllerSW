//
// Created by Yevhen Arteshchuk on 04.08.2025.
//

#ifndef ELRSRECEIVER_HPP
#define ELRSRECEIVER_HPP
#include <array>
#include <HardwareSerial.h>
#include <optional>
#include <vector>

#include "../Rate.hpp"

#define CRSF_SIGNAL_OK          0x00
#define CRSF_SIGNAL_LOST        0x01
//#define CRSF_SIGNAL_FAILSAFE    0x03

#define RC_CHNL_THROTTLE 2
#define RC_CHNL_YAW 3
#define RC_CHNL_PITCH 1
#define RC_CHNL_ROLL 0
#define RC_CHNL_ARM 4

// Basic setup
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
// Device address & type, The address means the destination of the data packet, so for decoder, the destination is the FC.
#define ADDR_MODULE                    0xEE  //  Crossfire transmitter
// Baud ELRS receiver baud 420000 bit/s v2
#define SERIAL_BAUDRATE 115200
#define ADDR_RADIO                     0xEA  //  Radio Transmitter
#define TYPE_SETTINGS_WRITE            0x2D
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8  // Flight Controler

//Define channel input limite
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 991
#define CRSF_CHANNEL_MAX 1811

// internal crsf variables
#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_PACKET_TIMEOUT_US 100000
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)
#define CRSF_MAX_CHANNEL 16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128
#define CRSF_PAYLOAD_SIZE_MAX   60
#define CRSF_PACKET_LENGTH 22
#define CRSF_PACKET_SIZE  26
#define CRSF_FRAME_LENGTH 24  // length of type + payload + crc

using ElrsReceiverRcChannelsData = std::array<uint16_t, CRSF_MAX_CHANNEL>;

typedef struct {
	ElrsReceiverRcChannelsData rcChannelsData;
} ElrsReceiverData;

class ElrsReceiver {
public:
	ElrsReceiver(HardwareSerialIMXRT &serial);

	std::optional<ElrsReceiverData> poll(void);

	static ControlState convertElrsReceiverDataToControlData(ElrsReceiverData &rawData);

private:
	void parseCrsfPacket(const uint8_t *buff, ElrsReceiverData &data);

private:
	HardwareSerialIMXRT &m_serialRef;
	uint8_t inBuffer[CRSF_FRAME_SIZE_MAX];
	ElrsReceiverData m_data;
};


#endif //ELRSRECEIVER_HPP
