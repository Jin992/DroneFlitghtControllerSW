//
// Created by Yevhen Arteshchuk on 04.08.2025.
//

#include "hardware/receiver/ElrsReceiver.hpp"

#include <optional>
#include <Wire.h>

#define CRSF_MAX_FRAME_SIZE 64
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_SYNC_BYTE 0xC8  // same as address

typedef enum : uint8_t {
	CRSF_FRAMETYPE_GPS = 0x02,
	CRSF_FRAMETYPE_VARIO = 0x07,
	CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
	CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
	CRSF_FRAMETYPE_AIRSPEED = 0x0A,
	CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
	CRSF_FRAMETYPE_RPM = 0x0C,
	CRSF_FRAMETYPE_TEMP = 0x0D,
	CRSF_FRAMETYPE_CELLS = 0x0E,
	CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
	CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
	CRSF_FRAMETYPE_ATTITUDE = 0x1E,
	CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
	// Extended Header Frames, range: 0x28 to 0x96
	CRSF_FRAMETYPE_DEVICE_PING = 0x28,
	CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
	CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
	CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
	CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
	//CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

	CRSF_FRAMETYPE_COMMAND = 0x32,
	CRSF_FRAMETYPE_HANDSET = 0x3A,

	// KISS frames
	CRSF_FRAMETYPE_KISS_REQ = 0x78,
	CRSF_FRAMETYPE_KISS_RESP = 0x79,
	// MSP commands
	CRSF_FRAMETYPE_MSP_REQ = 0x7A, // response request using msp sequence as command
	CRSF_FRAMETYPE_MSP_RESP = 0x7B, // reply with 58 byte chunked binary
	CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
	// Ardupilot frames
	CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;

/*
 C8 18 16 AE 70 85 2B 68 F1 8B 9F FC E2 17 7F F8 05 F8 28 08 00 00 4C 7C E2 63
+------------+--------+------+---------+-----+
| Start Byte | Length | Type | Payload | CRC |
+------------+--------+------+---------+-----+
     C8          18      16    AE...E2    63
 */

static uint8_t crsf_crc8tab[256] = {
	0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
	0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
	0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
	0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
	0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
	0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
	0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
	0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
	0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
	0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
	0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
	0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
	0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
	0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
	0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
	0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8_calc(const uint8_t *data, uint32_t lengthByte) {
	uint8_t crc = 0;
	for (uint32_t i = 0; i < lengthByte; i++) {
		crc = crsf_crc8tab[crc ^ *data++];
	}
	return crc;
}


ElrsReceiver::ElrsReceiver(HardwareSerialIMXRT &serial)
	: m_serialRef(serial)
	  , inBuffer{} {
	m_serialRef.begin(420000);
}

ElrsReceiverRcChannelsData parseRfChannels(const uint8_t *crsfData) {
	ElrsReceiverRcChannelsData channels;

	const unsigned numOfChannels = 16;
	const unsigned srcBits = 11;
	const unsigned inputChannelMask = (1 << srcBits) - 1;

	// code from BetaFlight rx/crsf.cpp / bitpacker_unpack
	uint8_t bitsMerged = 0;
	uint32_t readValue = 0;
	unsigned readByteIndex = 0;
	for (uint8_t n = 0; n < numOfChannels; n++) {
		while (bitsMerged < srcBits) {
			uint8_t readByte = crsfData[readByteIndex++];
			readValue |= ((uint32_t) readByte) << bitsMerged;
			bitsMerged += 8;
		}
		//printf("rv=%x(%x) bm=%u\n", readValue, (readValue & inputChannelMask), bitsMerged);
		channels[n] = (readValue & inputChannelMask);
		readValue >>= srcBits;
		bitsMerged -= srcBits;
	}
	return channels;
}

void printCrsfPacket(std::array<uint8_t, 64> pkt) {
	for (auto b: pkt) {
		Serial.print(b, HEX);
		Serial.print(" ");
	}
	Serial.println("");
}

void ElrsReceiver::parseCrsfPacket(const uint8_t *buff, ElrsReceiverData &data) {
	uint8_t pktType = buff[2];
	uint8_t payloadSize = buff[1] - 1;
	const uint8_t *payloadStart = buff + 2;
	uint8_t pktCrc = *(payloadStart + payloadSize);

	uint8_t calculatedCrc = crc8_calc(payloadStart, payloadSize);
	if (pktCrc != calculatedCrc) {
		//Serial.println("Bad packet CRC");
		return;
	}
	//printCrsfPacket(buff);
	switch (pktType) {
		case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: // RC Channels
			data.rcChannelsData = parseRfChannels(buff + 3);
			// You can extract channels here
			break;
		case CRSF_FRAMETYPE_ATTITUDE: // Link statistics
			//Serial.println("Link stats received");
			break;
		default:
			//Serial.print("Unknown CRSF type: ");
			//Serial.println(pktType, HEX);
			break;
	}
}

std::optional<ElrsReceiverData> ElrsReceiver::poll(void) {
	static uint8_t rxData = 0;
	static int buffIdx = 0;
	bool pktReady = false;

	while (m_serialRef.available() > 0) {
		rxData = m_serialRef.read();
		if (buffIdx == 0 && rxData != CRSF_SYNC_BYTE) {
			continue;
		}

		inBuffer[buffIdx++] = rxData;
		// After length byte is received, wait for full packet
		if (buffIdx == 2) {
			if (inBuffer[1] > CRSF_MAX_FRAME_SIZE || rxData < 4) {
				buffIdx = 0; // invalid length, reset
			}
		}
		// Once full packet is received
		if (buffIdx >= 2 && buffIdx == inBuffer[1] + 2) {
			parseCrsfPacket(inBuffer, m_data);
			pktReady = true;
			buffIdx = 0;
		}
	}
	if (pktReady)
		return m_data;

	return std::nullopt;
}

float normalize(int value, float old_min, float old_max, float new_min, float new_max) {
	return new_min + (new_max - new_min) * ((value - old_min) / (old_max - old_min));
}

ControlState ElrsReceiver::convertElrsReceiverDataToControlData(ElrsReceiverData &rawData) {
	ControlState state = {};
	if (rawData.rcChannelsData[RC_CHNL_ARM] > 1000) {
		state.arm = true;
	} else {
		state.arm = false;
	}

	state.throttle = normalize(rawData.rcChannelsData[RC_CHNL_THROTTLE], 174, 1811, 1000, 2000);
	state.rate.roll = 0.1 * (normalize(rawData.rcChannelsData[RC_CHNL_ROLL], 174, 1811, 1000, 2000) - 1500);
	state.rate.pitch = 0.1 * (normalize(rawData.rcChannelsData[RC_CHNL_PITCH], 174, 1811, 1000, 2000) - 1500);
	state.rate.yaw = 0.15 * (normalize(rawData.rcChannelsData[RC_CHNL_YAW], 174, 1811, 1000, 2000) - 1500);
	return state;
}
