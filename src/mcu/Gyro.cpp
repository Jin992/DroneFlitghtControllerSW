//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "Gyro.hpp"

namespace imu {

#define GYRO_CALIBRATION_MEASUREMENTS_CNT 2000

	// float RateRoll, RatePitch, RateYaw;
	// float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
	// int RateCalibrationNumber;

	// void gyro_signals(void) {
	//   Wire.beginTransmission(0x68);
	//   Wire.write(0x1A);
	//   Wire.write(0x05);
	//   Wire.endTransmission();
	//   Wire.beginTransmission(0x68);
	//   Wire.write(0x1C);
	//   Wire.write(0x10);
	//   Wire.endTransmission();
	//   Wire.beginTransmission(0x68);
	//   Wire.write(0x3B);
	//   Wire.endTransmission();
	//   Wire.requestFrom(0x68,6);
	//   int16_t AccXLSB = Wire.read() << 8 | Wire.read();
	//   int16_t AccYLSB = Wire.read() << 8 | Wire.read();
	//   int16_t AccZLSB = Wire.read() << 8 | Wire.read();
	//   Wire.beginTransmission(0x68);
	//   Wire.write(0x1B);
	//   Wire.write(0x8);
	//   Wire.endTransmission();
	//   Wire.beginTransmission(0x68);
	//   Wire.write(0x43);
	//   Wire.endTransmission();
	//   Wire.requestFrom(0x68,6);
	//   int16_t GyroX=Wire.read()<<8 | Wire.read();
	//   int16_t GyroY=Wire.read()<<8 | Wire.read();
	//   int16_t GyroZ=Wire.read()<<8 | Wire.read();
	//   // Serial.printf("Rate Roll: %d Rate Pitch: %d Rate Yaw: %d\n", GyroX, GyroY, GyroZ);
	//   RateRoll=(float)GyroX/65.5;
	//   RatePitch=(float)GyroY/65.5;
	//   RateYaw=(float)GyroZ/65.5;
	//   AccX=(float)AccXLSB/4096;
	//   AccY=(float)AccYLSB/4096;
	//   AccZ=(float)AccZLSB/4096;
	//   AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
	//   AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
	// }
	Gyro::Gyro(TwoWire &wire)
	: m_wire(wire)
	, m_calibrationAdjust{0.0,0.0,0.0}
	{
		m_init();
		m_calibrate();
	}

	void Gyro::m_init() {
		// Shared part for accelerometr and gyroscope, should be moved to IMU
		Serial.println("Starting IMU initialization.");
		m_wire.setClock(400000);
		m_wire.begin();
		delay(250);
		m_wire.beginTransmission(0x68);
		m_wire.write(0x6B);
		m_wire.write(0x00);
		m_wire.endTransmission();
	}

	GyroData Gyro::m_pollGyro() {
		m_wire.beginTransmission(0x68);
		m_wire.write(0x1A);
		m_wire.write(0x05);
		m_wire.endTransmission();

		m_wire.beginTransmission(0x68);
		m_wire.write(0x1B);
		m_wire.write(0x8);
		m_wire.endTransmission();

		m_wire.beginTransmission(0x68);
		m_wire.write(0x43);
		m_wire.endTransmission();

		m_wire.requestFrom(0x68,6);
		const int16_t GyroX=Wire.read()<<8 | Wire.read();
		const int16_t GyroY=Wire.read()<<8 | Wire.read();
		const int16_t GyroZ=Wire.read()<<8 | Wire.read();
		const float angleConst = 65.5;
		return {static_cast<float>(GyroX) / angleConst,
			  static_cast<float>(GyroY) / angleConst,
			   static_cast<float>(GyroZ) / angleConst
		};
	}

	GyroData Gyro::measure() {
		return m_pollGyro() - m_calibrationAdjust;
	}

	void Gyro::m_calibrate() {
		Serial.println("Initializing gyroscope calibration.");
		for (int16_t cnt = 0; cnt < GYRO_CALIBRATION_MEASUREMENTS_CNT; cnt++) {
			m_calibrationAdjust += m_pollGyro();
			delay(1);
		}
		m_calibrationAdjust /= GYRO_CALIBRATION_MEASUREMENTS_CNT;
		Serial.println("Gyroscope calibration done.");
		Serial.printf("Gyro error: Roll %f Pitch %f Yaw %f\n", m_calibrationAdjust.roll, m_calibrationAdjust.pitch, m_calibrationAdjust.yaw);
	}
} // imu