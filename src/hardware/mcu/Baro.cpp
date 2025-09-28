//
// Created by Yevhen Arteshchuk on 27.09.2025.
//

#include "hardware/mcu/Baro.hpp"

namespace imu {

	SensorCalibrationValues config;

	Baro::Baro(TwoWire &wire)
	: m_wire(wire)
	, m_altitudeBaroStartUp(0) {
		m_init();
	}

	float Baro::measure() {
		return m_measureAltitude() - m_altitudeBaroStartUp;
	}

	float Baro::groundAltitude() {
		return m_groundAltidute;
	}

	void Baro::m_init() {
		m_wire.beginTransmission(0x76);
		m_wire.write(0xF4);
		m_wire.write(0x57);
		m_wire.endTransmission();

		m_wire.beginTransmission(0x76);
		m_wire.write(0xF5);
		m_wire.write(0x14);
		m_wire.endTransmission();
		uint8_t data[24] = {0};
		int i = 0;

		Wire.beginTransmission(0x76);
		Wire.write(0x88);
		Wire.endTransmission();
		Wire.requestFrom(0x76,24);

		while (Wire.available()) {
			data[i] = Wire.read();
			i++;
		}
		config.dig_T1 = (data[1] << 8 | data[0]);
		config.dig_T2 = (data[3] << 8 | data[2]);
		config.dig_T3 = (data[5] << 8 | data[4]);
		config.dig_P1 = (data[7] << 8 | data[6]);
		config.dig_P2 = (data[9] << 8 | data[8]);
		config.dig_P3 = (data[11] << 8 | data[10]);
		config.dig_P4 = (data[13] << 8 | data[12]);
		config.dig_P5 = (data[15] << 8 | data[14]);
		config.dig_P6 = (data[17] << 8 | data[16]);
		config.dig_P7 = (data[19] << 8 | data[18]);
		config.dig_P8 = (data[21] << 8 | data[20]);
		config.dig_P9 = (data[23] << 8 | data[22]);
		delay(250);

		m_calibrate();
		m_groundAltidute = measure();

	}

	void Baro::m_calibrate() {
		for (int i = 0; i < 2000; i++) {
			m_altitudeBaroStartUp += m_measureAltitude();
		}
		m_altitudeBaroStartUp /= 2000;
	}

	float Baro::m_measureAltitude() {
		m_wire.beginTransmission(0x76);
		m_wire.write(0xF7);
		m_wire.endTransmission();
		m_wire.requestFrom(0x76, 6);

		const uint32_t pressure_msb = m_wire.read();
		const uint32_t pressure_lsb = m_wire.read();
		const uint32_t pressure_xlsb = m_wire.read();
		const uint32_t temperature_msb = m_wire.read();
		const uint32_t temperature_lsb = m_wire.read();
		const uint32_t temperature_xlsb = m_wire.read();


		unsigned long int adc_P = (pressure_msb << 12) | (pressure_lsb << 4) | (pressure_xlsb >> 4);
		unsigned long int adc_T = (temperature_msb << 12) | (temperature_lsb << 4) | (temperature_xlsb >> 4);

		signed long int var1 = ((((adc_T >> 3) - ((signed long int)config.dig_T1 <<1)))* ((signed long int)config.dig_T2)) >> 11;
		signed long int  var2 = (((((adc_T >> 4) - ((signed long int)config.dig_T1)) * ((adc_T>>4) - ((signed long int)config.dig_T1)))>> 12) * ((signed long int)config.dig_T3)) >> 14;
		signed long int t_fine = var1 + var2;

		unsigned long int p;
		var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
		var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)config.dig_P6);
		var2 = var2 + ((var1*((signed long int)config.dig_P5)) <<1);
		var2 = (var2>>2)+(((signed long int)config.dig_P4)<<16);
		var1 = (((config.dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >> 3)+((((signed long int)config.dig_P2) * var1) >> 1)) >> 18;
		var1 = ((((32768+var1))*((signed long int)config.dig_P1)) >> 15);

		if (var1 == 0) { p=0;}
		p = (((unsigned long int)(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
		if(p<0x80000000){ p = (p << 1) / ((unsigned long int) var1);}
		else { p = (p / (unsigned long int)var1) * 2;  }
		var1 = (((signed long int )config.dig_P9) * ((signed long int) (((p>>3) * (p>>3))>>13)))>>12;
		var2 = (((signed long int )(p>>2)) * ((signed long int)config.dig_P8))>>13;
		p = (unsigned long int)((signed long int)p + ((var1 + var2+ config.dig_P7) >> 4));
		double pressure=(double)p/100;
		return 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) * 100;
	}
} // imu