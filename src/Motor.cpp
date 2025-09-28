//
// Created by Yevhen Arteshchuk on 04.09.2025.
//

#include "Motor.hpp"
#include <core_pins.h>
#include <Arduino.h>
#include "Rate.hpp"

Motor::Motor(int controlPin)
	: m_controlPin(controlPin) {
	m_configurePinToPwmMode();
}

Motor::Motor(Motor &&obj): m_controlPin(obj.m_controlPin) {
	obj.m_controlPin = 255;
}

Motor &Motor::operator=(Motor &&other) {
	m_controlPin = std::move(other.m_controlPin);
	return *this;
}

void Motor::setPwm(float value) {
	float frequency = PWM_12_BIT_MULTIPLIER * value;
	analogWrite(m_controlPin, frequency);
}

void Motor::m_configurePinToPwmMode() {
	//Serial.printf("Motor set PWM mode\n");
	analogWriteFrequency(m_controlPin, DEFAULT_PWM_FREQUENCY);
}

float Motor::m_convertPwmResolutionTo12Bit(int pwm) {
	return 0.0;
}
