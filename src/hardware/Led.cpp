//
// Created by Yevhen Arteshchuk on 04.09.2025.
//

#include "hardware/Led.hpp"
#include <Arduino.h>

void Led::statusGreen(bool enable) {
	// internal led
	const int state = enable == true ? HIGH : LOW;
	pinMode(9, OUTPUT);
	digitalWrite(9, state);
}

void Led::statusRed(bool enable) {
	const int state = enable == true ? HIGH : LOW;
	pinMode(5, OUTPUT);
	digitalWrite(5, state);
}

void Led::statusArm(bool enable) {
	pinMode(13, OUTPUT);
	const int state = enable == true ? HIGH : LOW;
	digitalWrite(13, state);
}
