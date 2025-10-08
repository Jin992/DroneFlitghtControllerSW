#include <Arduino.h>
#include <memory>
#include "FlightController.hpp"

std::unique_ptr<fc::FlightController> flightController;

void setup() {
	Serial.begin(115200);
	flightController = std::make_unique<fc::FlightController>();
}

void loop() {
	flightController->runOnce();
}
