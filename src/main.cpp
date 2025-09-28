#include <Arduino.h>
#include <memory>
#include "FlightController.hpp"

std::unique_ptr<algo::FlightController> flightController;

void setup() {
	Serial.begin(115200);
	flightController = std::make_unique<algo::FlightController>();

}

void loop() {
	flightController->runOnce();
}
