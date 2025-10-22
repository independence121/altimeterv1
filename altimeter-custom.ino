#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

#include <sensors.h>

void setup(void) {
	Serial.begin(57600);
	setupSens();
	// why?
	delay(100);
}

void loop() {
	//height = 0;
	//current_altitude = [ratio] * pressure;
	//delta_time = millis() - previous_time;
	// if (current_altitude < previous_altitude) {
	//		deploy();
	// }
	//
	printSens();
}

