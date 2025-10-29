#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

int lastTime;

void setup(void) {
  Serial.begin(9600);
  //serial test
  while (!Serial) {
    if (millis() > 500) {
      break;
    }
  }
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("No ADXL375 detected");
    while (1)
      ;
  }
  if (!bmp.begin()) {
    Serial.println(F("No BMP280 detected"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  if (millis() > lastTime + 500) {
    displayAltitude();
    displayAcceleration();
    lastTime = millis();
  }
}

void displayAcceleration() {
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(event.acceleration.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(event.acceleration.z);
  Serial.print("  ");
  Serial.println("m/s^2 ");
}

void displayAltitude() {
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");
}
