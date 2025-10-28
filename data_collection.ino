#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

const byte rxRadio = D3, txRadio = D4;
const byte rxInGPS = D10, txOutGPS = D11;

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
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while (1)
      ;
  }

  delay(100);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  //height = 0;
  //current_altitude = [ratio] * pressure;
  //delta_time = millis() - previous_time;
  // if (current_altitude < previous_altitude) {
  //    deploy();
  // }
  //
  displayAltitude();
  displayAcceleration();
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