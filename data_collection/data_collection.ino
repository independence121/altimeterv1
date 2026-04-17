#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

Adafruit_BMP280 bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
float x_offset, y_offset, z_offset;

int lastTime;
float totalAccel;
float currentHeight;
float height0, height1, height2, height3, height4, height5;
bool descending = false;
int descentCounter;
int avg_new, avg_old;


void setup() {
  delay(1000);
  Serial.begin(115200);
  //serial test
  while (!Serial) delay(100);  // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  Serial.print("Hello!");
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("No ADXL375 detected");
    while (1)
      ;
  }
  calibrate();
}

void loop() {
  // if (millis() > lastTime + 500) {
  //   displayAltitude();
  //   displayAcceleration();
  //   lastTime = millis();
  // }
  if (millis() > lastTime + 100) {
    currentHeight = bmp.readAltitude(1008.90);  //sea level pressure in hpa
    height5 = height4;
    height4 = height3;
    height3 = height2;
    height2 = height1;
    height1 = height0;
    height0 = currentHeight;
    if (height5 > height4 && height4 > height3 && height3 > height2 && height2 > height1) descending = true;
    else descending = false;
    Serial.println(descending);
    lastTime = millis();
  }
}

void displayAcceleration() {
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  float x = event.acceleration.x - x_offset;
  float y = event.acceleration.y - y_offset;
  float z = event.acceleration.z - z_offset;
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(-1 * y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(z);
  Serial.print("  ");
  Serial.println("m/s^2 ");
  totalAccel = sqrt(sq(x) + sq(y) + sq(z));
  Serial.println(totalAccel);
}

void displayAltitude() {
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1008.9)); /* Adjusted to local forecast! */
  Serial.println(" m");
}


void calibrate() {
  float sx = 0, sy = 0, sz = 0;
  int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sx += event.acceleration.x;
    sy += event.acceleration.y;
    sz += event.acceleration.z;
    delay(10);
  }
  x_offset = sx / samples;
  y_offset = sy / samples;
  z_offset = (sz / samples) - 9.80665;  // Z should read 1g
}
