#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_DPS310.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
Adafruit_DPS310 dps;

const byte rxRadio = D3, txRadio = D4;
const byte rxInGPS = D10, txOutGPS = D11;

void setup(void) {
  Serial.begin(57600);
  Serial1.begin(115200, SERIAL_8N1, rxInGPS, txOutGPS);  // GPS
  Serial2.begin(57600, SERIAL_8N1, rxRadio, txRadio);    // Transceiver
  //serial test
  Serial2.println("HI");
  while (!Serial) {
    if (millis() > 500) {
      break;
    }
  }
  while (!Serial1) {
    if (millis() > 3000) {
      Serial.println("Error with Radio");
    }
  }
  while (!Serial2) {
    if (millis() > 3000) {
      Serial.println("Error with GPS");
    }
  }
  Serial.println("Hello");
  if (!mpu.begin(0x68)) {
    Serial2.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  if (!dps.begin_I2C(0x77)) {  // Can pass in I2C address here
    Serial2.println("Failed to find DPS");
    while (1) yield();
  }

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  delay(100);
}

void loop() {
  //height = 0;
  //current_altitude = [ratio] * pressure;
  //delta_time = millis() - previous_time;
  // if (current_altitude < previous_altitude) {
  //    deploy();
  // }
  //
  displayGPS();
  displayPressure();
  displayAcceleration();
}

void displayGPS() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  Serial2.print("Sats: ");
  Serial2.println(gps.satellites.value());

  Serial2.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial2.print(gps.location.lat(), 6);
    Serial2.print(F(","));
    Serial2.println(gps.location.lng(), 6);
  } else {
    Serial2.println(F("INVALID"));
  }
}

void displayAcceleration() {
  sensors_event_t a, g, temp;  // get data
  mpu.getEvent(&a, &g, &temp);

  Serial2.print("Accel.X: ");  // print acceleration in x-direction
  Serial2.print(a.acceleration.x);
  Serial2.print(";\t");

  Serial2.print("Accel.Y: ");  // print acceleration in y-direction
  Serial2.print(a.acceleration.y);
  Serial2.print(";\t");

  Serial2.print("Accel.Z: ");  // print acceleration in z-direction
  Serial2.print(a.acceleration.z);
  Serial2.println(";");
  Serial2.println();
}

void displayPressure() {
  sensors_event_t temp_event, pressure_event;

  dps.getEvents(&temp_event, &pressure_event);
  Serial2.print(F("Temperature: "));
  Serial2.print(temp_event.temperature);
  Serial2.println("\t");

  Serial2.print(F("Pressure: "));
  Serial2.print(pressure_event.pressure);
  Serial2.println("\t");
}

// void displayAltitude() {
//   senosrs_event_t pressure_event;

//   dps.getEvents(&pressure_event);
//   pressuretoAltitude();
//   Serial2.print("Altitude: ");
//   Serial2.print(dps.pressureToAltitude(pressure));
// }