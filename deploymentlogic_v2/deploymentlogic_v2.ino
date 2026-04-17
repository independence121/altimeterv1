#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LittleFS.h>


Adafruit_BMP280 bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

#define DROGUE A0
#define MAIN A1
#define G 9.81
#define SEA_LEVEL_HPA 1008.9

int launchTime, lastTime, drogueTime;
int state;
int descending;
float initialHeight, currentHeight, maxHeight;
float realAltitude;
float x_offset, y_offset, z_offset;
float a_mag;  //acceleration magnitude

bool launch = false;
bool drogueDeployed = false;

void setup() {
  digitalWrite(DROGUE, LOW);
  digitalWrite(MAIN, LOW);

  pinMode(DROGUE, OUTPUT);
  pinMode(MAIN, OUTPUT);
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
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  Serial.print("Hello!");
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("No ADXL375 detected");
    while (1)
      ;
  }
  initialHeight = bmp.readAltitude(SEA_LEVEL_HPA);
  lastTime = 0;
  calibrate();

  if (!LittleFS.begin(true)) {
    Serial.print("mount failed");
  }
}

void loop() {
  switch (state) {
    case 0:  //pad
      //grabbing accel data until launch detected
      if (getAccelDebug() > 4 * G) {  //detected launch
        state = 1;
        launchTime = millis();
      }
      break;
    case 1:  //motor burning
      if (getAccelDebug() < 2 * G && millis() > (launchTime + 7000)) {
        state = 2;  //detected coast - motor burned out
      }
      break;
    case 2:  // coasting
      if (millis() > lastTime + 50) {
        lastTime = millis();
        if (checkDescent()) {
          deployDrogue();
          state = 3;
        }
      }
      break;
    case 3:  //drogue
      if (millis() > drogueTime + 2000) {
        digitalWrite(DROGUE, LOW);
      }
      realAltitude = bmp.readAltitude(SEA_LEVEL_HPA) - initialHeight;
      if (realAltitude < 244) {
        deployMain();
        saveAltitude();
        state = 4;
      }
      break;
    case 4:  //main and landing
      break;
  }
}
float getAccelDebug() {
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  float x = event.acceleration.x - x_offset;
  float y = event.acceleration.y - y_offset;
  float z = event.acceleration.z - z_offset;
  a_mag = sqrt(x * x + y * y + z * z);
  return a_mag;
}

bool checkDescent() {
  currentHeight = bmp.readAltitude(SEA_LEVEL_HPA);  //sea level pressure in hpa
  if (currentHeight > maxHeight) {
    maxHeight = currentHeight;
  }
  if (currentHeight < maxHeight - 2) {
    descending++;
  } else {
    descending = 0;
  }
  if (descending > 10) {
    return true;
  } else {
    return false;
  }
}

void saveAltitude() {
  maxHeight -= initialHeight;
  File f = LittleFS.open("/flightlog.txt", "w");
  if (f) {
    f.print("Max Altitude: ");
    f.println(maxHeight);
    f.close();
    Serial.println("write OK");
  } else {
    Serial.println("write FAILED");
  }
  Serial.print("Max altitude: ");
  Serial.println(maxHeight);
  return;
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
  y_offset = (sy / samples) - 9.80665;  // y should read 1g
  z_offset = sz / samples;
}

void deployMain() {
  digitalWrite(MAIN, HIGH);
  delay(2000);
  digitalWrite(MAIN, LOW);
}

void deployDrogue() {
  digitalWrite(DROGUE, HIGH);
  drogueTime = millis();
}
