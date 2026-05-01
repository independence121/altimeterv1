#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>
#include <RFM69.h>
#include <SPI.h>
#include <stdio.h>

// Addressing and settings
#define NODEID 2       // The ID of this node
#define NETWORKID 100  // The network ID (must be same for all nodes)
#define GATEWAYID 1    // The ID of the receiving node
#define FREQUENCY RF69_433MHZ
#define FREQUENCY_EXACT 433500000      // Match your hardware (433/868/915)
#define ENCRYPTKEY "sampleEncryptKey"  // 16 characters exactly
#define CS_PIN D10
#define IRQ_PIN D8
#define RST D2

#define DROGUE A0
#define MAIN A1
#define G 9.81
#define SEA_LEVEL_HPA 1008.9

int launchTime, lastTime;
int accelCounter, maxCounter;
int state;
int descending;
float potentialMaxHeight;
float initialHeight, currentHeight, maxHeight;
float height1, height2, height3, height4, height5;
float realAltitude;
float x_offset, y_offset, z_offset;
float a_mag;  //acceleration magnitude


char maxAltitudeStr[15];
char initialHeightStr[15];

Adafruit_BMP280 bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
RFM69 radio(CS_PIN, IRQ_PIN, true);

void setup() {
  pinMode(DROGUE, OUTPUT);
  pinMode(MAIN, OUTPUT);
  digitalWrite(DROGUE, LOW);
  digitalWrite(MAIN, LOW);

  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  delay(100);
  digitalWrite(RST, LOW);
  delay(100);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();  // Necessary for HCW models
  radio.encrypt(ENCRYPTKEY);
  radio.setFrequency(FREQUENCY_EXACT);

  transmitText("BMP280 test");
  delay(500);
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!bmp.begin()) {
    transmitText("Could not find a valid BMP280 sensor, check wiring or "
                 "try a different address!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  transmitText("BMP done...moving on to accel");
  delay(500);
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    transmitText("No ADXL375 detected");
    while (1)
      ;
  }
  transmitText("Accel done...calibrating");
  delay(500);
  accel.setDataRate(ADXL3XX_DATARATE_200_HZ);
  initialHeight = bmp.readAltitude(SEA_LEVEL_HPA);
  transmitText("Initial height found...");
  delay(500);
  lastTime = 0;
  accelCounter = 0;
  maxCounter = 0;
  potentialMaxHeight = 0;
  calibrate();
  transmitText("Ready for flight! Initial Height:");
  delay(500);
  floatToString(initialHeight, initialHeightStr);
  //transmitText(initialHeightStr);
  radio.send(GATEWAYID, initialHeightStr, strlen(initialHeightStr));
}

void loop() {
  switch (state) {
    case 0:  //pad
      //grabbing accel data until launch detected
      if (getAccel() > 6 * G) {
        accelCounter++;  //detected launch
      } else {
        accelCounter = 0;
      }
      if (accelCounter > 20) {
        state = 1;
        launchTime = millis();
        transmitText("Liftoff!");
        EEPROM.put(0, 0);
        EEPROM.put(2, 0);
      }
      break;
    case 1:  //motor burning
      if (getAccel() < 2 * G && millis() > (launchTime + 7000)) {
        state = 2;  //detected coast - motor burned out
        transmitText("Now coasting");
      }
      break;
    case 2:                                             // coasting
      currentHeight = bmp.readAltitude(SEA_LEVEL_HPA);  //sea level pressure in hpa
      if (currentHeight > maxHeight) {
        maxHeight = currentHeight;
      }
      if (millis() > lastTime + 100) {
        lastTime = millis();
        if (checkDescent()) {
          deployDrogue();
          state = 3;
        }
      }
      if (millis() > launchTime + 60000) {
        deployDrogue();
        state = 3;
      }
      break;
    case 3:  //drogue
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
float getAccel() {
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
  height5 = height4;
  height4 = height3;
  height3 = height2;
  height2 = height1;
  height1 = currentHeight;
  if (height5 > height4 && height4 > height3 && height3 > height2 && height2 > height1) {
    return true;
  } else {
    return false;
  }


  // if (currentHeight > maxHeight) {
  //   if (currentHeight > potentialMaxHeight) {
  //     potentialMaxHeight = currentHeight;
  //     maxCounter++;
  //   } else {
  //     maxCounter = 0;
  //   }
  //   if (maxCounter > 3) {
  //     maxHeight = potentialMaxHeight;
  //   }
  // }
  // if (currentHeight < maxHeight - 5) {
  //   descending++;
  // } else {
  //   descending = 0;
  // }
  // return (descending > 10);
}


void saveAltitude() {
  maxHeight -= initialHeight;
  floatToString(maxHeight, maxAltitudeStr);
  transmitText("Max altitude: ");
  transmitText(maxAltitudeStr);
  EEPROM.put(4, maxHeight);

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
  transmitText("main deployed");
  EEPROM.put(2, 1);  //signals main was deployed in addr 2
}

void deployDrogue() {
  digitalWrite(DROGUE, HIGH);
  delay(2000);
  digitalWrite(DROGUE, LOW);
  transmitText("drogue deployed");
  EEPROM.put(0, 1);  //signals drogue was deployed in addr 0
}

void floatToString(float message, char* buf) {
  //sprintf(buf, "%.3f", message);
  dtostrf(message, 6, 3, buf);
}

void transmitText(String message) {

  // Convert String to a character buffer
  char buffer[message.length() + 1];
  message.toCharArray(buffer, message.length() + 1);

  // sendWithRetry(ReceiverID, Buffer, BufferLength)
  radio.send(GATEWAYID, buffer, strlen(buffer));
}
