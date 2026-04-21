#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LittleFS.h>
#include <RFM69.h>
#include <SPI.h>
#include <stdio.h>

// Addressing and settings
#define NODEID 1       // The ID of this node
#define NETWORKID 100  // The network ID (must be same for all nodes)
#define GATEWAYID 2    // The ID of the receiving node
#define FREQUENCY RF69_433MHZ
#define FREQUENCY_EXACT 433000000      // Match your hardware (433/868/915)
#define ENCRYPTKEY "sampleEncryptKey"  // 16 characters exactly
#define CS_PIN D6
#define IRQ_PIN D9
#define RST D4

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
  delay(1000);
  digitalWrite(RST, LOW);
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();  // Necessary for HCW models
  radio.encrypt(ENCRYPTKEY);
  radio.setFrequency(FREQUENCY_EXACT);

  transmitText("BMP280 test");
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    transmitText(F("Could not find a valid BMP280 sensor, check wiring or "
                   "try a different address!"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    transmitText("No ADXL375 detected");
    while (1)
      ;
  }
  accel.setDataRate(ADXL3XX_DATARATE_200_HZ);
  initialHeight = bmp.readAltitude(SEA_LEVEL_HPA);
  lastTime = 0;
  accelCounter = 0;
  maxCounter = 0;
  potentialMaxHeight = 0;
  calibrate();

  if (!LittleFS.begin(true)) {
    transmitText("mount failed");
  }
  transmitText("Ready for flight! Initial Height:");
  floatToString(initialHeight, initialHeightStr);
  transmitText(initialHeightStr);
}

void loop() {
  switch (state) {
    case 0:  //pad
      //grabbing accel data until launch detected
      if (getAccel() > 10 * G) {
        accelCounter++;  //detected launch
      } else {
        accelCounter = 0;
      }
      if (accelCounter > 20) {
        state = 1;
        launchTime = millis();
        transmitText("Liftoff!");
      }
      break;
    case 1:  //motor burning
      if (getAccel() < 2 * G && millis() > (launchTime + 7000)) {
        state = 2;  //detected coast - motor burned out
        transmitText("Now coasting");
      }
      break;
    case 2:  // coasting
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
  currentHeight = bmp.readAltitude(SEA_LEVEL_HPA);  //sea level pressure in hpa
  if (currentHeight > maxHeight) {
    maxHeight = currentHeight;
  }
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
  File f = LittleFS.open("/flightlog.txt", "w");
  if (f) {
    f.print("Max Altitude: ");
    f.println(maxHeight);
    f.close();
    transmitText("write OK");
  } else {
    transmitText("write FAILED");
  }
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
  transmitText("main deployed");
  delay(2000);
  digitalWrite(MAIN, LOW);
}

void deployDrogue() {
  digitalWrite(DROGUE, HIGH);
  delay(2000);
  digitalWrite(DROGUE, LOW);
  transmitText("drogue deployed");
}

void floatToString(float message, char* buf) {
  dtostrf(message, 15, 3, buf);
}

void transmitText(String message) {

  // Convert String to a character buffer
  char buffer[message.length() + 1];
  message.toCharArray(buffer, message.length() + 1);

  // sendWithRetry(ReceiverID, Buffer, BufferLength)
  radio.send(GATEWAYID, buffer, strlen(buffer));
}
