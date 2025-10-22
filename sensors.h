#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
Adafruit_DPS310 dps;

#define Serial1 SerialGPS
#define Serial2 SerialTrans

const byte rxRadio = D3, txRadio = D4;
const byte rxInGPS = D10, txOutGPS = D11;

void setupSens();
void printSens();

void displayGPS();
void displayAcceleration();
void displayPressure();
void displayAltitude();

