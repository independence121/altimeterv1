#include <Adafruit_ADXL375.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <EEPROM.h>
#include <RFM69.h>
#include <SPI.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


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

#define SAMPLE_RATE_HZ 100
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)
#define BUFFER_SIZE 50
#define DESCENT_THRESHOLD_M 2.0
#define DESCENT_SAMPLES 5
#define GROUND_LEVEL_SAMPLES 100


int launchTime, lastTime;
int accelCounter;
int state;
float x_offset, y_offset, z_offset;
float a_mag;  //acceleration magnitude
float currentAltitude, initialHeight;

/* ============================================================================
 * MovingAverageBuffer Class
 * Handles smoothing of altitude readings
 * ============================================================================ */
 
class MovingAverageBuffer {
private:
    float readings[BUFFER_SIZE];
    uint32_t index;
    uint32_t count;
    float sum;
 
public:
    MovingAverageBuffer() : index(0), count(0), sum(0.0f) {
        for (int i = 0; i < BUFFER_SIZE; i++) {
            readings[i] = 0.0f;
        }
    }
    
    float add(float altitude) {
        // Remove oldest value if buffer is full
        if (count == BUFFER_SIZE) {
            sum -= readings[index];
        } else {
            count++;
        }
        
        // Add new value
        readings[index] = altitude;
        sum += altitude;
        index = (index + 1) % BUFFER_SIZE;
        
        // Return average
        return sum / count;
    }
    
    void reset() {
        index = 0;
        count = 0;
        sum = 0.0f;
    }
};


/* ============================================================================
 * ApogeeDetector Class
 * Main apogee detection logic
 * ============================================================================ */

class ApogeeDetector {
private:
  float groundLevel_m;
  float maxAltitude_m;
  float apogee_m;
  bool apogeeDetected;
  uint32_t apogeeTimestamp_ms;
  float descentCount;
  float prevFilteredAltitude;
  MovingAverageBuffer altitudeBuffer;
  uint32_t calibrationSamples;

  /**
     * Calculate vertical velocity from altitude change
     * @return vertical velocity in m/s (positive = ascending, negative = descending)
     */
  float calculateVelocity(float currentAltitude, float prevAltitude) {
    return (currentAltitude - prevAltitude) / (SAMPLE_INTERVAL_MS / 1000.0f);
  }

public:
  /**
     * Constructor - Initialize the detector
     */
  ApogeeDetector()
    : groundLevel_m(0.0f),
      maxAltitude_m(0.0f),
      apogee_m(0.0f),
      apogeeDetected(false),
      apogeeTimestamp_ms(0),
      descentCount(0),
      prevFilteredAltitude(0.0f),
      calibrationSamples(0) {
    //printf("[APOGEE] Detector initialized\n");
  }

  /**
     * Calibrate ground level by averaging multiple readings
     * Call this before launch while rocket is on the pad
     * 
     * @param currentHeight current altitude reading from sensor
     * @return true when calibration is complete
     */
  bool calibrateGroundLevel(float currentHeight) {
    if (calibrationSamples < GROUND_LEVEL_SAMPLES) {
      groundLevel_m += currentHeight;
      calibrationSamples++;

      if (calibrationSamples == GROUND_LEVEL_SAMPLES) {
        groundLevel_m /= GROUND_LEVEL_SAMPLES;
        prevFilteredAltitude = groundLevel_m;
        //printf("[APOGEE] Ground level calibrated: %.2f m\n", groundLevel_m);
        return true;
      }
      return false;
    }
    return true;
  }

  /**
     * Check if detector is calibrated
     * @return true if ground level calibration is complete
     */
  bool isCalibrated() const {
    return calibrationSamples >= GROUND_LEVEL_SAMPLES;
  }

  /**
     * Main apogee detection update function
     * Call this repeatedly at SAMPLE_RATE_HZ frequency
     * 
     * @param rawAltitude raw altitude from sensor
     * @param timestamp_ms current time in milliseconds
     * @return true if apogee has been detected
     */
  bool update(float rawAltitude) {
    // Skip if not calibrated yet
    if (!isCalibrated()) {
      return false;
    }

    // Altitude relative to ground level
    float relativeAltitude = rawAltitude - groundLevel_m;

    // Apply moving average filter to reduce noise
    float filteredAltitude = altitudeBuffer.add(relativeAltitude);

    // Calculate vertical velocity
    float velocity = calculateVelocity(filteredAltitude, prevFilteredAltitude);

    // Track maximum altitude
    if (filteredAltitude > maxAltitude_m) {
      maxAltitude_m = filteredAltitude;
      descentCount = 0;  // Reset descent counter on new max
    }

    // ====================================================================
    // APOGEE DETECTION LOGIC
    // ====================================================================

    if (!apogeeDetected) {
      // Check if altitude is decreasing
      if (velocity < -DESCENT_THRESHOLD_M) {
        descentCount++;

        // Confirm apogee after N consecutive descent samples
        if (descentCount >= DESCENT_SAMPLES) {
          apogeeDetected = true;
          apogee_m = maxAltitude_m;

          // printf("\n");
          // printf("╔══════════════════════════════════════╗\n");
          // printf("║         APOGEE DETECTED!             ║\n");
          // printf("╚══════════════════════════════════════╝\n");
          // printf("[APOGEE] Maximum altitude: %.2f m\n", apogee_m);
          // printf("[APOGEE] Time to apogee: %.2f s\n",
          //        apogeeTimestamp_ms / 1000.0f);
          // printf("[APOGEE] Current velocity: %.2f m/s\n", velocity);
          // printf("[APOGEE] Descent confirmed after %d samples\n",
          //        DESCENT_SAMPLES);
          // printf("\n");

          return true;
        }
      } else {
        // Still ascending or descending slowly, reset counter
        descentCount = 0;
      }
    }

    // Update previous altitude for next iteration
    prevFilteredAltitude = filteredAltitude;

    return apogeeDetected;
  }

  // ========================================================================
  // Getter Methods
  // ========================================================================

  bool isApogeeDetected() const {
    return apogeeDetected;
  }

  float getInitialAltitude() const {
    return groundLevel_m;
  }

  float getApogee() const {
    return apogee_m;
  }

  float getMaxAltitude() const {
    return maxAltitude_m;
  }

  float getCurrentAltitude() const {
    return prevFilteredAltitude;
  }

  uint32_t getApogeeTime() const {
    return apogeeTimestamp_ms;
  }

  uint32_t getCalibrationProgress() const {
    return calibrationSamples;
  }

  // Reset for another flight
  void reset() {
    groundLevel_m = 0.0f;
    maxAltitude_m = 0.0f;
    apogee_m = 0.0f;
    apogeeDetected = false;
    apogeeTimestamp_ms = 0;
    descentCount = 0;
    prevFilteredAltitude = 0.0f;
    calibrationSamples = 0;
    altitudeBuffer.reset();
  }
};



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

  transmitText("BMP388 test");
  delay(500);
  if (!bmp.begin_I2C()) {
    transmitText("Could not find a valid BMP388 sensor, check wiring or "
                 "try a different address!");
    while (1) delay(10);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);  // 200 Hz for responsive apogee detection

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
  for (int i = 0, i < 10; i++) {
    bmp.readAltitude(SEA_LEVEL_HPA); // clearing bad initial readings
  }
  for (int i = 0; i < GROUND_LEVEL_SAMPLES; i++) {
    float height = bmp.readAltitude(SEA_LEVEL_HPA);
    detector.calibrateGroundLevel(height);
  }
  initialHeight = detector.getInitialAltitude();
  transmitText("Initial height found...");
  delay(500);
  lastTime = 0;
  accelCounter = 0;
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
    case 2:  // coasting (apogee detection)

      // Main acquisition loop
      if (millis - lastSampleTime >= SAMPLE_TIME_MS) {
        bool apogee_found = detector.update(bmp.readAltitude(SEA_LEVEL_HPA));
        //timestamp_ms += SAMPLE_INTERVAL_MS;

        if (apogee_found) {
          deployDrogue();
          state = 3;
        }
        lastSampleTime = millis();
      }
      break;
    case 3:  //drogue
      if (millis - lastSampleTime >= SAMPLE_TIME_MS) {
        detector.update(bmp.readAltitude(SEA_LEVEL_HPA));
        currentAltitude = detector.getCurrentAltitude();

        // realAltitude = bmp.readAltitude(SEA_LEVEL_HPA) - initialHeight;
        if (currentAltitude < 244) {
          deployMain();
          saveAltitude();
          state = 4;
        }
        lastSampleTime = millis();
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

float getVerticalAccel() {
  sensors_event_t event;
  accel.getEvent(&event);
  float y = event.acceleration.y - y_offset;
  return y;
}


void saveAltitude() {
  maxHeight = detector.getMaxAltitude();
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
  delay(1000);
  digitalWrite(MAIN, LOW);
  transmitText("main deployed");
  EEPROM.put(2, 1);  //signals main was deployed in addr 2
}

void deployDrogue() {
  digitalWrite(DROGUE, HIGH);
  delay(1000);
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
