#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ---------------- IMU ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ---------------- TFmini data struct ----------------
struct TFminiData {
  uint16_t distance_cm = 0;
  uint16_t strength = 0;
  bool valid = false;
};

// ---------------- Sensor data ----------------
TFminiData leftSensor;
TFminiData rightSensor;

// ---------------- Thresholds ----------------
const uint16_t ALERT_DISTANCE_CM = 100;   // obstacle threshold
const uint16_t VERY_CLOSE_CM     = 50;    // stronger alert threshold

// ---------------- TFmini frame reader ----------------
bool readTFminiFrame(HardwareSerial &port, TFminiData &data) {
  const int FRAME_SIZE = 9;

  while (port.available() >= FRAME_SIZE) {
    if (port.read() == 0x59) {
      if (port.peek() == 0x59) {
        uint8_t frame[FRAME_SIZE];
        frame[0] = 0x59;
        frame[1] = port.read();

        for (int i = 2; i < FRAME_SIZE; i++) {
          unsigned long start = millis();
          while (!port.available()) {
            if (millis() - start > 10) return false;
          }
          frame[i] = port.read();
        }

        uint16_t checksum = 0;
        for (int i = 0; i < 8; i++) checksum += frame[i];

        if ((checksum & 0xFF) == frame[8]) {
          data.distance_cm = frame[2] | (frame[3] << 8);
          data.strength = frame[4] | (frame[5] << 8);
          data.valid = true;
          return true;
        }
      }
    }
  }

  return false;
}

// ---------------- Helper functions ----------------
bool usableReading(const TFminiData &d) {
  return d.valid && d.distance_cm > 0 && d.distance_cm < 1200; // ignore 0 and absurd values
}

const char* getAlertState(bool leftOK, bool rightOK, uint16_t leftDist, uint16_t rightDist) {
  if (!leftOK && !rightOK) return "NO VALID DATA";

  bool leftNear  = leftOK  && leftDist  < ALERT_DISTANCE_CM;
  bool rightNear = rightOK && rightDist < ALERT_DISTANCE_CM;

  if (!leftNear && !rightNear) return "CLEAR";

  if (leftNear && !rightNear) {
    if (leftDist < VERY_CLOSE_CM) return "LEFT VERY CLOSE";
    return "LEFT";
  }

  if (!leftNear && rightNear) {
    if (rightDist < VERY_CLOSE_CM) return "RIGHT VERY CLOSE";
    return "RIGHT";
  }

  // both near
  if (leftDist < VERY_CLOSE_CM && rightDist < VERY_CLOSE_CM) return "BOTH VERY CLOSE";
  return "BOTH";
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);   // Left TFmini on RX1 pin 19
  Serial2.begin(115200);   // Right TFmini on RX2 pin 17

  delay(500);
  Serial.println("Sensor Aid integrated test starting...");

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check IMU wiring.");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 detected.");
  Serial.println("TFmini sensors expected on Serial1 and Serial2.");
}

void loop() {
  // Update latest LiDAR readings
  readTFminiFrame(Serial1, leftSensor);
  readTFminiFrame(Serial2, rightSensor);

  bool leftOK  = usableReading(leftSensor);
  bool rightOK = usableReading(rightSensor);

  // Read IMU Euler angles
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float heading = orientationData.orientation.x;
  float roll    = orientationData.orientation.y;
  float pitch   = orientationData.orientation.z;

  // Determine alert state
  const char* alert = getAlertState(
    leftOK,
    rightOK,
    leftSensor.distance_cm,
    rightSensor.distance_cm
  );

  // Print clean status line every 150 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 150) {
    lastPrint = millis();

    Serial.print("Left: ");
    if (leftOK) {
      Serial.print(leftSensor.distance_cm);
      Serial.print(" cm");
    } else {
      Serial.print("No data");
    }

    Serial.print(" | Right: ");
    if (rightOK) {
      Serial.print(rightSensor.distance_cm);
      Serial.print(" cm");
    } else {
      Serial.print("No data");
    }

    Serial.print(" | Heading: ");
    Serial.print(heading, 1);

    Serial.print(" | Roll: ");
    Serial.print(roll, 1);

    Serial.print(" | Pitch: ");
    Serial.print(pitch, 1);

    Serial.print(" | Alert: ");
    Serial.println(alert);
  }
}