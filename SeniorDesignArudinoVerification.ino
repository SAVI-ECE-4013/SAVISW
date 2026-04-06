void setup() {
  Serial.begin(115200);      // USB serial monitor
  Serial1.begin(115200);     // TFmini UART

  Serial.println("TFmini test starting...");
}

void loop() {
  if (Serial1.available() >= 9) {
    
    if (Serial1.read() == 0x59) {
      if (Serial1.read() == 0x59) {

        uint8_t data[7];
        for (int i = 0; i < 7; i++) {
          data[i] = Serial1.read();
        }

        uint16_t distance = data[0] | (data[1] << 8);
        uint16_t strength = data[2] | (data[3] << 8);

        // Verify checksum
        uint8_t sum = 0x59 + 0x59;
        for (int i = 0; i < 6; i++) {
          sum += data[i];
        }

        if ((sum & 0xFF) == data[6]) {
          Serial.print("Distance (cm): ");
          Serial.print(distance);
          Serial.print(" | Strength: ");
          Serial.println(strength);
        } else {
          Serial.println("Checksum error");
        }
      }
    }
  }
}