#include "main.h"

void updateCamera() {
  if (Serial8.available() >= 7) {
    byte header = Serial8.read();
    if (header == 0x99) {
      byte actual_sum = 0;
      for (int i = 0; i < 5; i++) {
        temp_data[i] = Serial8.read();
        actual_sum += temp_data[i];
      }
      byte desired_sum = Serial8.read();
      if (actual_sum % 256 == desired_sum) {
        for (int i = 0; i < 5; i++) {
          camera_data[i] = temp_data[i];  // * 2;
          // Serial.print(camera_data[i]); Serial.print(" ");
        }
        // Serial.println("");
      } else {
        Serial.println("Camera data sum error!");
      }
    }
  }

  yellow_ang = (camera_data[0] * 2) % 360;
  blue_ang = (camera_data[2] * 2) % 360;
  // Serial.print(yellow_ang); Serial.print(", "); Serial.println(blue_ang);
  yellow_detected = camera_data[1];
  blue_detected = camera_data[3];
  ball_captured = camera_data[4];
  // Serial.println(yellow_ang);
  // Serial.print(blue_ang); Serial.print(", ");
  // Serial.println(ball_captured);
}

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);  // send stop
}

void setupCompass() {
  Serial.println("Setting compass type to BNO055");

  if (!bno.begin()) {
    Serial.println("Error: Failed to initialize BNO055");
    while (1)
      ;  // halt
  }

  bno.setExtCrystalUse(true);
  sensor_t sensor;
  bno.getSensor(&sensor);

  // Set to NDOF mode
  bno.setMode(0x0C);

  // Configure BNO registers
  bno_write(BNO_ADDR, PAGE_ID, 1);        // Register page 1
  bno_write(BNO_ADDR, ACC_CONFIG, 0x0D);  // Accel 2g range
  bno_write(BNO_ADDR, PAGE_ID, 0);        // Back to page 0
  bno_write(BNO_ADDR, OPR_MODE, 0x0C);    // Operating mode

  delay(10);  // Allow time for BNO to switch modes
}

