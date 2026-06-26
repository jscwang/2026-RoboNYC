#include "main.h"

void printData() {

  // Serial.println(switch_on);
  // Serial.println(max_port_val);
  // Serial.println(robot_speed);


  // Head
  Serial.print(" ");
  Serial.print("Head:");
  Serial.print(processed_heading);
  Serial.print(" ");

  Serial.print(dist_front);
  Serial.print(", ");
  Serial.print(dist_left);
  Serial.print(", ");
  Serial.print(dist_right);
  Serial.print(", ");
  Serial.print(dist_back);

  // IR
  Serial.print(" ");
  Serial.print("BALL ANG:");
  Serial.print(ball_ang);
  Serial.print(" ");
  Serial.print("FRONT IR:");
  Serial.print(max_port_front);
  Serial.print(" ");
  Serial.print("BACK IR:");
  Serial.print(max_port_back);
  Serial.print(" ");
  Serial.print("IR INT:");
  Serial.print(max_port_val);
  Serial.print(" ");
  Serial.print("Left Val:");
  Serial.print(max_port_left_val);
  Serial.print(" ");
  Serial.print("Mid Val:");
  Serial.print(max_port_val);
  Serial.print(" ");
  Serial.print("Right Val:");
  Serial.print(max_port_right_val);
  Serial.print(" ");
  Serial.print("Capture Val:");
  Serial.print(ball_capture_val);

  printRawSensorReadings();

  // US
  // Serial.print(" ");
  // Serial.print("FDist:");
  // Serial.print(dist_front);
  // Serial.print(" ");

  // Serial.print("BDist:");
  // Serial.print(dist_back);
  // Serial.print(" ");

  // Serial.print("LDist:");
  // Serial.print(dist_left);
  // Serial.print(" ");

  // Serial.print("RDist:");
  // Serial.print(dist_right);
  // Serial.print(" ");

  // Serial.print("LOCX:");
  // Serial.print(loc_x);
  // Serial.print(" ");
}

void printPadded(int value) {
  if (value < 1000) Serial.print(" ");
  if (value < 100) Serial.print(" ");
  if (value < 10) Serial.print(" ");

  Serial.print(value);
  Serial.print(" ");
}

void printRawSensorArray() {

  // corresponds with actual LED label placement, not adjusted for wrong labels (what we use in actual code)

  Serial.println("int lineThresh[sensorSize] = {");

  for (int i = 0; i < sensorSize; i++) {

    Serial.print(sensorReadings[i] + 250);

    // Add comma except for last element
    if (i < sensorSize - 1) {
      Serial.print(", ");
    }

    // New line every 8 values
    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }

  Serial.println("};");
  Serial.println();
}

void printAdjustedSensorArray() {

  // Remapped so sensor 32 sits between 48 and 1 (physical order)
  // Values here are actual sensor labels (1-48), mapped to 0-indexed array internally
  

  Serial.println("int lineThresh[sensorSize] = {");

  for (int i = 0; i < sensorSize; i++) {

    Serial.print(sensorReadings[physicalOrder[i] - 1]);  // subtract 1 to convert to 0-indexed

    if (i < sensorSize - 1) {
      Serial.print(", ");
    }

    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }

  Serial.println("};");
  Serial.println();


}

void printRawSensorReadings() {

  // corresponds with actual LED label placement, not adjusted for wrong labels

  Serial.println("--------------------------------------------------");

  for (int i = 0; i < sensorSize; i++) {

    Serial.print(i + 1);
    Serial.print(": ");

    Serial.print(sensorReadings[i]);

    Serial.print(" ");

    // New line every 8 sensors
    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }

  Serial.println();
  Serial.println();
}

void printAdjustedSensorReadings() {

  Serial.println("--------------------------------------------------");

  for (int i = 0; i < sensorSize; i++) {

    Serial.print(physicalOrder[i]);          // print actual sensor label
    Serial.print(": ");

    Serial.print(sensorReadings[physicalOrder[i] - 1]);  // subtract 1 to convert to 0-indexed

    Serial.print(" ");

    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }

  Serial.println();
  Serial.println();
}

void printWhite() {

  for (int i = 0; i < sensorSize; i++) {

    // Sensor number (1–48)
    Serial.print(i + 1);
    Serial.print(": ");

    // isWhite value
    Serial.print(isWhite[i]);

    Serial.print(" ");

    // New line every 8 sensors
    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }

  Serial.println();
}

void printAdjustedWhite() {
  
  for (int i = 0; i < sensorSize; i++) {

    Serial.print(physicalOrder[i]);          // print actual sensor label
    Serial.print(": ");

    Serial.print(isWhite[physicalOrder[i] - 1]);  // subtract 1 to convert to 0-indexed

    Serial.print(" ");

    if ((i + 1) % 8 == 0) {
      Serial.println();
    }
  }

  Serial.println();
}

void printWhite0() {
  for (int i = 0; i < sensorSize; i++) {
    Serial.print(whiteReadings[i]);
    Serial.print(", ");
    Serial.println(isWhite[i]);
  }
  Serial.println(" ");
}

