#include "main.h"

// 48-sensor light ring: multiplexed sensor reading, threshold calibration /
// EEPROM storage, and white-line angle helpers.

void setMuxChannel(int channel) {
  digitalWrite(selectorPins[0], (channel >> 0) & 1);
  digitalWrite(selectorPins[1], (channel >> 1) & 1);
  digitalWrite(selectorPins[2], (channel >> 2) & 1);
  digitalWrite(selectorPins[3], (channel >> 3) & 1);
}

void updateSensorReadings() {

  for (int channel = 0; channel < 16; channel++) {

    setMuxChannel(channel);

    delayMicroseconds(10);

    // Throw away first read after mux switch
    analogRead(readPins[0]);
    analogRead(readPins[1]);
    analogRead(readPins[2]);

    int val0 = analogRead(readPins[0]);
    int val1 = analogRead(readPins[1]);
    int val2 = analogRead(readPins[2]);

    // -----------------------------
    // Bank 0 (Sensors 1-16)
    // 1-8 reversed
    // 9-16 normal
    // -----------------------------

    if (channel < 8) {
      sensorReadings[7 - channel] = val0;
    } else {
      sensorReadings[channel] = val0;
    }

    // -----------------------------
    // Bank 1 (Sensors 17-32)
    // 17-24 reversed
    // 25-32 normal
    // -----------------------------

    if (channel < 8) {
      sensorReadings[16 + (7 - channel)] = val1;
    } else {
      sensorReadings[16 + channel] = val1;
    }

    // -----------------------------
    // Bank 2 (Sensors 33-48)
    // 33-40 reversed
    // 41-48 normal
    // -----------------------------

    if (channel < 8) {
      sensorReadings[32 + (7 - channel)] = val2;
    } else {
      sensorReadings[32 + channel] = val2;
    }
  }
}

int getLightDif(int currentLight, int lastLight){
  int output = 0;
  int current = currentLight;
  int target = lastLight;

  if(target<180){
    if(current>target+180){
      output = -360+current-target;
    }
    else{
      output = current-target;
    }
  }
  else{
    if(current>target-180){
      output = current-target;
    }
    else{
      output = 360-target+current;
    }
  }
  return output;
}

void calibrateLineThresh() {
  if (calibrateOn) {
    for (int i = 0; i < sensorSize; i++) {
        if (sensorReadings[i] > highestThresh[i]) {
            highestThresh[i] = sensorReadings[i];
        }
    }

    if (millis() - lastPrintTime > printIntervalMs) {
        lastPrintTime = millis();
        Serial.println("int highestThresh[sensorSize] = {");
        for (int i = 0; i < sensorSize; i++) {
            Serial.print("  ");
            Serial.print(highestThresh[i]);
            if (i < sensorSize - 1) Serial.print(",");
            if ((i + 1) % 8 == 0) Serial.println();
        }
        Serial.println("};");
    }
  }
}

void resetCalibrateThreshold() {
  for (int i = 0; i < sensorSize; i++) {
    maxSensorThres[i] = 0;
    lineThresh[i] = 0;
  }
}

void saveThresholdToEEPROM() {
  for (int i = 0; i < sensorSize; i++) {
    EEPROM.put(EEPROM_THRESH_ADDR + i * (int)sizeof(int), lineThresh[i]);
  }
}

void loadThresholdFromEEPROM() {
  for (int i = 0; i < sensorSize; i++) {
    EEPROM.get(EEPROM_THRESH_ADDR + i * (int)sizeof(int), lineThresh[i]);
  }

  Serial.println("int lineThresh[sensorSize] = {");
  for (int i = 0; i < sensorSize; i++) {
    Serial.print("  ");
    Serial.print(lineThresh[i]);
    if (i < sensorSize - 1) Serial.print(",");
    if ((i + 1) % 8 == 0) Serial.println();
  }
  Serial.println("};");
}

void handleCalibrateButton() {
  int reading = digitalRead(calibrate_button_pin);

  if (reading != lastButtonReading) {
    lastButtonChangeTime = millis();  // input changed: restart debounce timer
  }

  if (millis() - lastButtonChangeTime > buttonDebounceMs) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {  // falling edge = button just pressed
        calibrating = !calibrating;
        if (calibrating) {
          resetCalibrateThreshold();
          Serial.println("[CAL] calibration STARTED -- sweep every sensor over GREEN only");
        } else {
          saveThresholdToEEPROM();
          Serial.println("[CAL] calibration STOPPED & saved to EEPROM");
        }
      }
    }
  }

  lastButtonReading = reading;
}

void calibrateLineEEPROM() {
  updateSensorReadings();

  for (int i = 0; i < sensorSize; i++) {
    if (sensorReadings[i] > maxSensorThres[i]) maxSensorThres[i] = sensorReadings[i];

    if (maxSensorThres[i] < MIN_VALID_GREEN) {
      // Sensor never reads above ~100 over green -> likely dead/disconnected. A
      // normal threshold would let it read white forever, so pin it to ADC_MAX
      // (the max reading) so it can never trip isWhite.
      lineThresh[i] = ADC_MAX;
    } else {
      lineThresh[i] = (int)(maxSensorThres[i] * greenRatio);
    }
  }

  if (millis() - lastPrintTime > printIntervalMs) {
    lastPrintTime = millis();
    saveThresholdToEEPROM();

    Serial.println("int lineThresh[sensorSize] = {");
    for (int i = 0; i < sensorSize; i++) {
      Serial.print("  ");
      Serial.print(lineThresh[i]);
      if (i < sensorSize - 1) Serial.print(",");
      if ((i + 1) % 8 == 0) Serial.println();
    }
    Serial.println("};");
  }
}

float getWhiteAng() {
  whiteX = 0;
  whiteY = 0;
  return 0;
}

float getWhiteAngClustered() {
  numClusters = 0;
  whiteX = 0;
  whiteY = 0;
  whiteDist = 0;
  return 0;
}

void updateIsWhite0() {
  for (int i = 0; i < 8; i++) {
    whiteReadings[i] = analogRead(grayscalePins[i]);
  }

  for (int i = 0; i < 8; i++) {
    if (analogRead(grayscalePins[i]) <= lineThresh[i]) {
      isWhite[i] = true;
    } else {
      isWhite[i] = false;
    }
  }
}

void updateIsWhite() {
  for (int i = 0; i < sensorSize; i++) {
    if (sensorReadings[i] >= lineThresh[i]) {
      isWhite[i] = true;
    } else {
      isWhite[i] = false;
    }
  }
}

int whiteCount() {
  int n = 0;
  for (int i = 0; i < sensorSize; i++) {
    if (isWhite[i]) n++;
  }
  return n;
}

bool whiteDetected() {
  return whiteCount() >= MIN_WHITE_SENSORS;
}

void calibrateThresh0() {

  for (int i = 0; i < 8; i++) {
    if (whiteReadings[i] < minWhiteThresh[i]) {
      minWhiteThresh[i] = whiteReadings[i];
    }
  }
  for (int i = 0; i < 8; i++) {
    whiteThresh[i] = minWhiteThresh[i] - 50;
    Serial.print(whiteThresh[i]);
    Serial.print(", ");
    Serial.print(minWhiteThresh[i]);
    Serial.print(", ");
    Serial.println(whiteReadings[i]);
  }
  Serial.println(" ");
}

bool lineDetected() {
  return false;
}

void setupLightSensors() {
  for (int i = 0; i < selectorPinsSize; i++) {
    pinMode(selectorPins[i], OUTPUT);
    digitalWrite(selectorPins[i], LOW);
  }

  for (int i = 0; i < readPinsSize; i++) {
    pinMode(readPins[i], INPUT);
  }

  analogReadResolution(12);

  Serial.println("48 light sensor reader started");
}
