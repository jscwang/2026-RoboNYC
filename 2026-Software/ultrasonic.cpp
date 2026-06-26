#include "main.h"

void urm13WriteBytes(uint8_t addr, regindexTypedef regIndex, uint8_t *dataBuf, uint8_t dataLen) {
  Wire.beginTransmission(addr);
  Wire.write(regIndex);
  for (uint8_t i = 0; i < dataLen; i++) {
    Wire.write(dataBuf[i]);
  }
  Wire.endTransmission();
}

bool urm13ReadBytes(uint8_t addr, regindexTypedef regIndex, uint8_t *dataBuf, uint8_t dataLen) {
  Wire.beginTransmission(addr);
  Wire.write(regIndex);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(addr, dataLen);
  if (received != dataLen) return false;
  for (uint8_t i = 0; i < dataLen; i++) dataBuf[i] = Wire.read();
  return true;
}

void urm13AddValidReading(URM13 &s, int16_t value) {
  s.validWindow[s.windowIndex] = value;
  s.windowIndex = (s.windowIndex + 1) % URM13_WINDOW_SIZE;
  if (s.windowCount < URM13_WINDOW_SIZE) s.windowCount++;
}

float urm13GetValidMean(URM13 &s) {
  if (s.windowCount == 0) return -1.0;
  long sum = 0;
  for (uint8_t i = 0; i < s.windowCount; i++) sum += s.validWindow[i];
  return (float)sum / s.windowCount;
}

bool urm13IsValidDistance(int16_t dist) {
  if (dist == -1) return false;
  if (dist < 0) return false;
  if (dist > URM13_MAX_VALID_DISTANCE) return false;
  return true;
}

int16_t urm13GetRawDistance(URM13 &s) {
  uint8_t buf[2];
  if (!urm13ReadBytes(s.addr, eDistanceH, buf, 2)) return -1;
  return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

float urm13GetDistance(URM13 &s) {
  s.rawDist = urm13GetRawDistance(s);
  float meanDist = urm13GetValidMean(s);
  float usedDist = -1.0;
  if (urm13IsValidDistance(s.rawDist)) {
    urm13AddValidReading(s, s.rawDist);
    usedDist = urm13GetValidMean(s);
  } else if (s.windowCount > 0) {
    usedDist = meanDist;
  }
  if (usedDist >= 0 && usedDist < 20 && !urm13IsValidDistance(s.rawDist)) {
    usedDist = usedDist * 0.7;
  }
  return usedDist;
}

void urm13Init(URM13 &s) {
  uint8_t cfg = 0;
  cfg |= MEASURE_RANGE_BIT;  // short-range mode
  cfg &= ~MEASURE_MODE_BIT;  // automatic I2C mode
  cfg |= TEMP_CPT_ENABLE_BIT;
  cfg &= ~TEMP_CPT_SEL_BIT;
  urm13WriteBytes(s.addr, eConfig, &cfg, 1);
  delay(100);
}

float computeMedian(float arr[], int n) {
  float temp[WINDOW_SIZE];
  memcpy(temp, arr, n * sizeof(float));
  // Simple insertion sort (small n = fine)
  for (int i = 1; i < n; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }
  // Median
  if (n % 2 == 0) return (temp[n / 2 - 1] + temp[n / 2]) / 2.0;
  else return temp[n / 2];
}

float getDistLeftSmooth() {
  distLeftBuffer[leftIndex] = dist_left;
  leftIndex = (leftIndex + 1) % WINDOW_SIZE;
  if (leftCount < WINDOW_SIZE) leftCount++;
  return computeMedian(distLeftBuffer, leftCount);
}

float getDistRightSmooth() {
  distRightBuffer[rightIndex] = dist_right;
  rightIndex = (rightIndex + 1) % WINDOW_SIZE;
  if (rightCount < WINDOW_SIZE) rightCount++;
  return computeMedian(distRightBuffer, rightCount);
}

float getDistFrontSmooth() {
  distFrontBuffer[frontIndex] = dist_front;
  frontIndex = (frontIndex + 1) % WINDOW_SIZE;
  if (frontCount < WINDOW_SIZE) frontCount++;
  return computeMedian(distFrontBuffer, frontCount);
}

float getDistBackSmooth() {
  distBackBuffer[backIndex] = dist_back;
  backIndex = (backIndex + 1) % WINDOW_SIZE;
  if (backCount < WINDOW_SIZE) backCount++;
  return computeMedian(distBackBuffer, backCount);
}

