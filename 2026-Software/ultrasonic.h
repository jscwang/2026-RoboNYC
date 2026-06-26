#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "main.h"

void urm13WriteBytes(uint8_t addr, regindexTypedef regIndex, uint8_t *dataBuf, uint8_t dataLen);
bool urm13ReadBytes(uint8_t addr, regindexTypedef regIndex, uint8_t *dataBuf, uint8_t dataLen);
void urm13AddValidReading(URM13 &s, int16_t value);
float urm13GetValidMean(URM13 &s);
bool urm13IsValidDistance(int16_t dist);
int16_t urm13GetRawDistance(URM13 &s);
float urm13GetDistance(URM13 &s);
void urm13Init(URM13 &s);
float computeMedian(float arr[], int n);
float getDistLeftSmooth();
float getDistRightSmooth();
float getDistFrontSmooth();
float getDistBackSmooth();

#endif
