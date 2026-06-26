#ifndef LIGHTSENSORS_H
#define LIGHTSENSORS_H
#include "main.h"

void setMuxChannel(int channel);
void updateSensorReadings();
int getLightDif(int currentLight, int lastLight);
void calibrateLineThresh();
void resetCalibrateThreshold();
void saveThresholdToEEPROM();
void loadThresholdFromEEPROM();
void handleCalibrateButton();
void calibrateLineEEPROM();
float getWhiteAng();
float getWhiteAngClustered();
void updateIsWhite0();
void updateIsWhite();
int whiteCount();
bool whiteDetected();
void calibrateThresh0();
bool lineDetected();
void setupLightSensors();

#endif
