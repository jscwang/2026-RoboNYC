#ifndef BALLSENSOR_H
#define BALLSENSOR_H
#include "main.h"

float getBallAng();    // primary: weighted vector-sum centroid over all 14 ports
float ballAngOld();    // previous: max-port + parabolic neighbor interpolation
float getBallDist();   // primary: estimated distance (cm) from peak strength + height model
float ballDistOld();   // previous: raw peak-strength proximity proxy (max_port_val)
int IRWriteCommand(byte command, byte port);
bool ballDetected();
bool ballCaptured();
void setupDribbler();
bool hasBall();

#endif
