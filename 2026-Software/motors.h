#ifndef MOTORS_H
#define MOTORS_H
#include "main.h"

void setMotorSpeed(float motor_speed, Stream &port);
void drive(float ang, float drive_speed);
void turnInPlace(float turnSpeed);
void drive_turn(float ang, float drive_speed, int turn_dir, float rotation);

#endif
