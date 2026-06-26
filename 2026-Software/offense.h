#ifndef OFFENSE_H
#define OFFENSE_H
#include "main.h"

void offenseMain();
void backSpin();
void strikerFunc();
void score();
void goHome();
void aroundBall();
void aroundBallTwoZone();
double getAirWallEffect(int dist_from_wall, float start_thresh, float stop_thresh, float reverse_thresh);
void airWall();
void oldAirWall();
void airWallDefense();
void whiteLine();
void circleAroundBall();
void setDriveBounds(float frontBoundDist, float leftBoundDist, float rightBoundDist, float backBoundDist);

#endif
