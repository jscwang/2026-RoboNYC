#include "main.h"

// Defensive behavior: white-line follow, recovery and goalie positioning. The
// entry point defenseMain() is called from loop() after updateData().
// printDefenseViz() streams the per-loop CSV feed consumed by the Python tools.

void cameraDefense() {
}

void defenseMain() {
  esc.write(90);

  if (whiteDetected()) {
    whiteTime = millis();
    whiteMove();
  } else {
    numClusters = 0;
    whiteDist = 0;
    robot_speed = 0;
    printDefenseViz(0.0, 0.0, 5);
  }

  drive(drive_ang, robot_speed);
}

// Wrap-safe signed angle difference.
float getAngleDif(float a, float b) {
  float d = fmod(a - b + 540.0, 360.0) - 180.0;
  return d;
}

void whiteMove() {
  lastWhiteAng = getWhiteAngClustered();
  drive_ang = 0;
  robot_speed = 0;
  printDefenseViz(0.0, 0.0, 0);
}

// Serial feed for the Python visualizer (defense_viz.py). One CSV line per loop:
//   V,wlAngle,whiteDist,ballAng,numClusters,slope,cornerness,state,driveAng,speed,<48 isWhite 0/1>
// state: 0 straight  1 push-in  2 retreat  3 back-out  4 recover(off-line)  5 hold(off-line)
void printDefenseViz(float slope, float cornerness, int state) {
  Serial.print("V,");
  Serial.print(lastWhiteAng, 1);  Serial.print(',');
  Serial.print(whiteDist, 3);     Serial.print(',');
  Serial.print(ball_ang, 1);      Serial.print(',');
  Serial.print(numClusters);      Serial.print(',');
  Serial.print(slope, 2);         Serial.print(',');
  Serial.print(cornerness, 2);    Serial.print(',');
  Serial.print(state);            Serial.print(',');
  Serial.print(drive_ang, 1);     Serial.print(',');
  Serial.print(robot_speed, 2);   Serial.print(',');
  for (int i = 0; i < sensorSize; i++) Serial.print(isWhite[i] ? '1' : '0');
  Serial.println();
}

void goalieFunc() {
  robot_speed = nominal_robot_speed;
  if (!ballDetected()) goHome();
}

void goalieFunc0() {
  robot_speed = nominal_robot_speed;
  if (!ballDetected()) goHome();
}
