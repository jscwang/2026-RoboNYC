#include "main.h"

// Offense behavior: ball approach, possession handling, scoring and wall
// avoidance. The entry point offenseMain() is called from loop() after
// updateData() has refreshed the sensors.

void offenseMain() {
  esc.write(90);

  if (ballDetected()) {
    aroundBall();
    airWall();
    drive(drive_ang, robot_speed);
  } else {
    goHome();
    drive(drive_ang, robot_speed);
  }

  // Heartbeat LED (loop-alive indicator).
  if (millis() - ledStartTime > 200) {
    ledOn = !ledOn;
    ledStartTime = millis();
  }
  digitalWrite(13, ledOn);
}

void backSpin() {
}

void strikerFunc() {
  robot_speed = nominal_robot_speed;
  if (ballDetected()) {
    aroundBall();
  } else {
    goHome();
  }
}

void score() {
}

// Return-to-home navigation.
void goHome() {
  float home_delta_x = home_x - loc_x;
  float home_delta_y = home_y - loc_y;
  float home_ang = atan2(home_delta_x, home_delta_y) * RAD_TO_DEG;
  float max_dist_from_home = 100;
  float dist_from_home = sqrt(sq(home_delta_x) + sq(home_delta_y));
  float home_speed_ratio = dist_from_home / max_dist_from_home;
  home_speed_ratio = constrain(home_speed_ratio, 0.20, 1);

  float home_speed = 0.6 * home_speed_ratio;
  if (dist_from_home < 25) home_speed = 0;
  if (home_ang < 0) home_ang += 360;

  drive_ang = home_ang;
  robot_speed = home_speed;
}

void aroundBall() {
  drive_ang = ball_ang;
  robot_speed = 0.45;
}

void aroundBallTwoZone() {
  drive_ang = ball_ang;
  robot_speed = 0.5;
  drive(drive_ang, robot_speed);
}

double getAirWallEffect(int dist_from_wall, float start_thresh, float stop_thresh, float reverse_thresh) {
  return 1.0;
}

void airWall() {
}

void oldAirWall() {
}

void airWallDefense() {
}

void whiteLine() {
}

void circleAroundBall() {
  drive_ang = ball_ang;
}

void setDriveBounds(float frontBoundDist, float leftBoundDist, float rightBoundDist, float backBoundDist) {
}
