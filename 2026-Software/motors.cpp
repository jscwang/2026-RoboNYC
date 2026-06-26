#include "main.h"

void setMotorSpeed(float motor_speed, Stream &port) {  // Port is passed as reference as to modify real object
  int real_motor_speed = 3200 * motor_speed;
  if (real_motor_speed < 0) {
    port.write(0x86);                      // motor reverse command
    real_motor_speed = -real_motor_speed;  // make speed positive
  } else {
    port.write(0x85);  // motor forward command
  }
  port.write(real_motor_speed & 0x1F);
  port.write(real_motor_speed >> 5 & 0x7F);
}

void drive(float ang, float drive_speed) {
  float rotation = (processed_heading / 180) / rot_const;
  float ang_accepted;
  float TL_speed = 0;
  float TR_speed = 0;
  float BL_speed = 0;
  float BR_speed = 0;
  if (motor_switch_on) {  // && module_on) {
    if (ang >= 0 && ang <= 45) {
      ang_accepted = 45 - ang;
      BR_speed = cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = -sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 45 && ang <= 90) {
      ang_accepted = ang - 45;
      BR_speed = cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = -sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 90 && ang <= 135) {
      ang_accepted = ang - 45;
      BR_speed = cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = -sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 135 && ang <= 180) {
      ang_accepted = 225 - ang;
      BR_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = -sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 180 && ang <= 225) {
      ang_accepted = 225 - ang;
      BR_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = -sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 225 && ang <= 270) {
      ang_accepted = ang - 225;
      BR_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = -sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 270 && ang <= 315) {
      ang_accepted = ang - 225;
      BR_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = -sin(toRadian(ang_accepted)) * drive_speed;
    } else if (ang > 315 && ang <= 360) {  // ang > 315 && ang <= 360
      ang_accepted = 405 - ang;
      BR_speed = cos(toRadian(ang_accepted)) * drive_speed;
      TR_speed = sin(toRadian(ang_accepted)) * drive_speed;
      TL_speed = -cos(toRadian(ang_accepted)) * drive_speed;
      BL_speed = -sin(toRadian(ang_accepted)) * drive_speed;
    }
    TL_speed = constrain(TL_speed + rotation, -1, 1);
    TR_speed = constrain(TR_speed + rotation, -1, 1);
    BL_speed = constrain(BL_speed + rotation, -1, 1);
    BR_speed = constrain(BR_speed + rotation, -1, 1);
  } else {
    BR_speed = 0;
    TR_speed = 0;
    BL_speed = 0;
    TL_speed = 0;
  }

  setMotorSpeed(BR_speed, BR_MOT);
  setMotorSpeed(TL_speed, TL_MOT);
  setMotorSpeed(TR_speed, TR_MOT);
  setMotorSpeed(BL_speed, BL_MOT);

  // Serial.print("TL: ");
  // Serial.print(TL_speed);
  // Serial.print(", ");
  // Serial.print("TR: ");
  // Serial.print(TR_speed);
  // Serial.print(", ");
  // Serial.print("BL: ");
  // Serial.print(BL_speed);
  // Serial.print(", ");
  // Serial.print("BR: ");
  // Serial.print(BR_speed);
  // Serial.print(", ");
  // Serial.print("Rot: ");
  // Serial.print(rotation);
  // Serial.print(", ");
  // Serial.print("ang: ");

  //polarity depends how you wire motors
  // Serial.println(ang);
}

void turnInPlace(float turnSpeed) {
  if (!motor_switch_on) turnSpeed = 0;
  setMotorSpeed(turnSpeed, BR_MOT);
  setMotorSpeed(turnSpeed, TL_MOT);
  setMotorSpeed(turnSpeed, TR_MOT);
  setMotorSpeed(turnSpeed, BL_MOT);
}

