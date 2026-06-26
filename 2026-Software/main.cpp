#include "main.h"

// Per-loop sensor + state refresh. Called from setup()'s start-up wait loop and
// at the top of loop() (both in Robot6-Organized.ino). Reads the light ring, IR
// ball, ultrasonics and compass, and recomputes derived state (heading, loc_x/y,
// opponent-block flags, home target).
void updateData() {

  // updateIsWhite();
  // updateSensorReadings();

  module_on = digitalRead(module_pin);

  ball_capture_val = analogRead(ball_capture_pin);

  getBallAng();

  switch_on = digitalRead(switch_pin);
  motor_switch_on = digitalRead(motor_switch_pin);

  // Status LED2 (active-low: LOW = on, HIGH = off) = scanning/calibrating green.
  // LED1 is latched on in setup() once the start button releases the wait loop.
  digitalWrite(led2_pin, calibrating ? LOW : HIGH);

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  raw_heading = orientationData.orientation.x;  // - starting_ang;
  if (!switch_on) {
    init_heading = raw_heading;
  }
  processed_heading = raw_heading - init_heading;  // - starting_ang;
  compass_heading = raw_heading - init_heading;

  if (processed_heading > 180) {
    processed_heading -= 360;
  } else if (processed_heading < -180) {
    processed_heading += 360;
  }

  dist_front = urm13GetDistance(urmFront);
  dist_back = urm13GetDistance(urmBack);
  dist_left = urm13GetDistance(urmLeft);
  dist_right = urm13GetDistance(urmRight);

  loc_x = dist_left < dist_right ? dist_left : field_width - dist_right;
  loc_y = dist_back < dist_front ? dist_back : field_length - dist_front;

}
