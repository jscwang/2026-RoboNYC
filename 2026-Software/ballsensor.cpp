#include "main.h"

// IR ball sensor: I2C transport to the sensor co-processor, ball bearing /
// distance helpers, and capture detection (dribbler light gate).

float getBallAng() {
  max_port_front_val = 0; max_port_back_val = 0;
  max_port_front = 0; max_port_back = 0;
  max_port_val = 0;
  return ball_ang;
}

float getBallDist() {
  return 999.0;
}

float ballDistOld() {
  return max_port_val;
}

float ballAngOld() {
  return ball_ang;
}

int IRWriteCommand(byte command, byte port) {
  if (port == 1) {                 // Actually port 0 (front)
    Wire.beginTransmission(0x01);  // Tells Arduino to begin transmission to Teensy
    Wire.write(command);           // See pulse of IR ball - set to pulse mode
    Wire.endTransmission();
    Wire.requestFrom(0x01, 1);  // Acknowledge whether Teensy received above commands (1 byte)
    while (!Wire.available())   // Wait if I2C not available
      ;
    return Wire.read();
  } else if (port == 2) {           // Actually port 1 (back)
    Wire2.beginTransmission(0x01);  // Tells Arduino to begin transmission to Teensy
    Wire2.write(command);           // See pulse of IR ball - set to pulse mode
    Wire2.endTransmission();
    Wire2.requestFrom(0x01, 1);  // Acknowledge whether Teensy received above commands (1 byte)
    while (!Wire2.available())   // Wait if I2C not available
      ;
    return Wire2.read();
  } else {
    return -1;
  }
}

bool ballDetected() {
  return max_port_val > 3;  // normally 5
}

bool ballCaptured() {
  if (ball_capture_val >= ball_capture_thresh) {
    time_last_captured = millis();
  }
  return millis() - time_last_captured < ball_capture_time;
}

void setupDribbler() {
  esc.attach(esc_pin);
  esc.write(95);  // neutral/arm value
  delay(3000);
}

bool hasBall() {
  lightGateBuffer[lightGateIndex] = analogRead(light_gate_pin);
  lightGateIndex = (lightGateIndex + 1) % LIGHT_GATE_WINDOW;
  if (lightGateCount < LIGHT_GATE_WINDOW) lightGateCount++;

  long sum = 0;
  for (int i = 0; i < lightGateCount; i++) sum += lightGateBuffer[i];
  float avg = (float)sum / lightGateCount;

  return avg < light_gate_thresh;
}
