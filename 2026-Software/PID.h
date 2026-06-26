#ifndef PID_H
#define PID_H

#include <Arduino.h>

// ============================================================================
// PID.h -- lightweight reusable PID controller for the robot's control loops.
//
// One PID struct holds all the state for one loop, so declare a controller at
// file scope (its state must persist across loop() iterations) and step it once
// per cycle. dt is measured internally from micros(), so the controller tolerates
// the variable loop period (sensor reads, I2C, etc.). Each loop regulates its
// error toward 0: pass the current error to pidStep() and apply the returned,
// clamped output.
// ============================================================================

struct PID {
  // --- gains / limits (set by pidConfig) ---
  float kp, ki, kd;     // proportional, integral, derivative gains
  float outMin, outMax; // output clamp
  float iTermMax;       // anti-windup: max |ki*integral|  (<= 0 disables the clamp)
  float dAlpha;         // derivative low-pass in (0,1];  1 = raw, smaller = smoother

  // --- internal state (managed by pidConfig / pidReset / pidStep) ---
  float integral;       // accumulated error*seconds
  float prevError;      // previous error, for the derivative
  float dState;         // filtered derivative
  unsigned long lastUs; // micros() at the previous step
  bool first;           // true until the first step seeds dt and prevError
};

// Set gains + limits and clear state. iTermMax <= 0 -> no integral clamp;
// dAlpha = 1 -> no derivative filtering.
void pidConfig(PID &c, float kp, float ki, float kd,
               float outMin, float outMax, float iTermMax, float dAlpha);

// Clear integral, derivative, and the dt timer. Call whenever the loop is
// interrupted (line lost, ball side flips, gate override) so a stale integral or
// a first-step derivative spike can't leak into the next engagement.
void pidReset(PID &c);

// One control step toward setpoint 0. Pass the current error; returns the clamped
// output. dt is measured internally; the I and D terms are skipped on the first
// step and any time dt <= 0.
float pidStep(PID &c, float error);

#endif  // PID_H
