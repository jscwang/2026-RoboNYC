#include "main.h"

void pidConfig(PID &c, float kp, float ki, float kd,
               float outMin, float outMax, float iTermMax, float dAlpha) {
  c.kp = kp;
  c.ki = ki;
  c.kd = kd;
  c.outMin = outMin;
  c.outMax = outMax;
  c.iTermMax = iTermMax;
  c.dAlpha = dAlpha;
  pidReset(c);
}

void pidReset(PID &c) {
  c.integral = 0.0;
  c.prevError = 0.0;
  c.dState = 0.0;
  c.lastUs = 0;
  c.first = true;
}

float pidStep(PID &c, float error) {
  unsigned long now = micros();
  float dt = c.first ? 0.0 : (now - c.lastUs) / 1000000.0;  // seconds (unsigned subtraction is wrap-safe)
  c.lastUs = now;

  // Proportional (always applied).
  float out = c.kp * error;

  // Integral with anti-windup. Only when enabled, and never on the first step / a
  // bad dt. The clamp limits the I term's contribution and back-solves the stored
  // integral so it can't keep winding past the limit.
  if (c.ki != 0.0 && !c.first && dt > 0.0) {
    c.integral += error * dt;
    float iTerm = c.ki * c.integral;
    if (c.iTermMax > 0.0) {
      if (iTerm > c.iTermMax) { iTerm = c.iTermMax; c.integral = iTerm / c.ki; }
      else if (iTerm < -c.iTermMax) { iTerm = -c.iTermMax; c.integral = iTerm / c.ki; }
    }
    out += iTerm;
  }

  // Derivative on error, low-pass filtered. Skipped on the first step / a bad dt.
  if (c.kd != 0.0 && !c.first && dt > 0.0) {
    float deriv = (error - c.prevError) / dt;
    c.dState += c.dAlpha * (deriv - c.dState);  // dAlpha = 1 -> raw derivative
    out += c.kd * c.dState;
  }

  c.prevError = error;
  c.first = false;

  // Output clamp.
  if (out > c.outMax) out = c.outMax;
  else if (out < c.outMin) out = c.outMin;
  return out;
}
