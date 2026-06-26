// Robot 6 (organized) -- sketch entry point.
// Arduino runs setup() once and loop() forever after upload, so they live here
// (same structure as Lightweight-0718). All other logic lives in the module
// .cpp files and is declared through main.h; the per-loop sensor refresh
// updateData() is in main.cpp.
#include "main.h"

void setup() {
  pinMode(13, OUTPUT);

  pinMode(switch_pin, INPUT);
  pinMode(module_pin, INPUT_PULLUP);
  pinMode(calibrate_button_pin, INPUT_PULLUP);  // line-calibration button to GND
  pinMode(mode_button_pin, INPUT_PULLUP);       // button 3: offense/defense toggle
  pinMode(strategy_button_pin, INPUT_PULLUP);   // button 4: usingStrategy toggle
  pinMode(light_gate_pin, INPUT);

  // -------- Status LEDs --------
  pinMode(led1_pin, OUTPUT);  // 5  -> robot on
  pinMode(led2_pin, OUTPUT);  // 30 -> scanning green
  pinMode(led3_pin, OUTPUT);  // 31
  pinMode(led4_pin, OUTPUT);  // 22
  // LEDs are active-low: HIGH = off, LOW = on
  digitalWrite(led1_pin, HIGH);
  digitalWrite(led2_pin, HIGH);
  digitalWrite(led3_pin, playingOffense ? LOW : HIGH);  // on for offense (default), off for defense
  digitalWrite(led4_pin, usingStrategy ? LOW : HIGH);   // on while usingStrategy (default true)
  // digitalWrite(13, HIGH);
  Serial.begin(9600);  // Begin Serial for Serial Monitor
  Serial8.begin(57600);

  delay(100);

  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  IRWriteCommand(14, 1);
  IRWriteCommand(14, 2);

  // -------- Motors ---------

  // Top Left Motor
  TL_MOT.begin(19200);
  delay(5);
  TL_MOT.write(0xAA);
  TL_MOT.write(0x83);  // Exit Safe Start
  // Top Right Motor
  TR_MOT.begin(19200);
  delay(5);
  TR_MOT.write(0xAA);
  TR_MOT.write(0x83);  // Exit Safe Start
  // Bottom Left Motor
  BL_MOT.begin(19200);
  delay(5);
  BL_MOT.write(0xAA);
  BL_MOT.write(0x83);  // Exit Safe Start
  // Bottom Right Motor
  BR_MOT.begin(19200);
  delay(5);
  BR_MOT.write(0xAA);
  BR_MOT.write(0x83);  // Exit Safe Start

  // Ultrasonic
  urm13Init(urmFront);
  urm13Init(urmRight);
  urm13Init(urmBack);
  urm13Init(urmLeft);

  // -------- BNO ------------

  // while (!bno.begin()) { // 0x0C is NDOF mode - not true north
  // /* There was a problem detecting the BNO055 ... check your connections */
  // Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  // delay(1000);
  // }
  // delay(100);

  setupCompass();
  setupLightSensors();
  setupDribbler();


  while (switch_on) {
    updateData();
  }
  digitalWrite(led1_pin, LOW);  // robot on: start button released the wait loop (active-low = on)
  loadThresholdFromEEPROM();

}
                                              
// Debounced falling-edge detector for an active-low button. Returns true once on
// the loop where the button transitions released -> pressed. Each button keeps
// its own state (passed by reference) so they don't interfere.
bool buttonPressed(int pin, int &lastReading, int &state, unsigned long &lastChange) {
  int reading = digitalRead(pin);
  bool pressed = false;
  if (reading != lastReading) {
    lastChange = millis();  // input changed: restart debounce timer
  }
  if (millis() - lastChange > buttonDebounceMs) {
    if (reading != state) {
      state = reading;
      if (state == LOW) pressed = true;  // falling edge = button just pressed
    }
  }
  lastReading = reading;
  return pressed;
}

// Button 3: toggle offense <-> defense. LED3 on for offense, off for defense.
void handleModeButton() {
  if (buttonPressed(mode_button_pin, modeButtonReading, modeButtonState, modeButtonChangeTime)) {
    playingOffense = !playingOffense;
  }
  digitalWrite(led3_pin, playingOffense ? LOW : HIGH);  // active-low: LOW = on
}

// Button 4: toggle usingStrategy. LED4 on while usingStrategy is true.
void handleStrategyButton() {
  if (buttonPressed(strategy_button_pin, strategyButtonReading, strategyButtonState, strategyButtonChangeTime)) {
    usingStrategy = !usingStrategy;
  }
  digitalWrite(led4_pin, usingStrategy ? LOW : HIGH);  // active-low: LOW = on
}

void loop() {
  updateData();

  handleCalibrateButton();

  if (calibrating) {
    calibrateLineEEPROM();  // sweep mode: read sensors, recompute thresholds, save + print
    return;                 // don't drive while calibrating
  }

  handleModeButton();      // button 3: offense/defense
  handleStrategyButton();  // button 4: usingStrategy

  if (playingOffense) {
    offenseMain();
  } else {
    defenseMain();
  }

  Serial.print(dist_front); Serial.print(", ");
  Serial.print(dist_left); Serial.print(", ");
  Serial.print(dist_right); Serial.print(", ");
  Serial.println(dist_back);

}
