#ifndef MAIN_H
#define MAIN_H

// ============================================================================
// main.h -- central declarations hub for Robot6-Organized.
//
//   * library includes        * #define constants
//   * compile-time const vars  * shared types (URM13 / regindexTypedef)
//   * extern decls for every mutable global (defined once in globals.cpp)
//   * inline helpers
//   * pulls in every module's public API at the bottom
//
// Every .cpp does `#include "main.h"`, so it sees all globals and all module
// prototypes -- cross-module calls can never hit a missing declaration.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>              // IR Sensor
#include <EEPROM.h>            // White-line threshold storage
#include "DFRobot_URM09.h"     // Ultrasonic Sensor
#include <Adafruit_Sensor.h>   // IMU BNO055
#include <Adafruit_BNO055.h>   // IMU BNO055
#include <utility/imumaths.h>  // IMU BNO055
#include <Servo.h>             // Dribbler ESC

// ---- BNO055 register defines ----
#define RST 11               // BNO RST pin number
#define BNO_ADDR 0x28        // I2C address of the BNO055, 28
#define PAGE_ID 0x07         // Register for selecting register page
#define ACC_DATA_X_LSB 0x08  // Page 0: Acceleration Data X LSB
#define OPR_MODE 0x3D        // Page 0: Operation Mode register
#define ACC_CONFIG 0x08      // Page 1: Accelerometer Configuration
#define MODE_AMG 0x07        // Non-fusion mode (Accel + Gyro + Mag)

// ---- Motor serial ports ----
#define BR_MOT Serial2
#define BL_MOT Serial3
#define TR_MOT Serial1
#define TL_MOT Serial5

// ---- URM13 ultrasonic register map / type ----
typedef enum {
  eAddr = 0,
  ePid,
  eVid,
  eDistanceH,
  eDistanceL,
  eInternalTempretureH,
  eInternalTempretureL,
  eExternalTempretureH,
  eExternalTempretureL,
  eConfig,
  eCmd,
  eNoise,
  eSensitivity,
  eRegNum
} regindexTypedef;

#define MEASURE_RANGE_BIT ((uint8_t)0x01 << 4)
#define MEASURE_MODE_BIT ((uint8_t)0x01 << 2)
#define TEMP_CPT_ENABLE_BIT ((uint8_t)0x01 << 1)
#define TEMP_CPT_SEL_BIT ((uint8_t)0x01 << 0)

#define URM13_WINDOW_SIZE 100
#define URM13_MAX_VALID_DISTANCE 200

struct URM13 {
  uint8_t addr;
  int16_t validWindow[URM13_WINDOW_SIZE];
  uint8_t windowCount;
  uint8_t windowIndex;
  int16_t rawDist;
};

// ---- Compile-time constants (header-defined: internal linkage, usable as array sizes) ----
const int esc_pin = 12;
const float striker_home_x = 80;
const float striker_home_y = 120;
const float goalie_home_x = 91;
const float goalie_home_y = 33;
const float goal_x = 79;
const float goal_y = 219 - 12 - 5;
const int sensorSize = 48;
const int ADC_MAX = 4095;            // analogReadResolution(12) -> 0..4095
const int MIN_VALID_GREEN = 30;     // max green below this => sensor treated as broken
const int EEPROM_THRESH_ADDR = 0;    // base address; uses sensorSize*sizeof(int) bytes
const int light_gate_pin = 39;
const int LIGHT_GATE_WINDOW = 10;    // readings in the hasBall() moving average
const float TURN_SPEED = 0.3;        // in-place turn speed
const unsigned long BALL_HOLD_MS = 1000;
const float TURN_TARGET_HEADING = 150;
const float TURN_DEADBAND = 3;
const int MAX_CLUSTERS = sensorSize / 2;
const unsigned long printIntervalMs = 100;
const int switch_pin = 37;
const int motor_switch_pin = 36;
const int module_pin = A12;
const int calibrate_button_pin = 2;  // line-calibration button to GND
const int mode_button_pin = 3;       // button 3: toggle offense <-> defense
const int strategy_button_pin = 4;   // button 4: toggle usingStrategy
const int led1_pin = 5;              // status LED: ON while robot is switched on
const int led2_pin = 30;             // status LED: ON while scanning/calibrating green
const int led3_pin = 31;             // status LED (unassigned)
const int led4_pin = 22;             // status LED (unassigned)
const unsigned long buttonDebounceMs = 50;
const int WINDOW_SIZE = 11;          // ultrasonic median window
const int MIN_WHITE_SENSORS = 2;     // sensors that must read white to count as the line

// ---- inline helpers ----
inline float toRadian(float ang) { return (ang / 180) * 3.14159; }

// ---- extern globals (single definition in globals.cpp) ----
extern Servo esc;
extern uint16_t BNO055_SAMPLERATE_DELAY_MS;
extern Adafruit_BNO055 bno;

extern float goalie_speed;
extern float striker_speed;
extern bool striker;
extern float line_thresh;
extern float slow_down_factor;
extern float nominal_robot_speed;
extern float robot_speed;

extern DFRobot_URM09 URM09_FRONT;
extern DFRobot_URM09 URM09_BACK;
extern DFRobot_URM09 URM09_LEFT;
extern DFRobot_URM09 URM09_RIGHT;

extern int16_t dist_front;
extern int16_t dist_back;
extern int16_t dist_left;
extern int16_t dist_right;

extern int IR_map[8];
extern float front_port_ang_map[8];
extern float back_port_ang_map[8];

extern int max_port_back;
extern int max_port_front;
extern float max_port_front_val;
extern float max_port_back_val;
extern float max_port_val;
extern float max_port_left_val;
extern float max_port_right_val;
extern float port_ang;
extern float ball_ang;
extern float drive_ang;

extern float raw_heading;
extern float init_heading;
extern float processed_heading;
extern float compass_heading;
extern float rotation;
extern float north;

extern float field_length;
extern float field_width;
extern float loc_x;
extern float loc_y;
extern float home_x;
extern float home_y;
extern float striker_home_speed_min;
extern float striker_home_speed_max;
extern float goalie_home_speed_min;
extern float goalie_home_speed_max;
extern float goal_ang;
extern float processed_goal_ang;

extern unsigned long init;
extern bool first_in_front;
extern long ball_in_front_start;
extern float bounce_back_thresh;

extern int selectorPins[];
extern int selectorPinsSize;
extern int readPins[];
extern int readPinsSize;
extern int sensorReadings[sensorSize];

extern int lineThresh[sensorSize];
extern int physicalOrder[sensorSize];
extern int highestThresh[sensorSize];
extern int maxSensorThres[sensorSize];
extern float greenRatio;

extern int light_gate_thresh;
extern int lightGateBuffer[LIGHT_GATE_WINDOW];
extern int lightGateIndex;
extern int lightGateCount;

extern unsigned long lastBall;
extern bool turning;
extern bool turnDone;

extern float whiteX;
extern float whiteY;
extern int numClusters;
extern float whiteDist;

extern bool firstFlip;
extern bool flip;
extern float lastAngle;
extern unsigned long lastWhiteTime;
extern unsigned long whiteTime;
extern float lastWhiteAng;
extern unsigned long lastPrintTime;

extern int ball_capture_pin;
extern float ball_capture_thresh;
extern int ball_capture_val;
extern long time_last_captured;
extern float ball_capture_time;

extern bool switch_on;
extern bool motor_switch_on;
extern bool first_reset_heading;

extern int circle_const;
extern float rot_const;
extern bool opponent_block_x;
extern bool opponent_block_y;
extern bool module_on;

extern int whiteReadings[sensorSize];
extern bool isWhite[sensorSize];
extern bool calibrateOn;
extern int grayscalePins[8];

extern bool calibrating;
extern int lastButtonReading;
extern int buttonState;
extern unsigned long lastButtonChangeTime;

// Game-mode toggles (set by the front-panel buttons, read in loop()).
extern bool playingOffense;  // true = offense, false = defense
extern bool usingStrategy;   // true = aroundBallTwoZone + backSpin, false = plain aroundBall
extern int modeButtonReading;
extern int modeButtonState;
extern unsigned long modeButtonChangeTime;
extern int strategyButtonReading;
extern int strategyButtonState;
extern unsigned long strategyButtonChangeTime;

extern float distLeftBuffer[WINDOW_SIZE];
extern float distRightBuffer[WINDOW_SIZE];
extern float distFrontBuffer[WINDOW_SIZE];
extern float distBackBuffer[WINDOW_SIZE];
extern int leftIndex, rightIndex, frontIndex, backIndex;
extern int leftCount, rightCount, frontCount, backCount;

extern bool camera_on;
extern int yellow_ang;
extern int blue_ang;
extern bool yellow_detected;
extern bool blue_detected;
extern bool ball_captured;

extern URM13 urmRight;
extern URM13 urmFront;
extern URM13 urmLeft;
extern URM13 urmBack;

extern long init_time;
extern int line_dir;
extern bool ledOn;
extern long ledStartTime;

extern byte temp_data[5];
extern byte camera_data[5];

extern float ang_arr[sensorSize];
extern float x_arr[sensorSize];
extern float y_arr[sensorSize];
extern float realX[sensorSize];
extern float realY[sensorSize];

extern int minWhiteThresh[8];
extern int whiteThresh[8];

// ---- core orchestration (defined in main.cpp) ----
void updateData();

// ---- game-mode buttons (defined in the .ino) ----
void handleModeButton();
void handleStrategyButton();

// ---- module public APIs ----
#include "PID.h"
#include "motors.h"
#include "ultrasonic.h"
#include "lightsensors.h"
#include "ballsensor.h"
#include "compass.h"
#include "offense.h"
#include "defense.h"
#include "debug.h"

#endif  // MAIN_H
