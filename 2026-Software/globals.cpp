// globals.cpp -- single definition of every mutable global (declared extern in main.h).
// Sliced verbatim from the original Robot6.ino so values are byte-for-byte identical.
#include "main.h"

Servo esc;               // ESC controlling the dribbler motor
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float goalie_speed = 0.6;
float striker_speed = 0.7;
bool striker = true;
float line_thresh = 450;  // 1023 = black, 100 = white
float slow_down_factor = 0.7;

float nominal_robot_speed;
float robot_speed = nominal_robot_speed;
DFRobot_URM09 URM09_FRONT;  // Create a URM09 object to communicate with I2C
DFRobot_URM09 URM09_BACK;   // Create a URM09 object to communicate with I2C
DFRobot_URM09 URM09_LEFT;   // Create a URM09 object to communicate with I2C
DFRobot_URM09 URM09_RIGHT;  // Create a URM09 object to communicate with I2C

int16_t dist_front;
int16_t dist_back;
int16_t dist_left;
int16_t dist_right;
int IR_map[8] = { 0, 5, 6, 7, 1, 2, 3, 4 };
float front_port_ang_map[8] = { 0, 90, 60, 30, 0, 330, 300, 270 };     //0, 270, 300, 330, 0, 30, 60, 90
float back_port_ang_map[8] = { 0, 270, 240, 210, 180, 150, 120, 90 };  //90, 120, 150, 180, 210, 240, 270

int max_port_back;
int max_port_front;
float max_port_front_val;
float max_port_back_val;
float max_port_val;
float max_port_left_val;
float max_port_right_val;
float port_ang;
float ball_ang;
float drive_ang;

float raw_heading = 0;
float init_heading = 0;
float processed_heading = 0;
float compass_heading = 0;
float rotation;
float north;

float field_length = 220;
float field_width = 160;
float loc_x;
float loc_y;
float home_x;
float home_y;
float striker_home_speed_min = 0.5;
float striker_home_speed_max = 0.6;
float goalie_home_speed_min = 0.3;
float goalie_home_speed_max = 0.4;
float goal_ang;
float processed_goal_ang;

unsigned long init;
bool first_in_front = true;
long ball_in_front_start;

float bounce_back_thresh = 25;
int selectorPins[] = { 9, 10, 11, 32 };
int selectorPinsSize = 4;

int readPins[] = { 23, 27, 26 };
int readPinsSize = 3;

int sensorReadings[sensorSize];
int lineThresh[sensorSize] = {
1134, 2342, 2293, 1986, 1391, 1915, 2147, 1641,
1421, 1936, 1927, 2275, 1661, 1722, 1797, 2562, 
1188, 1266, 1572, 1651, 1665, 2336, 2226, 2687, 
1890, 1564, 2164, 2474, 1540, 1539, 2759, 2015, 
1693, 1726, 1429, 1764, 1746, 1515, 1942, 2254, 
1702, 1726, 2066, 1443, 2584, 2006, 1287, 1681
};
int physicalOrder[sensorSize] = {
    // New board: firmware index already increases around the ring (SEN_i = sheet SENSOR_i),
    // so physical order is simply 1..48 (1-indexed; the code subtracts 1). The 47->0 seam at
    // the front is handled by the wraparound merge in getWhiteAngClustered().
    1,  2,  3,  4,  5,  6,  7,  8,
    9,  10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 32,
    33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48
  };

int highestThresh[sensorSize] = {0};
int maxSensorThres[sensorSize];     // highest (brightest green/background) reading seen per sensor
float greenRatio = 1.1;             // threshold = max green * ratio; raise => stricter (less white)
int light_gate_thresh = 3500;            // temporary threshold; below this => ball present
int lightGateBuffer[LIGHT_GATE_WINDOW];  // ring buffer of recent readings
int lightGateIndex = 0;                  // next write position
int lightGateCount = 0;                  // valid samples so far (<= LIGHT_GATE_WINDOW)
unsigned long lastBall = 0;
bool turning = false;                     // mid in-place turn
bool turnDone = false;                    // already completed the turn for this possession
float whiteX;
float whiteY;
int numClusters = 0;
float whiteDist = 0;  // normalized distance from center to the white line, ~0 on the line, ~1 at ring edge

bool firstFlip = true;
bool flip = false;
float lastAngle = 0;
unsigned long lastWhiteTime = 0;
unsigned long whiteTime = 0;
float lastWhiteAng = 0;
unsigned long lastPrintTime = 0;
int ball_capture_pin = A9;
float ball_capture_thresh = 30;
int ball_capture_val;
long time_last_captured;
float ball_capture_time = 200;
bool switch_on = true;
bool motor_switch_on = false;
bool first_reset_heading = true;

int circle_const = 100;  // bigger is smaller circle

float rot_const = 2;  // increase to rotate slower
bool opponent_block_x = false;
bool opponent_block_y = false;
bool module_on = false;
int whiteReadings[sensorSize];
bool isWhite[sensorSize] = { false };
bool calibrateOn = true;
int grayscalePins[8] = { A14, A15, A8, A9, A13, A12, A17, A16 };
bool calibrating = false;
int lastButtonReading = HIGH;
int buttonState = HIGH;
unsigned long lastButtonChangeTime = 0;

bool playingOffense = true;  // default: offense
bool usingStrategy = true;   // default: backspin + two-zone strategy on
int modeButtonReading = HIGH;
int modeButtonState = HIGH;
unsigned long modeButtonChangeTime = 0;
int strategyButtonReading = HIGH;
int strategyButtonState = HIGH;
unsigned long strategyButtonChangeTime = 0;
float distLeftBuffer[WINDOW_SIZE];
float distRightBuffer[WINDOW_SIZE];
float distFrontBuffer[WINDOW_SIZE];
float distBackBuffer[WINDOW_SIZE];

// Counters to track position in each buffer
int leftIndex = 0, rightIndex = 0, frontIndex = 0, backIndex = 0;
int leftCount = 0, rightCount = 0, frontCount = 0, backCount = 0;
bool camera_on = false;
int yellow_ang;
int blue_ang;
bool yellow_detected = false;
bool blue_detected = false;
bool ball_captured = false;
URM13 urmFront = { 0x14, { 0 }, 0, 0, -1 };
URM13 urmLeft = { 0x11, { 0 }, 0, 0, -1 };
URM13 urmBack = { 0x12, { 0 }, 0, 0, -1 };
URM13 urmRight = { 0x13, { 0 }, 0, 0, -1 };
long init_time = 0;
int line_dir = 0;
bool ledOn = true;
long ledStartTime;
byte temp_data[5];
byte camera_data[5];
float ang_arr[sensorSize] = {
   172.4,  164.9,  157.3,  149.9,  142.5,  134.6,  127.4,  120.0,
   112.5,  105.0,   97.6,   90.0,   82.6,   74.9,   67.3,   59.9,
    52.5,   44.9,   37.6,   30.0,   22.3,   14.9,    7.6,    0.1,
   352.5,  345.0,  337.6,  330.0,  322.4,  315.1,  307.5,  299.9,
   292.4,  284.9,  277.4,  270.0,  262.4,  255.0,  247.3,  239.9,
   232.5,  225.0,  217.5,  209.9,  202.5,  194.9,  187.4,  179.9
};
float x_arr[sensorSize] = {
    0.13,   0.26,   0.39,   0.50,   0.61,   0.71,   0.79,   0.87,
    0.92,   0.97,   0.99,   1.00,   0.99,   0.97,   0.92,   0.87,
    0.79,   0.71,   0.61,   0.50,   0.38,   0.26,   0.13,   0.00,
   -0.13,  -0.26,  -0.38,  -0.50,  -0.61,  -0.71,  -0.79,  -0.87,
   -0.92,  -0.97,  -0.99,  -1.00,  -0.99,  -0.97,  -0.92,  -0.87,
   -0.79,  -0.71,  -0.61,  -0.50,  -0.38,  -0.26,  -0.13,   0.00
};
float y_arr[sensorSize] = {
   -0.99,  -0.97,  -0.92,  -0.87,  -0.79,  -0.70,  -0.61,  -0.50,
   -0.38,  -0.26,  -0.13,   0.00,   0.13,   0.26,   0.39,   0.50,
    0.61,   0.71,   0.79,   0.87,   0.93,   0.97,   0.99,   1.00,
    0.99,   0.97,   0.92,   0.87,   0.79,   0.71,   0.61,   0.50,
    0.38,   0.26,   0.13,   0.00,  -0.13,  -0.26,  -0.39,  -0.50,
   -0.61,  -0.71,  -0.79,  -0.87,  -0.92,  -0.97,  -0.99,  -1.00
};
// Light-sensor positions from BASE_PLATE.kicad_pcb (board dated 2026-06-15), in centimetres
// from the sensor-ring centroid. Index i = SEN_i mux channel = schematic sheet SENSOR_i (NOT
// the silkscreen Sx, which runs opposite: i -> S(23-i) for i<=23, S(71-i) for i>=24).
// +X = robot right, +Y = robot forward; angle = atan2(X, Y). Orientation: index 23 (silkscreen
// S0) is at the FRONT (0 deg), index 47/0 at the back -- the frame is mirrored over the X axis (left/right kept) from the
// raw PCB so forward matches the robot's actual front. These are the TRUE distances -- the ring
// is elliptical (~7.06 cm front/back to ~9.05 cm at the sides), not a fixed radius.
// getWhiteAngClustered() normalizes each cluster, so only direction feeds the math; the unit
// (cm, matching defense_sim.py) is kept for honesty. NOTE: physicalOrder, x_arr, y_arr, ang_arr
// are in this same index order.
float realX[sensorSize] = {
    0.959,   2.009,   3.159,   3.934,   4.634,   5.334,   6.034,   6.784,
    7.834,   8.659,   8.909,   8.959,   8.934,   8.659,   7.734,   6.784,
    6.034,   5.309,   4.659,   3.909,   3.134,   1.984,   0.959,   0.009,
   -0.941,  -1.991,  -3.141,  -3.941,  -4.666,  -5.366,  -6.041,  -6.841,
   -7.791,  -8.741,  -8.941,  -9.016,  -8.891,  -8.641,  -7.691,  -6.766,
   -6.041,  -5.341,  -4.616,  -3.866,  -3.141,  -1.966,  -0.941,   0.009
};
float realY[sensorSize] = {
   -7.218,  -7.443,  -7.543,  -6.793,  -6.043,  -5.268,  -4.618,  -3.918,
   -3.243,  -2.318,  -1.193,   0.007,   1.157,   2.332,   3.232,   3.932,
    4.632,   5.332,   6.057,   6.757,   7.632,   7.432,   7.157,   7.057,
    7.107,   7.432,   7.607,   6.832,   6.057,   5.382,   4.632,   3.932,
    3.207,   2.332,   1.157,   0.007,  -1.193,  -2.318,  -3.218,  -3.918,
   -4.643,  -5.343,  -6.018,  -6.718,  -7.593,  -7.393,  -7.243,  -7.218
};
int minWhiteThresh[8] = { 1023 };  // smaller = more white
int whiteThresh[8] = { 0 };
