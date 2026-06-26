# 2026-RoboNYC

RoboNYC's 2026 RoboCup Junior Soccer robot software and simulation tools.

## Repository layout

```
2026-Software/   Arduino sketch (Teensy) for the robot
Python-Sim/      matplotlib-based defense visualizers / simulators
```

### `2026-Software/`

The Arduino sketch. Open `2026-Software.ino` in the Arduino IDE (the sketch
folder name and the `.ino` name must match). The code is split into modules, all
of which include `main.h` (the central declarations hub):

| Module | Role |
| --- | --- |
| `2026-Software.ino` | `setup()` / `loop()` entry points |
| `main.cpp` / `main.h` | per-loop sensor refresh + shared declarations |
| `globals.cpp` | definitions of all shared globals |
| `motors.*` | motor serial drivers |
| `ultrasonic.*` | URM ultrasonic distance sensors |
| `compass.*` | BNO055 IMU heading |
| `lightsensors.*` | 48-sensor light ring + white-line geometry |
| `ballsensor.*` | IR ball sensor (bearing / distance / capture) |
| `offense.*` | offensive behavior |
| `defense.*` | defensive behavior |
| `PID.*` | generic PID helper |
| `debug.*` | serial debug helpers |

**Dependencies (Arduino libraries):** `Wire`, `EEPROM`, `Servo`,
`DFRobot_URM09`, `Adafruit_Sensor`, `Adafruit_BNO055`. Built for a Teensy with
multiple hardware serial ports (`Serial1`/`Serial2`/`Serial3`/`Serial5`/`Serial8`)
and three I2C buses (`Wire`/`Wire1`/`Wire2`).

### `Python-Sim/`

matplotlib tools for the defense module.

| File | What it does | Run |
| --- | --- | --- |
| `defense_viz.py` | Live visualizer; reads the firmware's serial `V,...` feed and draws the robot's view | `python defense_viz.py <serial_port> [baud]` |
| `defense_sim.py` | Interactive field simulator (arrow keys move the robot, click places the ball) | `python defense_sim.py` |
| `defense_viz_demo.py` | Offline demo that renders a grid of scenarios to `defense_demo.png` | `python defense_viz_demo.py` |

**Dependencies:** `pip install matplotlib numpy pyserial`
