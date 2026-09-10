# Code

The boat software, split into the two algorithms and the firmware, with the
development excerpts kept next to them so the optimisation steps in the report
can be traced back to the code.

## Requirements

- Python 3.9 or newer.
- Python packages: install them from the repository root with
  `pip install -r requirements.txt` (matplotlib, pandas, openpyxl, numpy).
- The tracking controller needs the course simulator running on the same
  machine and listening on its usual UDP ports (GPS 60100, IMU 60101, buoys
  60103, control 60199, path 60198). The planner runs on its own and needs no
  simulator.
- To drive the real boat you also need the Arduino IDE for `arduino/basic.ino`,
  and a serial connection to the INS and the GPS (the optional `pyserial`
  package listed in `requirements.txt`).

## Planning

| File | Role |
| --- | --- |
| `planning/rrt_planner.py` | Final planner. Converts longitude/latitude to a map, grows an RRT with goal-biased sampling, prunes the path, evaluates each run by length and reports the score. Same file as Listing 7 in the report. |
| `planning/excerpts/angle_constraint.py` | `steer()` with the 20° turning-angle limit that keeps the path from taking sharp corners. |
| `planning/excerpts/path_pruning.py` | `smooth_path()`: replaces the zig-zag node chain with collision-free straight segments. |
| `planning/excerpts/path_length.py` | The 200-iteration loop that keeps the shortest path and logs the run statistics to Excel. |
| `planning/excerpts/rrt_class_skeleton.py` | Outline of the RRT class, its `Node` and `AreaBounds` helper classes and the methods behind them. |

## Control

| File | Role |
| --- | --- |
| `control/tracking_controller.py` | Final controller. Receives GPS, IMU and buoy data over UDP, runs LOS guidance and the PID steering law, applies the throttle schedule and sends the commands to the simulator. Same file as Listing 8 in the report. |
| `control/excerpts/pid_parameters.py` | The PID steering function, including the wrap-around handling of the heading error near 0°/360°. |
| `control/excerpts/throttle_control.py` | The LOS function with the throttle schedule (0.4 / 0.8 / 1.0) used near waypoints. |

## Firmware

`arduino/basic.ino` runs on the boat's Arduino. Pin 2 selects between RC control
and automatic control; pins 8–11 drive the left and right propellers; in
automatic mode the sketch reads `left,right` power ratios from the serial port
and generates the PWM timing for the H-bridges.

## Archive

`archive/` keeps the earlier scripts that were written while the project was
developing, including their original Chinese comments:

| File | Role |
| --- | --- |
| `archive/rrt_animation_dev.py`, `archive/rrt_animation_dev_v2.py` | Early RRT implementations with a matplotlib animation of the tree growing. |
| `archive/los_pid_simulator_dev.py` | Early version of the simulator client, before the LOS/PID rewrite. |
| `archive/heading_correction_dev.py` | First attempt at correcting the steering around the 0°/360° discontinuity. |
| `archive/throttle_los_dev.py` | First attempt at the waypoint switching and throttle logic. |
