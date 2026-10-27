# Yesterday's Ark

Course project for **Intelligent Ship Innovation Practice — Assignment 4**,
School of Ocean Engineering.

The project builds an autonomous model boat and the software that drives it: an
RRT-based path planner and a line-of-sight (LOS) plus PID path-tracking
controller, ported onto a differential-drive boat with an industrial PC, an
inertial navigation system (INS) and a GPS receiver.

**Team:** Xue Jiang, Linyu Li, Ruoxuan Wang, Lixing Rui, Yumeng Du, Runtao Zhang

## Interactive RRT demo

**[Open the browser-based RRT Navigation Lab](https://choppa129.github.io/Path-Planning-and-Tracking-Control-for-an-Autonomous-Marine-Vehicle/)**

The standalone site lives in [`docs/index.html`](docs/index.html). It visualizes a browser reimplementation of RRT with obstacle clearance, goal bias, turning-angle constraints, tree growth and collision-checked path pruning. Drag start/goal markers and experiment with controls. The map and performance values are **illustrative browser simulation**, not logged data from the original vessel tests.

> **Publishing:** To make the above URL live, enable GitHub Pages for this repository using **Settings → Pages → Deploy from a branch → main → /docs**. The demo file is already committed.

## Repository contents

| Path | What it is |
| --- | --- |
| `docs/Yesterday_Ark_report.pdf` | Written report: RRT planning, LOS/PID tracking, AHP-weighted evaluation, remaining problems and outlook (55 pages) |
| `docs/Yesterday_Ark_slides.pptx` | Presentation slides covering the build, the ported algorithms, the problems we hit and the final runs (45 slides, includes the onboard video) |
| `code/planning/` | RRT path planner and the development excerpts for each optimisation |
| `code/control/` | LOS + PID tracking controller and the development excerpts |
| `code/arduino/basic.ino` | Arduino sketch: RC/automatic switching and motor drive |
| `code/archive/` | Earlier development scripts, kept for reference (comments in Chinese) |
| `requirements.txt` | Python packages used by the code in `code/` |

## How the system works

1. **Setup.** The boat carries an industrial PC, an Arduino, an INS and a GPS.
   The Arduino switches between RC control and automatic control, and drives the
   four propeller channels from a `left,right` command stream.
2. **Path planning.** Obstacles are given in longitude and latitude and mapped to
   a Cartesian grid. An RRT grows a tree from the start to the goal with a
   goal-biased sampler, then prunes the result into a short, smooth path.
   Turning angle, safety radius, pruning, probability concentration and
   repeated evaluation with the shortest path are the optimisations compared in
   the report.
3. **Path tracking.** The LOS guidance law turns the planned waypoints into a
   desired heading; a PID controller converts the heading error into a rudder
   command, and the throttle is reduced near waypoints and widened elsewhere so
   the boat turns on a smaller arc.
4. **Evaluation.** Both algorithms are scored with AHP weights over completion
   time, turning-angle difference, cross-track deviation and path length.


## Running the code

Install the Python dependencies first:

```bash
pip install -r requirements.txt
```

`code/planning/rrt_planner.py` runs standalone and writes
`final_result_shortest_path.png` plus an Excel log of every run:

```bash
python rrt_planner.py
```

`code/control/tracking_controller.py` talks to the course simulator over UDP and
expects it to be running on localhost (GPS 60100, IMU 60101, buoys 60103,
control 60199, path 60198). It logs the run to `path_data.xlsx` when the boat
reaches the goal.

## Notes

- The slide photographs and screenshots keep their original on-image labels;
  only the editable text was translated.
- `basic.ino` was transcribed from the Arduino slide of the presentation, and
  reproduces the sketch as shown there.
- `docs/Yesterday_Ark_slides.pptx` is about 133 MB because it embeds the onboard
  video, so it is tracked with Git LFS (see `.gitattributes`).
- Coursework: please credit the team if you reuse anything here.
