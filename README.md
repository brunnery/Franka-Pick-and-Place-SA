# Franka Pick and Place – Semester Thesis

Repository for my semester thesis on pick and place with a Franka robot arm.
It contains the CAD files, the Arduino firmware and the Python software.

Gameplan, Methodik und offene Fragen: [docs/PLAN.md](docs/PLAN.md)

## Repository structure

```
├── cad/                    Mechanical design
│   ├── source/             Native CAD files (parts, assemblies)
│   ├── export/             Exchange and print files (STEP, STL, 3MF)
│   └── drawings/           Technical drawings (PDF, DXF)
├── arduino/                Microcontroller code
│   └── sketch_sep24a/      Motor shield test sketch
├── software/               ROS 2 workspace (colcon)
│   ├── src/                ROS 2 packages (sa_interfaces, ...)
│   ├── launch/             Stand-alone launch files (camera)
│   └── config/             Configuration of stand-alone launch files
├── calibration/            Camera intrinsics
├── data/                   Measurements and logs
├── visualization/          3D view of the pick and place sequence
└── docs/                   Documentation
    ├── images/             Photos, screenshots, figures
    └── datasheets/         Datasheets of the components used
```

## 3D visualisation

Interactive 3D view of the sequence (Franka FR3, iris wheel gripper, nut pickup to screwing):

- online: https://brunnery.github.io/Franka-Pick-and-Place-SA/visualization/
- local: open `visualization/index.html` in a browser

Details in [visualization/README.md](visualization/README.md).

## Getting started

### ROS 2 software

ROS 2 Humble, system Python packages only (NumPy 1.21, OpenCV 4.5). A NumPy 2 in `~/.local` breaks the ROS OpenCV bindings, so always run with `PYTHONNOUSERSITE=1` and do not install anything with pip.

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
cd software
colcon build --symlink-install
source install/setup.bash
```

Camera only: `ros2 launch software/launch/camera.launch.py` (from the repository root).

### Arduino firmware

Open `arduino/sketch_sep24a/sketch_sep24a.ino` in the Arduino IDE, select the board and port, and upload.

<!-- TODO: board type, wiring, required libraries -->

## Hardware

<!-- TODO: robot, gripper, sensors, microcontroller -->

## Author

<!-- TODO: name, institute, supervisor, semester -->
