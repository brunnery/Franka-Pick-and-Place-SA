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
├── software/               Python software
│   ├── franka_pick_place/  Python package
│   ├── config/             Configuration files
│   ├── tests/              Tests
│   ├── main.py             Entry point
│   └── requirements.txt    Python dependencies
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

### Python software

```bash
cd software
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

### Arduino firmware

Open `arduino/sketch_sep24a/sketch_sep24a.ino` in the Arduino IDE, select the board and port, and upload.

<!-- TODO: board type, wiring, required libraries -->

## Hardware

<!-- TODO: robot, gripper, sensors, microcontroller -->

## Author

<!-- TODO: name, institute, supervisor, semester -->
