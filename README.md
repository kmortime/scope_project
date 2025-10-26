# A Digital Camera for Public Use  
### Automated Microscope Display System  

**Author:** Tom Mortimer  
**Software Development:** Kevin Mortimer  
**Version:** 1.17  
**Project:** MinDatNH – Public Interactive Digital Microscope  

---

## Overview

This repository contains the **software and configuration files** for an **automated microscope viewer** designed to bring the micro-mineral collecting hobby to the public in an interactive, hands-on way — without actually touching the microscope.

Visitors can view up to ten real mineral specimens under a digital microscope and:
- Rotate a motorized carousel of specimens.
- Adjust focus and magnification electronically.
- Watch live video feed on a large HDMI monitor.
- Read specimen details and see supporting images onscreen.

The system runs on a **Raspberry Pi 4**, using **Python**, **OpenCV**, **Pygame**, and **GPIO motor control** to drive three stepper motors and manage sensors, buttons, and display overlay.

This is part of the ongoing *A Digital Camera for Public Use* project presented at mineral shows by **Tom Mortimer** of **MinDatNH**.

---

## Project Description

Traditional stereo microscopes at mineral shows engage only one viewer at a time and require manual alignment and coaching.  
This project replaces that with a **digital, motorized, and autonomous system**:

- The viewer interacts via large arcade-style buttons — no microscope handling required.  
- A Raspberry Pi drives stepper motors to control:
  - **Carousel rotation (tray)**
  - **Magnification (zoom ring)**
  - **Focus (rack gear)**  
- Optical sensors and limit switches ensure precise specimen alignment and prevent overtravel.  
- The system automatically displays specimen names, chemistry, and collector information, drawn from per-specimen `.json` files.  

When left idle, the system enters an **auto-run mode**, cycling through each specimen every 20 seconds.  
Manual interaction pauses the sequence until the user stops pressing controls.

---

## Hardware Summary

| Component | Function |
|------------|-----------|
| **Raspberry Pi 4 (4GB)** | Central controller, HDMI video output |
| **Hayear 41 MP Digital Camera** | Provides live HDMI microscope feed |
| **Stepper Motors (3× NEMA 17)** | Drive carousel, focus, and magnification axes |
| **Optical Sensors (2×)** | Detect carousel tab positions for specimen indexing |
| **Limit Switches (2×)** | Protect focus and zoom from overtravel |
| **Arcade Pushbuttons** | User interface for bidirectional control |
| **3D-Printed Carousel** | Integrated lazy-Susan base with optical tabs |
| **Counterweighted Focus Arm** | Compensates for heavy lens and camera assembly |

---

## Software Components

### `test_auto.py`

Main control program — runs all motion logic, camera display, and user interface.

#### Key Capabilities
- Initializes zoom, focus, and tray axes on startup.
- Reads optical sensors to locate “Specimen 1”.
- Loads specimen settings from individual JSON files.
- Controls stepper motors via GPIO for precise movement.
- Handles limit detection and recovery.
- Displays live video feed using OpenCV + Pygame.
- Shows overlay panel with specimen info and optional images.
- Supports both **manual** and **autonomous** modes.

#### Major Threads
| Thread | Role |
|--------|------|
| `monitor_sensors()` | Tracks optical sensors to detect specimen transitions. |
| `autonomous_loop()` | Automatically cycles through specimens on idle. |
| `monitor_button()` | Handles user button presses and motor jog control. |
| `run_initialization()` | Homes and aligns all axes safely. |

#### User Interface
- **Video window**: Full-screen microscope view with real-time ruler scale.
- **Info panel**: Slides in to show specimen name, chemical formula, locality, and collector.
- **Debug overlay**: Optional (`--debug` flag) shows motor step positions.

#### Command Line
```bash
python3 test_auto.py [-d]


### `specimen_10.json`

Example specimen metadata file. Each specimen has its own JSON (e.g., `specimen_1.json` … `specimen_10.json`).

#### Example Contents
```json
{
  "name": "Whitmoreite",
  "chem": "Fe3(PO4)2(OH)2 · 4H2O",
  "location": "Chickering Mine, Walpole, NH",
  "collector": "Tom Mortimer",
  "default_zoom": -1337,
  "default_focus": -610,
  "default_rotation_offset": 121,
  "display_number": 10,
  "map_image": "",
  "eds_image": "",
  "qr_code_image": ""
}

