# Microscope Scope Control Program

**Version:** 1.17  
**Author:** Tom & Kevin Mortimer  
**Project:** MinDatNH Automated Microscope Viewer  

---

## Overview

This repository contains the control software for an automated microscope platform built on a **Raspberry Pi**.  
The system combines motorized control (tray rotation, zoom, focus), real-time video display, and specimen metadata visualization.

The main program (`test_auto.py`) orchestrates all hardware interactions, user input, and autonomous sequencing of specimens.  
Each specimen has a corresponding `.json` configuration file (e.g., `specimen_10.json`) that defines its display parameters and descriptive metadata.

---

## Files

### `test_auto.py`

The primary control script for the microscope system.

#### Features
- **Motor control:** Uses GPIO pins to drive stepper motors for the TRAY, ZOOM, and FOCUS axes.  
- **Sensor integration:** Reads optical sensors and limit switches to determine tray position and prevent overtravel.  
- **Specimen mapping:** Associates tray rotation positions with known specimen ranges.  
- **Autonomous operation:** Cycles through specimens after a configurable idle period.  
- **User override:** Manual control via physical buttons pauses autonomous behavior.  
- **Pygame interface:**  
  - Displays live video feed from the microscope camera.  
  - Shows a right-hand panel with specimen metadata and optional images.  
  - Includes on-screen ruler scaling with zoom.  
- **Initialization routines:** Automatically homes the tray, zoom, and focus axes.  
- **Safety protections:** Step limits, debounced sensor reads, and GPIO error handling.  

#### Key Components
| Component | Purpose |
|------------|----------|
| **motors** | Defines GPIO pins for each motor (TRAY, FOCUS, ZOOM). |
| **specimen_ranges** | Maps each specimen to its two possible tray step ranges. |
| **monitor_sensors()** | Watches optical sensors to detect tab transitions and specimen alignment. |
| **move_motor_relative() / move_motor_to_absolute()** | Low-level motor stepping logic with safety checks. |
| **autonomous_loop()** | Continuously cycles through specimens after idle timeouts. |
| **initialize_*()** | Home the respective axes and prepare for autonomous operation. |
| **UI panel** | Displays specimen info and image thumbnails. |

#### Dependencies
- `pygame` (UI and image display)
- `opencv-python` (`cv2`, camera capture)
- `RPi.GPIO` (hardware control)
- `math`, `time`, `threading`, `json`, `os`

---

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

