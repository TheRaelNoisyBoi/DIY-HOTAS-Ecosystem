# DIY Flight Control Ecosystem - Firmware Architecture

<p align="center">
  <img width="780" alt="Hangar_v2" src="https://github.com/user-attachments/assets/b7710558-3cf9-41cd-b2a7-af7f84f8bb70" />
</p>

This repository contains the unified firmware, technical documentation, configuration app releases and Bill of Materials (BOM) for a series of DIY flight simulation controllers.

**About the Project:**
This ecosystem is the result of an actively developed and continuously evolving personal endeavor. It was born purely out of a lifelong passion for aviation. Being based in Chile, South America, importing high-end commercial flight simulation hardware is notoriously difficult and prohibitively expensive due to extreme shipping costs and import taxes. What started as a personal quest to bring the aircraft cockpit to my desk using accessible, off-the-shelf electronics has naturally grown into the comprehensive ecosystem you see today.

**Note:** This repository is dedicated solely to the software layer and electrical documentation. The mechanical 3D assets (STLs) and the assembly instructions are distributed via Cults3D.

[Get the 3D Assets on Cults3D](https://cults3d.com/en/users/NoisyBoeh/3d-models)

---

## Quick Start

1. **Build** the device using the assembly instructions included with the Cults3D files.
2. **Read the guide** for your device in [`DOCS/`](./DOCS): wiring, I2C addresses, firmware upload and configuration.
3. **Flash the firmware** from [`FIRMWARE/`](./FIRMWARE) with the Arduino IDE (board: *Arduino Leonardo*).
4. **Download the configuration app** from [Releases](../../releases) to calibrate the device and tune its settings.

---

## Repository Contents

| Folder | Contents |
|---|---|
| [`FIRMWARE/`](./FIRMWARE) | Arduino sketches for each device, plus the `I2C_Scanner` diagnostic sketch |
| [`DOCS/`](./DOCS) | Electronics, Firmware & Configuration Guide for each device (PDF) |
| [`BOM/`](./BOM) | Bill of Materials for each device and grip model (CSV) |
| [`DCS/`](./DCS) | DCS World telemetry export script for the HFB Joystick haptics |
| [Releases](../../releases) | Tactical MFD Control Hub configuration app for Windows (`.exe`) |

---

## Core Architecture

The entire ecosystem is built around the highly accessible Arduino Pro Micro (ATmega32u4) microcontroller. To ensure plug-and-play compatibility with Windows and modern flight simulators as standard USB HID controllers, the firmware uses the `Joystick` library developed by Matthew Heironimus.

To overcome the physical I/O limitations of standard microcontrollers, the architecture heavily leverages the I2C protocol. Through modular I2C expansions, the system is capable of managing:
* High-density button matrices via I/O expanders.
* High-precision 16-bit analog-to-digital conversions for flight axes.
* Haptic feedback controllers for physical immersion.

Every device stores its calibration and settings in EEPROM and is configured over USB serial, either with the **Tactical MFD Control Hub** app or with plain serial commands.

---

## Hardware Modules

| Device | Firmware | Axes | Buttons | I2C devices | Guide |
|---|---|---|---|---|---|
| HFB Joystick | `01_HFB_Joystick.ino` | 6 (X, Y, Z, Rx, Ry, Rz) | 28 + 1 hat | 0x20, 0x21, 0x48, 0x49, 0x50, 0x5A | [PDF](./DOCS/HFB_Joystick_Electronics_Firmware_Configuration_Guide.pdf) |
| Twin Engine Throttle | `02_Throttle.ino` | 6 (X, Y, Z, Rx, Ry, Rz) | 31 + 1 hat | 0x20, 0x21, 0x4A | [PDF](./DOCS/Throttle_Electronics_Firmware_Configuration_Guide.pdf) |
| Black Shark Collective | `03_Collective.ino` | 2 (Rx, Ry) | 32 | 0x20, 0x21 | [PDF](./DOCS/Collective_Electronics_Firmware_Configuration_Guide.pdf) |
| Rudder Pedals | `04_Rudder_Pedals.ino` | 3 (Rudder, Rx, Ry) | — | — | [PDF](./DOCS/Rudder_Pedals_Electronics_Firmware_Configuration_Guide.pdf) |

### 1. HFB Joystick Base & Grips
Gimbal-based flight joystick featuring a modular grip attachment interface, comparable to commercial high-end control systems.
* **I/O Capacity:** Two ADS1115 16-bit ADCs (gimbal in the base, grip axes in the grip) and two PCF8575 expanders for up to 28 buttons. Each grip stores its own calibration in an AT24C32 EEPROM.
* **Mechanical Design:** Interchangeable quick-release grips connected through a 4-pin 360° pogo connector, so grips can be swapped without rewiring or recalibrating.
* **Haptics:** DRV2605L driver and ERM motor. Local mode (stick deflection + button effects) or real-time mode driven by DCS World telemetry through the configuration app.
* **Grip Variants:** Grip models engineered based on the geometry of the **Su-57 Felon**, **Su-27 Flanker**, **F-22 Raptor**, and **F/A-18 Hornet**.

<p align="center">
  <img width="720" alt="SU-57 Joystick V5 v53" src="https://github.com/user-attachments/assets/208b57bc-017a-462e-8316-417245c94875" />
  <img width="876" alt="SU-30sm Joystick V2 v8" src="https://github.com/user-attachments/assets/1cf1e56d-277f-46ae-9259-5d3dfb1a3825" />
  <img width="720" alt="F-18 Joystick v17" src="https://github.com/user-attachments/assets/427fb63a-2681-4cf9-a7c4-b8166cb68b93" />
  <img width="876" alt="F 22 Joystick v24" src="https://github.com/user-attachments/assets/d5ad2825-a5a3-488a-874c-024b5ed98492" />
</p>

### 2. Twin Engine Throttle
Multi-axis control for engine and avionics management.
* **I/O Capacity:** Up to 31 programmable digital inputs and 6 analog axes. Hybrid input system: two hall-sensor throttle levers, rotary and flaps on an ADS1115 (0x4A), mini stick directly on the Arduino analog pins.
* **Throttle Variants:** Dual-throttle configurations based on the **Su-57 Felon** and **F-22 Raptor**.

<p align="center">
  <img width="720" alt="ThrottleV5 v37" src="https://github.com/user-attachments/assets/ad36f37d-df54-4dd0-8e6d-52c25b13db43" />
  <img width="692" alt="RaptorThrottle 1" src="https://github.com/user-attachments/assets/39fc8e13-be4b-4303-a7ca-ff3d74343f58" />
</p>

### 3. Black Shark Helicopter Collective
Rotary-wing collective pitch lever with twist throttle.
* **I/O Capacity:** 2 analog axes (collective pitch and twist throttle, both hall-sensor based) and 32 digital inputs through two PCF8575 expanders in the grip.
* **Mechanical Design:** Swappable grip connected through a 4-pin 360° pogo connector, so the twist throttle can rotate freely.

<p align="center">
  <img width="720" alt="collective grip v28" src="https://github.com/user-attachments/assets/10ee0ea5-df1a-4436-ac1a-b20b8c73f6a6" />
</p>

### 4. Rudder Pedals
Pedal assembly for yaw axis manipulation and independent ground braking.
* **I/O Capacity:** 3 independent hall-sensor analog axes mapped to Main Yaw (Rudder Axis), Left Toe Brake (Rx) and Right Toe Brake (Ry).

<p align="center">
  <img width="720" alt="Rudder Pedals v36" src="https://github.com/user-attachments/assets/047e73e6-7729-4134-9959-6f85a6beef7e" />
</p>

### 5. Force Feedback (FFB) Joystick Base [In Development]
Active control system designed to provide realistic Force Feedback dynamics using cost-effective DIY hardware.
* **Development Status:** Active prototyping phase.

<p align="center">
  <img width="720" alt="FFB Joystick Base 3" src="https://github.com/user-attachments/assets/04c18008-3910-40e7-9fa6-aa12f22a305e" />
</p>

---

## Tactical MFD Control Hub (Configuration App)

A Windows desktop app styled as a fighter-jet multifunction display. It detects which device is connected and lets you:
* run the calibration wizard,
* invert axes, tune the filters and the jitter threshold, and toggle the ATB modes,
* monitor every axis, hat and button live,
* drive the HFB Joystick's haptic motor in real time from DCS World telemetry.

**Download:** get the latest `Tactical MFD Control Hub.exe` from the [Releases](../../releases) page (Windows 10/11, 64-bit, no installation required). The app is free to use and is distributed as a compiled executable; it is not covered by this repository's source license.

> Windows SmartScreen may show "Windows protected your PC" the first time, because the executable is not code-signed. Click **More info → Run anyway**. Each release lists the file's SHA-256 checksum so you can verify your download.

All settings are stored on the devices themselves, so the app is only needed for configuration and for DCS haptics.

---

## DCS World Haptics

The HFB Joystick can vibrate in response to G-load, AoA buffet, taxiing, gun fire, weapon release and touchdowns. This requires the telemetry export script in [`DCS/`](./DCS) and the configuration app running during the flight. Installation instructions are in [`DCS/README.md`](./DCS/README.md). The script chains with other exporters (SRS, Tacview, DCS-BIOS).

---

## Bill of Materials (BOM)
Each device and grip model has its own CSV with all the electrical components and hardware, quantities and purchase links. This allows for efficient parts sourcing and project budgeting.

[Access the BOM folder](./BOM)

---

## Firmware Setup

* **Board:** Tools → Board → *Arduino Leonardo* (the Pro Micro uses the same bootloader).
* **USB identity:** give each device its own name and PID with a `boards.local.txt` file next to the AVR core's `boards.txt`, so Windows and games don't mix them up. Suggested values (VID `0x2342`):

| Device | PID | USB product name |
|---|---|---|
| HFB Joystick | `0x8038` | `HFB Joystick` |
| Twin Engine Throttle | `0x8039` | `Twin Engine Throttle` |
| Black Shark Collective | `0x803A` | `Black Shark Collective` |
| Rudder Pedals | `0x803B` | `Rudder Pedals` |

* **Wiring check:** upload [`FIRMWARE/I2C_Scanner.ino`](./FIRMWARE/I2C_Scanner.ino) and open the Serial Monitor at 115200 baud. The expected addresses for each device are listed in the sketch header and in the table above.
* **Version:** the `STATUS` command prints the installed firmware version (`Firmware: …`).

Step-by-step instructions are in each device's guide in [`DOCS/`](./DOCS).

---

## Command Line Interface (CLI)
Every device implements the same serial command interface, accessible from any serial terminal (e.g., the Arduino IDE Serial Monitor). This is what the configuration app uses behind the scenes. All parameters are written to the internal EEPROM, so they persist across power cycles and firmware updates without recompiling.

**Serial Connection Parameters:**
* **Baud Rate:** 115200
* **Line Ending:** Newline (`\n`) or Both NL & CR
* Commands are not case-sensitive.

### Common Commands

| Command | Description |
|---|---|
| `STATUS` | Prints the firmware version, filter coefficients, jitter threshold and axis inversion states. The throttle and collective also print the stored calibration ranges. |
| `CAL` | Starts the interactive calibration. Move each requested axis to both ends, then press Enter (empty line) to save it. |
| `FIL1 <0.01–1.00>` | Alpha coefficient of the primary Exponential Moving Average filter. Lower = heavier smoothing, `1.00` = raw. Example: `FIL1 0.35` |
| `FIL2 <0.01–1.00>` | Alpha coefficient of the secondary/auxiliary axes filter. |
| `JITTER <0–500>` | Hysteresis window (dynamic deadband) in output units (range ±16384). Example: `JITTER 15` |

### Device-Specific Commands

| Device | Axis inversion | Other |
|---|---|---|
| HFB Joystick | `INV_X`, `INV_Y`, `INV_Z`, `INV_RX`, `INV_RY`, `INV_RZ` | `ATB1` (mini stick → hat), `ATB2` (brake lever → button 28), `VCC`, `HAPTIC_RTP_ON`, `HAPTIC_RTP_OFF`, `RTP <0–127>` |
| Twin Engine Throttle | `INV_X`, `INV_Y`, `INV_Z`, `INV_RX`, `INV_RY`, `INV_RZ` | `ATB1` (mini stick → hat), `VCC` |
| Black Shark Collective | `INV_RX` (collective), `INV_RY` (twist throttle) | — |
| Rudder Pedals | `INV_RUDDER`, `INV_RX` (left toe brake), `INV_RY` (right toe brake) | — |

**Calibration notes:**
* **HFB Joystick:** `CAL` opens a menu: `1` = base (X, Y), `2` = grip (Z, Rx, Ry, Rz). For an axis the grip doesn't have, send `d` to disable it instead of saving.
* **Other devices:** `CAL` walks through all axes in order: throttle X → Rz, collective Rx → Ry, pedals Rudder → Rx → Ry.

**Diagnostics:**
* `VCC` prints the measured USB supply voltage (mV), used by the supply compensation of the ADS1115 axes.
* `HAPTIC_RTP_ON` / `HAPTIC_RTP_OFF` / `RTP <value>` control the haptic motor in real time; the app uses them for DCS haptics.

---

## Dependencies & Libraries
To compile the firmware, the following libraries are required:

| Library | Author | Used by | Install |
|---|---|---|---|
| [Joystick](https://github.com/MHeironimus/ArduinoJoystickLibrary) | Matthew Heironimus | All devices | Download the ZIP from GitHub → Sketch → Include Library → Add .ZIP Library |
| [PCF8575](https://github.com/RobTillaart/PCF8575) | Rob Tillaart | Joystick, Throttle, Collective | Library Manager (several libraries share this name: install Rob Tillaart's) |
| [ADS1X15](https://github.com/RobTillaart/ADS1X15) | Rob Tillaart | Joystick, Throttle | Library Manager |
| [Adafruit_DRV2605](https://github.com/adafruit/Adafruit_DRV2605_Library) | Adafruit | Joystick | Library Manager |

`Wire` and `EEPROM` are built into the Arduino AVR core.

---

## License
The firmware and documentation in this repository are released under the [GNU GPL v3](./LICENSE). The Tactical MFD Control Hub executable published in Releases is freeware and is not covered by this license.

---

A sincere thank you to everyone in the flight simulation and DIY community on Cults3D who has downloaded the designs, shared feedback, and supported this project. Your continuous engagement directly drives the active development and refinement of these hardware modules.

Thank you for trusting my work and helping bring these projects to life!

---

*Designed and engineered by NoisyBoeh.*

