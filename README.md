# DIY Flight Control Ecosystem - Firmware Architecture

<img width="1561" height="1384" alt="Hangar_v2" src="https://github.com/user-attachments/assets/b7710558-3cf9-41cd-b2a7-af7f84f8bb70" />

This repository contains the unified firmware, technical documentation, and Bill of Materials (BOM) for a series of DIY flight simulation controllers. 

**About the Project:**
This ecosystem is the result of an actively developed and continuously evolving personal endeavor. It was born purely out of a lifelong passion for aviation. Being based in Chile, South America, importing high-end commercial flight simulation hardware is notoriously difficult and prohibitively expensive due to extreme shipping costs and import taxes. What started as a personal quest to bring the aircraft cockpit to my desk using accessible, off-the-shelf electronics has naturally grown into the comprehensive ecosystem you see today.

**Note:** This repository is dedicated solely to the software layer and electrical documentation. The mechanical 3D assets (STLs) required for assembly are distributed via Cults3D.

[Get the 3D Assets on Cults3D](https://cults3d.com/en/users/NoisyBoeh/3d-models)

---

## Core Architecture

The entire ecosystem is built around the highly accessible Arduino Pro Micro (ATmega32u4) microcontroller. To ensure plug-and-play compatibility with Windows and modern flight simulators as standard USB HID controllers, the firmware utilizes `Joystick` library developed by Matthew Heironimus.

To overcome the physical I/O limitations of standard microcontrollers, the architecture heavily leverages the I2C protocol. Through modular I2C expansions, the system is capable of managing:
* High-density button matrices via I/O expanders.
* High-precision 16-bit analog-to-digital conversions for flight axes.
* Haptic feedback controllers for physical immersion.

---

## Hardware Modules

### 1. HBF Joystick Base & Grips
Gimbal-based flight Joystick featuring a modular grip attachment interface, comparable to commercial high-end control systems.
* **I/O Capacity:** Scalable architecture utilizing I2C expanders to manage multiple 16-bit analog axes and high-density digital button matrices.
* **Mechanical Design:** Incorporates an interchangeable quick-release ecosystem for swapping grips without hardware rewiring.
* **Grip Variants:** Grip models engineered based on the geometry of the **Su-57 Felon**, **Su-27 Flanker**, **F-22 Raptor**, and **F/A-18 Hornet**.

<img width="1440" height="1440" alt="SU-57 Joystick V5 v53" src="https://github.com/user-attachments/assets/208b57bc-017a-462e-8316-417245c94875" />
<img width="1753" height="1753" alt="SU-30sm Joystick V2 v8" src="https://github.com/user-attachments/assets/1cf1e56d-277f-46ae-9259-5d3dfb1a3825" />
<img width="1440" height="1440" alt="F-18 Joystick v17" src="https://github.com/user-attachments/assets/427fb63a-2681-4cf9-a7c4-b8166cb68b93" />
<img width="1753" height="1753" alt="F 22 Joystick v24" src="https://github.com/user-attachments/assets/d5ad2825-a5a3-488a-874c-024b5ed98492" />

### 2. Throttle Quadrant
Multi-axis control for engine and avionics management.
* **I/O Capacity:** The multiplexed I2C architecture supports up to 31 programmable digital inputs and 6 discrete analog axes per unit.
* **Throttle Variants:** Available in dual-throttle configurations based on the **Su-57 Felon** and **F-22 Raptor**.

<img width="1440" height="1440" alt="ThrottleV5 v37" src="https://github.com/user-attachments/assets/ad36f37d-df54-4dd0-8e6d-52c25b13db43" />
<img width="1384" height="1384" alt="RaptorThrottle 1" src="https://github.com/user-attachments/assets/39fc8e13-be4b-4303-a7ca-ff3d74343f58" />

### 3. Helicopter Collective
Rotary-wing collective pitch lever.
* **I/O Capacity:** Features 1 primary analog axis for collective pitch and a 24-input digital matrix (implementing 9 direct GPIO pins and 15 multiplexed I2C inputs).

<img width="1440" height="1440" alt="collective grip v28" src="https://github.com/user-attachments/assets/10ee0ea5-df1a-4436-ac1a-b20b8c73f6a6" />

### 4. Rudder Pedals
Pedal assembly for yaw axis manipulation and independent ground braking.
* **I/O Capacity:** 3 independent analog axes standardly mapped to Main Yaw (Rudder Axis), Left Toe Brake, and Right Toe Brake.

<img width="1440" height="1440" alt="Rudder Pedals v36" src="https://github.com/user-attachments/assets/047e73e6-7729-4134-9959-6f85a6beef7e" />

### 5. Force Feedback (FFB) Joystick Base [In Development]
Active control system designed to provide realistic Force Feedback dynamics using cost-effective DIY hardware.
* **Development Status:** Active prototyping phase. 

<img width="1440" height="1440" alt="FFB Joystick Base 3" src="https://github.com/user-attachments/assets/04c18008-3910-40e7-9fa6-aa12f22a305e" />

---

## Bill of Materials (BOM)
A comprehensive master list detailing all required electrical components, microcontrollers, sensors, and hardware across all modules is maintained in a centralized spreadsheet. This allows for efficient parts sourcing and project budgeting.

[Access the Master BOM (.xlsx)](./BOM/MASTER_BOM.xlsx)

---

## Command Line Interface (CLI) & Calibration
All microcontrollers in this ecosystem implement a unified Command Line Interface (CLI) accessible via any standard serial terminal (e.g., Arduino IDE Serial Monitor). All configuration parameters are written to the internal EEPROM, ensuring persistence across power cycles without requiring firmware recompilation.

**Serial Connection Parameters:**
* **Baud Rate:** 115200
* **Line Ending:** Newline (`\n`) / Both NL & CR

### System & Calibration Commands
* `STATUS`: Outputs a comprehensive diagnostic report, including current filter coefficients, hysteresis thresholds, axis inversion states, and the min/max physical boundaries stored in the EEPROM.
* `CAL`: Initializes the interactive hardware calibration routine. The system will prompt the user to deflect all connected axes to their physical limits and save the new boundaries to memory.

### Signal Tuning Commands
Requires a numeric parameter passed after the command.
* `JITTER <value>`: Defines the hysteresis window (dynamic deadband) to eliminate sensor noise and mechanical oscillation. Example: `JITTER 15` freezes the axis output if the raw fluctuation is under 15 units.
* `FIL1 <value>`: Sets the alpha coefficient for the primary Exponential Moving Average (EMA) filter. Valid range is `0.01` (heavy smoothing) to `1.00` (instantaneous/raw). Example: `FIL1 0.35`.
* `FIL2 <value>`: Sets the alpha coefficient for the secondary/auxiliary axes filter.

### Axis Inversion Logic
Toggles the logical output direction of the specified axis. Repeated execution reverts the state to normal.
* `INV_X`, `INV_Y`, `INV_Z`: Toggles primary X,Y,Z axes.
* `INV_RX`, `INV_RY`, `INV_RZ`: Toggles auxiliary rotational axes (and Rudder Toe Brakes).
* `INV_THR`: Toggles the primary collective/throttle axis.
* `INV_RUDDER`: Toggles the primary yaw axis.

### Module-Specific Functions
* `ATB1`: Toggles Axis-to-Button (ATB) translation for primary ministicks, enabling digital Hat Switch emulation.
* `ATB2`: Toggles Axis-to-Button translation for analog brake levers (Joystick module specific).

---

## Dependencies & Libraries
To successfully compile and run the firmware modules across this repository, the following third-party and native libraries are required:

* **Joystick** (by Matthew Heironimus)
  * *Repository:* [MHeironimus/ArduinoJoystickLibrary](https://github.com/MHeironimus/ArduinoJoystickLibrary)
* **ADS1X15** (by Rob Tillaart)
  * *Repository:* [RobTillaart/ADS1X15](https://github.com/RobTillaart/ADS1X15)
* **PCF8575** (by Rob Tillaart)
  * *Repository:* [RobTillaart/PCF8575](https://github.com/RobTillaart/PCF8575)
* **Adafruit_DRV2605** (by Adafruit)
  * *Repository:* [Adafruit_DRV2605_Library](https://github.com/adafruit/Adafruit_DRV2605_Library)

---

A sincere thank you to everyone in the flight simulation and DIY community on Cults3D who has downloaded the designs, shared feedback, and supported this project. Your continuous engagement directly drives the active development and refinement of these hardware modules. 

Thank you for trusting my work and helping bring these projects to life!

---

*Designed and engineered by NoisyBoeh.*
