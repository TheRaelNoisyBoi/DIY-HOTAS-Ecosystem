# DCS World Telemetry Export (HFB Joystick haptics)

`HOTAS_Export.lua` exports live flight data from DCS World to a small text file. The **Tactical MFD Control Hub** app reads that file to drive the HFB Joystick's haptic motor (G-load, AoA buffet, taxi/roll rumble, gun, weapon release and touchdown) and to show the instruments on its MON › DCS page.

It works with FC3 and full-fidelity modules, and it does not replace other export scripts: SRS, Tacview, DCS-BIOS and other exporters keep working.

Only the HFB Joystick uses this. The throttle, collective and rudder pedals do not need it.

## Installation

1. Close DCS World.
2. Open your DCS Saved Games folder: `%USERPROFILE%\Saved Games\DCS\` (on some installs it is `DCS.openbeta`).
3. Open the `Scripts` folder inside it. Create it if it doesn't exist.
4. Copy `HOTAS_Export.lua` into `Scripts`.
5. In the same folder, open `Export.lua` with a text editor. If it doesn't exist, create an empty text file with that exact name (check that Windows didn't save it as `Export.lua.txt`).
6. Add this line at the **end** of `Export.lua`, after any lines that other tools (SRS, Tacview, DCS-BIOS…) have already added:

   ```lua
   dofile(lfs.writedir() .. [[Scripts\HOTAS_Export.lua]])
   ```

7. Save the file and start DCS.

Once you are in a mission, the script writes to `Saved Games\DCS\Temp\hotas_telemetry.txt` about 30 times per second. The app picks it up automatically: MON › DCS shows your aircraft and live data.

> **Why at the end?** The script chains the export callbacks that were defined before it, so every tool loaded earlier keeps receiving its data.

## Exported fields

One line, `KEY:value` pairs separated by `;`:

| Field | Meaning |
|---|---|
| `TIME` | Mission time (s) |
| `AC` | Aircraft type |
| `STATE` | `PARKED`, `TAXI`, `ROLL`, `AIRBORNE` (or `INIT`, `NO_AIRCRAFT`, `OFFLINE`) |
| `G` | Vertical G-load |
| `AOA` | Angle of attack (°), computed from the body-frame velocity |
| `TD` | 1 for about one second after a touchdown |
| `SINK` | Sink rate of the last touchdown (m/s) |
| `GUN` | 1 while the gun is firing |
| `MSL` | 1 for about one second after a weapon release |
| `PL` | 1 if payload data is available for the module |
| `PITCH`, `BANK`, `HDG` | Attitude and heading (°) |
| `ALT` | Altitude above sea level (m) |
| `IAS` | Indicated airspeed (km/h) |
| `MACH` | Mach number |
| `VS` | Vertical speed (m/s, positive = climbing) |

If something goes wrong inside the script, the line starts with `ERR:` and the app shows **EXPORT ERROR**. Check `Saved Games\DCS\Logs\dcs.log` for details.

## Uninstall

Delete the `dofile(...)` line from `Export.lua` and the `HOTAS_Export.lua` file.
