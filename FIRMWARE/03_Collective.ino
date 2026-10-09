/***************************************
  HOTAS Collective - Updated Firmware (Dual PCF8575 + Rx & Ry Axes)
  - Hardware: Arduino Pro Micro (ATmega32u4)
  - Sensors: Analog A1 -> Rx Axis (Collective), Analog A2 -> Ry Axis (Twist/Throttle).
  - Expanders: 2x PCF8575 (0x20, 0x21) for 32 Buttons.
  - Resolution: +/- 16384.
  - Serial Interface: Calibration, Filters, Hysteresis, Inversions.
***************************************/

// Firmware version, shown by the STATUS command
#define FW_VERSION "2026.10"

#include <Wire.h>
#include <PCF8575.h>
#include <Joystick.h>
#include <EEPROM.h>

// -------------------- Hardware Config --------------------
#define NUM_BUTTONS 32
#define PCF8575_ADDR_1 0x20
#define PCF8575_ADDR_2 0x21
#define SERIAL_BAUD 115200
#define COLLECTIVE_PIN A1
#define ACCEL_PIN      A2

// -------------------- Filters & Smoothing --------------------
float collectiveFiltered = 0;
float accelofenFiltered  = 0;
int last_collectiveAxis  = 0;
int last_accelAxis       = 0;

// -------------------- I2C Devices --------------------
PCF8575 pcf1(PCF8575_ADDR_1);
PCF8575 pcf2(PCF8575_ADDR_2);

// -------------------- Joystick HID --------------------
// HID: 32 buttons, 0 hats, Rx and Ry axes enabled (A1=Rx, A2=Ry)
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_BUTTONS, 0,        
                   false, false, false,   // X, Y, Z
                   true,  true,  false,   // Rx, Ry, Rz
                   false, false, false,   // Rudder, Throttle, Accelerator
                   false, false);         // Brake, Steering

int lastButtonStates[NUM_BUTTONS] = {0};

// -------------------- Persistent Memory Structures --------------------
struct CollectiveCalibration {
  uint16_t min[2]; // [0] = Collective / Rx (A1), [1] = Twist / Ry (A2)
  uint16_t max[2];
  uint16_t magic;  
};

struct SystemSettings {
  uint16_t invRx;             // Invert Rx logic
  uint16_t invRy;             // Invert Ry logic
  float alpha1;               // Filter for Analog A1
  float alpha2;               // Filter for Analog A2
  uint16_t jitter_threshold;  // Hysteresis deadband
  uint16_t magic;
};

CollectiveCalibration currentCalib;
SystemSettings sysSettings;

// Defaults
CollectiveCalibration defaultCalib = {
  {318, 300}, // Mins
  {513, 700}, // Maxs
  0xCAFE
}; 

SystemSettings defaultSettings = {
  0, 0,        // Inversions
  0.35, 0.35,  // Alphas
  15,          // Jitter threshold
  0x112E       // Magic
}; 

const int EEPROM_CALIB_ADDR = 0;
const int EEPROM_SETTINGS_ADDR = 40;

// -------------------- EEPROM Helpers --------------------
void saveCalibration() {
  currentCalib.magic = defaultCalib.magic;
  EEPROM.put(EEPROM_CALIB_ADDR, currentCalib);
  Serial.println("Calibration saved to Internal EEPROM.");
}

void loadCalibration() {
  EEPROM.get(EEPROM_CALIB_ADDR, currentCalib);
  if (currentCalib.magic != defaultCalib.magic) {
    currentCalib = defaultCalib;
    saveCalibration(); 
  }
}

void saveSettings() {
  sysSettings.magic = defaultSettings.magic;
  EEPROM.put(EEPROM_SETTINGS_ADDR, sysSettings);
  Serial.println("System settings saved.");
}

void loadSettings() {
  EEPROM.get(EEPROM_SETTINGS_ADDR, sysSettings);
  if (sysSettings.magic != defaultSettings.magic) {
    sysSettings = defaultSettings;
    saveSettings();
  }
}

// -------------------- Utilities & Advanced Filtering --------------------
void updateButtonState(int buttonIndex, bool state) {
  if (buttonIndex < 0 || buttonIndex >= NUM_BUTTONS) return;
  if (state != lastButtonStates[buttonIndex]) {
    Joystick.setButton(buttonIndex, state);
    lastButtonStates[buttonIndex] = state;
  }
}

int applyHysteresis(int current, int &last, int threshold) {
  if (abs(current - last) >= threshold) {
    last = current;
  }
  return last;
}

void applyAdaptiveEMA(long rawValue, float &filteredValue, float baseAlpha, int shiftBits) {
  long truncatedRaw = (rawValue >> shiftBits) << shiftBits;
  float diff = abs(truncatedRaw - filteredValue);
  float dynAlpha = baseAlpha;
  
  int lowThresh = 32 >> shiftBits;
  int highThresh = 256 >> shiftBits;
  
  if (diff < max(1, lowThresh)) {
    dynAlpha = baseAlpha * 0.1;
  } else if (diff > highThresh) {
    dynAlpha = 0.85;
  }
  
  filteredValue = dynAlpha * truncatedRaw + (1.0 - dynAlpha) * filteredValue;
}

void printStatus() {
  Serial.println("\n=== COLLECTIVE STATUS ===");
  Serial.print("Firmware: "); Serial.println(FW_VERSION);
  Serial.println("--- Settings ---");
  Serial.print("Filter 1 (Collective A1 / Rx) Alpha : "); Serial.println(sysSettings.alpha1);
  Serial.print("Filter 2 (Twist A2 / Ry) Alpha      : "); Serial.println(sysSettings.alpha2);
  Serial.print("Jitter Threshold (Hysteresis)       : "); Serial.println(sysSettings.jitter_threshold);
  
  Serial.println("\n--- Axis Inversions ---");
  Serial.print("Rx Axis : "); Serial.println(sysSettings.invRx ? "INV" : "NORM");
  Serial.print("Ry Axis : "); Serial.println(sysSettings.invRy ? "INV" : "NORM");

  Serial.println("\n--- Calibration Ranges ---");
  Serial.print("Rx Axis (A1) : ["); Serial.print(currentCalib.min[0]); Serial.print(", "); Serial.print(currentCalib.max[0]); Serial.println("]");
  Serial.print("Ry Axis (A2) : ["); Serial.print(currentCalib.min[1]); Serial.print(", "); Serial.print(currentCalib.max[1]); Serial.println("]");
  Serial.println("=======================\n");
}

// -------------------- Interactive Calibration --------------------
void calibrateAxisInteractive(const char* name, int idx, int pin) {
  Serial.print("\nCalibrating "); Serial.println(name);
  Serial.println(">>> Move axis from MIN to MAX. Press ENTER to save. <<<");
  
  int vmin = 1023, vmax = 0;
  unsigned long startTime = millis();

  while(Serial.available()) Serial.read(); 

  while (true) {
    int raw = analogRead(pin);
    if (raw < vmin) vmin = raw;
    if (raw > vmax) vmax = raw;

    if (millis() - startTime > 250) {
      Serial.print(name); Serial.print(" min="); Serial.print(vmin); Serial.print(" max="); Serial.println(vmax);
      startTime = millis();
    }

    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) {
        currentCalib.min[idx] = vmin;
        currentCalib.max[idx] = vmax;
        Serial.println("Saved."); delay(200); return;
      }
    }
  }
}

void calibrateCollective() {
  Serial.println("\n=== CALIBRATING COLLECTIVE SYSTEM ===");
  calibrateAxisInteractive("Rx Axis (A1)", 0, COLLECTIVE_PIN);
  calibrateAxisInteractive("Ry Axis (A2)", 1, ACCEL_PIN);
  saveCalibration();
  Serial.println("All axes calibrated and saved successfully.");
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000); 

  pcf1.begin();
  pcf2.begin();
  pcf1.setButtonMask(0xFFFF); 
  pcf2.setButtonMask(0xFFFF); 

  // Protected 16-bit mapped ranges for Rx and Ry
  Joystick.setRxAxisRange(-16384, 16384);
  Joystick.setRyAxisRange(-16384, 16384);
  Joystick.begin(false);
  
  loadCalibration();
  loadSettings();
  
  collectiveFiltered = analogRead(COLLECTIVE_PIN);
  accelofenFiltered  = analogRead(ACCEL_PIN);
}

// -------------------- Main Loop --------------------
void loop() {

  // 1. Serial Command Parser
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();
    
    if (cmd == "CAL") {
      calibrateCollective();
    } else if (cmd == "STATUS") { 
      printStatus();
    } else if (cmd == "INV_RX") { 
      sysSettings.invRx = !sysSettings.invRx; saveSettings(); Serial.println("Rx Axis Inverted"); 
    } else if (cmd == "INV_RY") { 
      sysSettings.invRy = !sysSettings.invRy; saveSettings(); Serial.println("Ry Axis Inverted"); 
    } else if (cmd.startsWith("FIL1 ")) { 
      sysSettings.alpha1 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha1 set to: "); Serial.println(sysSettings.alpha1); 
    } else if (cmd.startsWith("FIL2 ")) { 
      sysSettings.alpha2 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha2 set to: "); Serial.println(sysSettings.alpha2); 
    } else if (cmd.startsWith("JITTER ")) { 
      sysSettings.jitter_threshold = cmd.substring(7).toInt(); saveSettings(); Serial.print("Jitter Deadband set to: "); Serial.println(sysSettings.jitter_threshold); 
    }
  }

  // 2. Read PCF8575 Expanders (32 buttons total across 2 chips)
  uint16_t p1 = pcf1.read16();
  for (int i = 0; i < 16; i++) {
    updateButtonState(i, !((p1 >> i) & 1));
  }
  
  uint16_t p2 = pcf2.read16();
  for (int i = 0; i < 16; i++) {
    updateButtonState(i + 16, !((p2 >> i) & 1));
  }

  // 3. Axis 1 Logic: Collective Pitch mapped to Rx (A1)
  long rawRx = analogRead(COLLECTIVE_PIN);
  applyAdaptiveEMA(rawRx, collectiveFiltered, sysSettings.alpha1, 0); 
  int rxConst = constrain((int)collectiveFiltered, currentCalib.min[0], currentCalib.max[0]);
  int rxAxis = map(rxConst, currentCalib.min[0], currentCalib.max[0], -16384, 16384);
  if (sysSettings.invRx) rxAxis *= -1;
  Joystick.setRxAxis(applyHysteresis(rxAxis, last_collectiveAxis, sysSettings.jitter_threshold));

  // 4. Axis 2 Logic: Twist Grip / Throttle mapped to Ry (A2)
  long rawRy = analogRead(ACCEL_PIN);
  applyAdaptiveEMA(rawRy, accelofenFiltered, sysSettings.alpha2, 0); 
  int ryConst = constrain((int)accelofenFiltered, currentCalib.min[1], currentCalib.max[1]);
  int ryAxis = map(ryConst, currentCalib.min[1], currentCalib.max[1], -16384, 16384);
  if (sysSettings.invRy) ryAxis *= -1;
  Joystick.setRyAxis(applyHysteresis(ryAxis, last_accelAxis, sysSettings.jitter_threshold));

  // Send state to PC
  Joystick.sendState();
  delay(5);
}