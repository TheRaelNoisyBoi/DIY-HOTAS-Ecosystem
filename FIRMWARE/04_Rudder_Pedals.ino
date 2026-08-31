/***************************************
  HOTAS Rudder Pedals - Universal Firmware
  - Hardware: Arduino Pro Micro (ATmega32u4)
  - Sensors: Analog A0 (Rudder), A1 (Rx/Left Brake), A2 (Ry/Right Brake).
  - Resolution: +/- 16384.
  - Serial Interface: Calibration, Filters, Hysteresis, Inversions.
***************************************/

#include <Joystick.h>
#include <EEPROM.h>

// -------------------- Hardware Config --------------------
#define SERIAL_BAUD 115200
const int RUDDER_PIN = A0; 
const int RX_PIN     = A1;  
const int RY_PIN     = A2; 

// -------------------- Filters & Smoothing --------------------
float rudderFiltered = 0, rxFiltered = 0, ryFiltered = 0;
int last_rudder = 0, last_rx = 0, last_ry = 0;

// -------------------- Joystick HID --------------------
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 0, 0,        
                   false, false, false,   // X, Y, Z
                   true,  true,  false,   // Rx, Ry, Rz
                   true,  false, false,   // Rudder, Throttle, Accelerator
                   false, false);         // Brake, Steering

// -------------------- Persistent Memory --------------------
struct RudderCalibration {
  uint16_t min[3]; // Rudder, Rx, Ry
  uint16_t max[3];
  uint16_t magic;  
};

struct SystemSettings {
  uint16_t invRudder, invRx, invRy;
  float alpha1;               // Rudder filter
  float alpha2;               // Toe brakes filter
  uint16_t jitter_threshold;  
  uint16_t magic;
};

RudderCalibration currentCalib;
SystemSettings sysSettings;

// Defaults based on old constraints
RudderCalibration defaultCalib = {{390, 453, 424}, {600, 521, 509}, 0xCAFE}; 
SystemSettings defaultSettings = {1, 1, 1, 0.30, 0.20, 15, 0x112C}; 

const int EEPROM_CALIB_ADDR = 0;
const int EEPROM_SETTINGS_ADDR = 50;

void saveCalibration() {
  currentCalib.magic = defaultCalib.magic;
  EEPROM.put(EEPROM_CALIB_ADDR, currentCalib);
  Serial.println("Calibration saved.");
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
  Serial.println("Settings saved.");
}

void loadSettings() {
  EEPROM.get(EEPROM_SETTINGS_ADDR, sysSettings);
  if (sysSettings.magic != defaultSettings.magic) {
    sysSettings = defaultSettings;
    saveSettings();
  }
}

// -------------------- Utilities --------------------
int applyHysteresis(int current, int &last, int threshold) {
  if (abs(current - last) >= threshold) last = current;
  return last;
}

void applyAdaptiveEMA(long rawValue, float &filteredValue, float baseAlpha) {
  float diff = abs(rawValue - filteredValue);
  float dynAlpha = baseAlpha;
  
  if (diff < 32) dynAlpha = baseAlpha * 0.1;
  else if (diff > 256) dynAlpha = 0.85;
  
  filteredValue = dynAlpha * rawValue + (1.0 - dynAlpha) * filteredValue;
}

void printStatus() {
  Serial.println("\n=== RUDDER STATUS ===");
  Serial.print("Filter 1 (Rudder) Alpha : "); Serial.println(sysSettings.alpha1);
  Serial.print("Filter 2 (Toe Brakes) Alpha: "); Serial.println(sysSettings.alpha2);
  Serial.print("Jitter Threshold : "); Serial.println(sysSettings.jitter_threshold);
  Serial.println("\n--- Inversions ---");
  Serial.print("Rudder: "); Serial.println(sysSettings.invRudder ? "INV" : "NORM");
  Serial.print("Rx (Left Toe) : "); Serial.println(sysSettings.invRx ? "INV" : "NORM");
  Serial.print("Ry (Right Toe): "); Serial.println(sysSettings.invRy ? "INV" : "NORM");
  Serial.println("=======================\n");
}

int readAxis(int axis) {
  if (axis == 0) return analogRead(RUDDER_PIN);
  if (axis == 1) return analogRead(RX_PIN);
  return analogRead(RY_PIN);
}

void calibrateAxis(const char* name, int idx) {
  Serial.print("Calibrating "); Serial.println(name);
  Serial.println(">>> Move from MIN to MAX. Press ENTER to save. <<<");
  int vmin = 1023, vmax = 0;
  unsigned long startTime = millis();
  
  while(Serial.available()) Serial.read(); 
  
  while (true) {
    int raw = readAxis(idx);
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

void runCalibration() {
  Serial.println("\n=== CALIBRATION ===");
  calibrateAxis("Rudder", 0);
  calibrateAxis("Rx (Left Toe)", 1);
  calibrateAxis("Ry (Right Toe)", 2);
  saveCalibration();
}

// -------------------- Setup & Loop --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  Joystick.setRudderRange(-16384, 16384);
  Joystick.setRxAxisRange(-16384, 16384);
  Joystick.setRyAxisRange(-16384, 16384);
  Joystick.begin(false);
  
  loadCalibration();
  loadSettings();
  
  rudderFiltered = analogRead(RUDDER_PIN);
  rxFiltered = analogRead(RX_PIN);
  ryFiltered = analogRead(RY_PIN);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim(); cmd.toUpperCase();
    
    if (cmd == "CAL") runCalibration();
    else if (cmd == "STATUS") printStatus();
    else if (cmd == "INV_RUDDER") { sysSettings.invRudder = !sysSettings.invRudder; saveSettings(); Serial.println("Rudder Inverted"); }
    else if (cmd == "INV_RX") { sysSettings.invRx = !sysSettings.invRx; saveSettings(); Serial.println("Rx Inverted"); }
    else if (cmd == "INV_RY") { sysSettings.invRy = !sysSettings.invRy; saveSettings(); Serial.println("Ry Inverted"); }
    else if (cmd.startsWith("FIL1 ")) { sysSettings.alpha1 = cmd.substring(5).toFloat(); saveSettings(); Serial.println("Alpha1 Updated"); }
    else if (cmd.startsWith("FIL2 ")) { sysSettings.alpha2 = cmd.substring(5).toFloat(); saveSettings(); Serial.println("Alpha2 Updated"); }
    else if (cmd.startsWith("JITTER ")) { sysSettings.jitter_threshold = cmd.substring(7).toInt(); saveSettings(); Serial.println("Jitter Updated"); }
  }

  // Rudder
  applyAdaptiveEMA(analogRead(RUDDER_PIN), rudderFiltered, sysSettings.alpha1);
  int rConst = constrain((int)rudderFiltered, currentCalib.min[0], currentCalib.max[0]);
  int rAxis = map(rConst, currentCalib.min[0], currentCalib.max[0], -16384, 16384);
  if (sysSettings.invRudder) rAxis *= -1;
  Joystick.setRudder(applyHysteresis(rAxis, last_rudder, sysSettings.jitter_threshold));

  // Left Brake
  applyAdaptiveEMA(analogRead(RX_PIN), rxFiltered, sysSettings.alpha2);
  int rxConst = constrain((int)rxFiltered, currentCalib.min[1], currentCalib.max[1]);
  int rxAxis = map(rxConst, currentCalib.min[1], currentCalib.max[1], -16384, 16384);
  if (sysSettings.invRx) rxAxis *= -1;
  Joystick.setRxAxis(applyHysteresis(rxAxis, last_rx, sysSettings.jitter_threshold));

  // Right Brake
  applyAdaptiveEMA(analogRead(RY_PIN), ryFiltered, sysSettings.alpha2);
  int ryConst = constrain((int)ryFiltered, currentCalib.min[2], currentCalib.max[2]);
  int ryAxis = map(ryConst, currentCalib.min[2], currentCalib.max[2], -16384, 16384);
  if (sysSettings.invRy) ryAxis *= -1;
  Joystick.setRyAxis(applyHysteresis(ryAxis, last_ry, sysSettings.jitter_threshold));

  Joystick.sendState();
  delay(5);
}