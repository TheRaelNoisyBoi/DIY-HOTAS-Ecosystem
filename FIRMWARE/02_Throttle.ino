/***************************************
  HOTAS Throttle Universal Firmware
  - Hardware: Arduino Pro Micro (ATmega32u4)
  - Sensors: ADS1115 (0x4A) -> Z, Rx, Ry, Rz. Analog A0/A1 -> X, Y.
  - Expanders: 2x PCF8575 (0x20, 0x21) for 31 Buttons.
  - Resolution: +/- 16384.
  - High-Speed I2C (400kHz) & ADS1115 (860 SPS).
  - Serial Interface: Calibration, Filters, Hysteresis, Inversions, ATB.
***************************************/

#include <Wire.h>
#include <PCF8575.h>
#include <Joystick.h>
#include <ADS1X15.h> 
#include <EEPROM.h>

// -------------------- Hardware Config --------------------
#define NUM_BUTTONS 31
#define PCF8575_ADDR_1 0x20
#define PCF8575_ADDR_2 0x21
#define SERIAL_BAUD 115200

// -------------------- Filters & Smoothing --------------------
float xFiltered = 0, yFiltered = 0, zFiltered = 0;
float rxFiltered = 0, ryFiltered = 0, rzFiltered = 0;

int last_xAxis = 0, last_yAxis = 0, last_zAxis = 0;
int last_rxAxis = 0, last_ryAxis = 0, last_rzAxis = 0;

// -------------------- I2C Devices --------------------
PCF8575 pcf8575_1(PCF8575_ADDR_1);
PCF8575 pcf8575_2(PCF8575_ADDR_2);
ADS1115 ADS(0x4A); // Throttle ADS address

// -------------------- Joystick HID --------------------
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_BUTTONS, 1,
                   true, true, true,    
                   true, true, true,    
                   false, false,        
                   false, false, false);

int lastButtonStates[NUM_BUTTONS] = {0};

// -------------------- Persistent Memory Structures --------------------
struct ThrottleCalibration {
  uint16_t min[6]; // X, Y, Z, Rx, Ry, Rz
  uint16_t max[6];
  uint16_t magic;  
};

struct SystemSettings {
  uint16_t invX, invY, invZ, invRx, invRy, invRz;
  float alpha1;               // For Analog X, Y
  float alpha2;               // For ADS1115 Z, Rx, Ry, Rz
  uint8_t atb1;               // ATB Mode status
  uint16_t jitter_threshold;  // Hysteresis deadband
  uint16_t magic;
};

ThrottleCalibration currentCalib;
SystemSettings sysSettings;

// Defaults
ThrottleCalibration defaultCalib = {
  {0, 0, 0, 13350, 12700, 11080},          
  {1023, 1023, 26000, 21350, 21850, 23500},  
  0xCAFE                                   
};

SystemSettings defaultSettings = {0, 0, 0, 0, 0, 0, 0.35, 0.15, 0, 15, 0x112A}; 

const int EEPROM_CALIB_ADDR = 0;
const int EEPROM_SETTINGS_ADDR = 50;

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

// Adaptive EMA Filter (Shiftbits allows bypassing truncation for 10-bit analog pins)
void applyAdaptiveEMA(long rawValue, float &filteredValue, float baseAlpha, int shiftBits) {
  long truncatedRaw = (rawValue >> shiftBits) << shiftBits;
  float diff = abs(truncatedRaw - filteredValue);
  float dynAlpha = baseAlpha;
  
  // Adjust thresholds based on resolution
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
  Serial.println("\n=== THROTTLE STATUS ===");
  Serial.println("--- Settings ---");
  Serial.print("ATB Mode (X/Y -> Hat): "); Serial.println(sysSettings.atb1 ? "ON" : "OFF");
  Serial.print("Filter 1 (Analog X,Y) Alpha : "); Serial.println(sysSettings.alpha1);
  Serial.print("Filter 2 (ADS Z,Rx,Ry,Rz) Alpha: "); Serial.println(sysSettings.alpha2);
  Serial.print("Jitter Threshold (Hysteresis) : "); Serial.println(sysSettings.jitter_threshold);
  
  Serial.println("\n--- Axis Inversions ---");
  Serial.print("X: "); Serial.print(sysSettings.invX ? "INV" : "NORM");
  Serial.print(" | Y: "); Serial.print(sysSettings.invY ? "INV" : "NORM");
  Serial.print(" | Z: "); Serial.println(sysSettings.invZ ? "INV" : "NORM");
  Serial.print("Rx: "); Serial.print(sysSettings.invRx ? "INV" : "NORM");
  Serial.print(" | Ry: "); Serial.print(sysSettings.invRy ? "INV" : "NORM");
  Serial.print(" | Rz: "); Serial.println(sysSettings.invRz ? "INV" : "NORM");

  Serial.println("\n--- Calibration Ranges ---");
  Serial.print("Analog X : ["); Serial.print(currentCalib.min[0]); Serial.print(", "); Serial.print(currentCalib.max[0]); Serial.println("]");
  Serial.print("Analog Y : ["); Serial.print(currentCalib.min[1]); Serial.print(", "); Serial.print(currentCalib.max[1]); Serial.println("]");
  Serial.print("ADS Z    : ["); Serial.print(currentCalib.min[2]); Serial.print(", "); Serial.print(currentCalib.max[2]); Serial.println("]");
  Serial.print("ADS Rx   : ["); Serial.print(currentCalib.min[3]); Serial.print(", "); Serial.print(currentCalib.max[3]); Serial.println("]");
  Serial.print("ADS Ry   : ["); Serial.print(currentCalib.min[4]); Serial.print(", "); Serial.print(currentCalib.max[4]); Serial.println("]");
  Serial.print("ADS Rz   : ["); Serial.print(currentCalib.min[5]); Serial.print(", "); Serial.print(currentCalib.max[5]); Serial.println("]");
  Serial.println("=======================\n");
}

// -------------------- Interactive Calibration --------------------
int readRawHardware(int axisIndex) {
  switch (axisIndex) {
    case 0: return analogRead(A0);             
    case 1: return analogRead(A1);             
    case 2: return ADS.readADC(0);             
    case 3: return ADS.readADC(1);             
    case 4: return ADS.readADC(2);             
    case 5: return ADS.readADC(3);             
  }
  return 0;
}

void calibrateSingleAxis(const char* axisName, int axisIndex) {
  Serial.print("Calibrating Axis: "); Serial.println(axisName);
  Serial.println(">>> Move axis from MIN to MAX. Press ENTER to save. <<<");
  
  int vmin = 32767;
  int vmax = -32768;
  unsigned long startTime = millis();

  while(Serial.available()) Serial.read(); 

  while (true) {
    int raw = readRawHardware(axisIndex);
    if (raw < vmin) vmin = raw;
    if (raw > vmax) vmax = raw;

    if (millis() - startTime > 250) {
      Serial.print(axisName); Serial.print(" min="); Serial.print(vmin); Serial.print(" max="); Serial.println(vmax);
      startTime = millis();
    }

    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) {
        currentCalib.min[axisIndex] = vmin;
        currentCalib.max[axisIndex] = vmax;
        Serial.println("Saved."); delay(200); return;
      }
    }
  }
}

void runCalibrationMenu() {
  Serial.println("\n=== CALIBRATION MODE ===");
  calibrateSingleAxis("X", 0);
  calibrateSingleAxis("Y", 1);
  calibrateSingleAxis("Z", 2);
  calibrateSingleAxis("Rx", 3);
  calibrateSingleAxis("Ry", 4);
  calibrateSingleAxis("Rz", 5);
  saveCalibration();
  Serial.println("Done. Resuming...");
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast I2C

  if (!ADS.begin()) {
    Serial.println("Error: ADS1115 not connected");
  } else {
    ADS.setGain(1); // 1 = 4.096V
    ADS.setDataRate(7); // 860 SPS
    Serial.println("ADS1115 Connected & Optimized.");
  }

  pcf8575_1.begin();
  pcf8575_2.begin();
  pcf8575_1.setButtonMask(0xFFFF);
  pcf8575_2.setButtonMask(0xFFFF);

  // Protected 16-bit mapped range (same as standard joystick)
  Joystick.setXAxisRange(-16384, 16384);
  Joystick.setYAxisRange(-16384, 16384);
  Joystick.setZAxisRange(-16384, 16384);
  Joystick.setRxAxisRange(-16384, 16384);
  Joystick.setRyAxisRange(-16384, 16384);
  Joystick.setRzAxisRange(-16384, 16384);

  Joystick.begin(false);
  
  loadCalibration();
  loadSettings();
}

// -------------------- Main Loop --------------------
void loop() {

  // 1. Serial Command Parser
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();
    
    if (cmd == "CAL") {
      runCalibrationMenu();
    } else if (cmd == "STATUS") { 
      printStatus();
    } else if (cmd == "INV_X")  { sysSettings.invX = !sysSettings.invX; saveSettings(); Serial.println("X Inverted"); }
      else if (cmd == "INV_Y")  { sysSettings.invY = !sysSettings.invY; saveSettings(); Serial.println("Y Inverted"); }
      else if (cmd == "INV_Z")  { sysSettings.invZ = !sysSettings.invZ; saveSettings(); Serial.println("Z Inverted"); }
      else if (cmd == "INV_RX") { sysSettings.invRx = !sysSettings.invRx; saveSettings(); Serial.println("Rx Inverted"); }
      else if (cmd == "INV_RY") { sysSettings.invRy = !sysSettings.invRy; saveSettings(); Serial.println("Ry Inverted"); }
      else if (cmd == "INV_RZ") { sysSettings.invRz = !sysSettings.invRz; saveSettings(); Serial.println("Rz Inverted"); }
      else if (cmd == "ATB1")   { sysSettings.atb1 = !sysSettings.atb1; saveSettings(); Serial.println(sysSettings.atb1 ? "ATB1 ON" : "ATB1 OFF"); }
      else if (cmd.startsWith("FIL1 ")) { sysSettings.alpha1 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha1 (Analog) set to: "); Serial.println(sysSettings.alpha1); }
      else if (cmd.startsWith("FIL2 ")) { sysSettings.alpha2 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha2 (ADS) set to: "); Serial.println(sysSettings.alpha2); }
      else if (cmd.startsWith("JITTER ")) { sysSettings.jitter_threshold = cmd.substring(7).toInt(); saveSettings(); Serial.print("Jitter Deadband set to: "); Serial.println(sysSettings.jitter_threshold); }
  }

  // 2. Read buttons (PCF8575)
  uint16_t p1 = pcf8575_1.read16();
  for (int i=0; i<16; i++) updateButtonState(i, !((p1 >> i) & 1));
  
  uint16_t p2 = pcf8575_2.read16();
  for (int i=0; i<15; i++) updateButtonState(i+16, !((p2 >> i) & 1)); // 31 buttons total

  // 3. Axes Logic (Adaptive Filters, Mapping & Hysteresis)

  // --- Analog X axis (A0) ---
  long xRaw = analogRead(A0);
  applyAdaptiveEMA(xRaw, xFiltered, sysSettings.alpha1, 0); // 0 bit-shift for 10-bit analog
  int xConst = constrain((int)xFiltered, currentCalib.min[0], currentCalib.max[0]);
  int xAxis = map(xConst, currentCalib.min[0], currentCalib.max[0], -16384, 16384);
  if(sysSettings.invX) xAxis *= -1;
  
  // --- Analog Y axis (A1) ---
  long yRaw = analogRead(A1);
  applyAdaptiveEMA(yRaw, yFiltered, sysSettings.alpha1, 0); // 0 bit-shift for 10-bit analog
  int yConst = constrain((int)yFiltered, currentCalib.min[1], currentCalib.max[1]);
  int yAxis = map(yConst, currentCalib.min[1], currentCalib.max[1], -16384, 16384);
  if(sysSettings.invY) yAxis *= -1;

  if (sysSettings.atb1) {
    int hatDir = -1;
    bool up    = (yAxis < -8000); 
    bool down  = (yAxis >  8000); 
    bool right = (xAxis >  8000);
    bool left  = (xAxis < -8000);

    if (up && !right && !left)          hatDir = 0;
    else if (up && right)               hatDir = 45;
    else if (!up && !down && right)     hatDir = 90;
    else if (down && right)             hatDir = 135;
    else if (down && !right && !left)   hatDir = 180;
    else if (down && left)              hatDir = 225;
    else if (!up && !down && left)      hatDir = 270;
    else if (up && left)                hatDir = 315;

    Joystick.setHatSwitch(0, hatDir);
    Joystick.setXAxis(0);
    Joystick.setYAxis(0);
  } else {
    Joystick.setHatSwitch(0, -1);
    Joystick.setXAxis(applyHysteresis(xAxis, last_xAxis, sysSettings.jitter_threshold));
    Joystick.setYAxis(applyHysteresis(yAxis, last_yAxis, sysSettings.jitter_threshold));
  }

  // --- ADS Z axis ---
  long zRaw = ADS.readADC(0);
  applyAdaptiveEMA(zRaw, zFiltered, sysSettings.alpha2, 3); // 3 bit-shift for 16-bit ADS
  int zConst = constrain((int)zFiltered, currentCalib.min[2], currentCalib.max[2]);
  int zAxisVal = map(zConst, currentCalib.min[2], currentCalib.max[2], -16384, 16384);
  if(sysSettings.invZ) zAxisVal *= -1;
  Joystick.setZAxis(applyHysteresis(zAxisVal, last_zAxis, sysSettings.jitter_threshold));

  // --- ADS Rx axis ---
  long rxRaw = ADS.readADC(1);
  applyAdaptiveEMA(rxRaw, rxFiltered, sysSettings.alpha2, 3);
  int rxConst = constrain((int)rxFiltered, currentCalib.min[3], currentCalib.max[3]);
  int rxAxisVal = map(rxConst, currentCalib.min[3], currentCalib.max[3], -16384, 16384);
  if(sysSettings.invRx) rxAxisVal *= -1;
  Joystick.setRxAxis(applyHysteresis(rxAxisVal, last_rxAxis, sysSettings.jitter_threshold));

  // --- ADS Ry axis ---
  long ryRaw = ADS.readADC(2);
  applyAdaptiveEMA(ryRaw, ryFiltered, sysSettings.alpha2, 3);
  int ryConst = constrain((int)ryFiltered, currentCalib.min[4], currentCalib.max[4]);
  int ryAxisVal = map(ryConst, currentCalib.min[4], currentCalib.max[4], -16384, 16384);
  if(sysSettings.invRy) ryAxisVal *= -1;
  Joystick.setRyAxis(applyHysteresis(ryAxisVal, last_ryAxis, sysSettings.jitter_threshold));

  // --- ADS Rz axis ---
  long rzRaw = ADS.readADC(3);
  applyAdaptiveEMA(rzRaw, rzFiltered, sysSettings.alpha2, 3);
  int rzConst = constrain((int)rzFiltered, currentCalib.min[5], currentCalib.max[5]);
  int rzAxisVal = map(rzConst, currentCalib.min[5], currentCalib.max[5], -16384, 16384);
  if(sysSettings.invRz) rzAxisVal *= -1;
  Joystick.setRzAxis(applyHysteresis(rzAxisVal, last_rzAxis, sysSettings.jitter_threshold));

  Joystick.sendState();
  delay(5);
}