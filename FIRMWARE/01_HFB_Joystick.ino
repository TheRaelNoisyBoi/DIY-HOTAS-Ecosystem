/***************************************
  HOTAS Universal Firmware
  - Serial Calibration & Settings Interface.
  - Axes Resolution: +/- 16384 (Overflow protected).
  - ADS1115: ADS (0x48) -> Z,Rx,Ry,Rz ; ADS2 (0x49) -> X,Y.
  - Calibration: Send "CAL" via Serial.
  - High-Speed I2C (400kHz) & ADS1115 (860 SPS).
  - Adaptive EMA Filters, Bit Truncation & Hysteresis.
***************************************/

#include <Wire.h>
#include <PCF8575.h>
#include <Joystick.h>
#include <ADS1X15.h>           
#include <Adafruit_DRV2605.h>
#include <EEPROM.h>

// -------------------- Hardware Config --------------------
#define NUM_BUTTONS 28
#define PCF8575_ADDR_1 0x20
#define PCF8575_ADDR_2 0x21
#define SWITCH_POS1 7
#define SWITCH_POS2 8
#define LED_ATB_PIN 5         
#define LED_HAPTIC_PIN 6      
#define AT24C32_ADDR 0x50     
#define SERIAL_BAUD 115200

// -------------------- Filters & Smoothing --------------------
float xFiltered = 0, yFiltered = 0, zFiltered = 0;
float rxFiltered = 0, ryFiltered = 0, rzFiltered = 0;

int last_xAxis = 0, last_yAxis = 0, last_zAxis = 0;
int last_rxAxis = 0, last_ryAxis = 0, last_rzAxis = 0;

// -------------------- I2C Devices --------------------
PCF8575 pcf8575_1(PCF8575_ADDR_1);
PCF8575 pcf8575_2(PCF8575_ADDR_2);
ADS1115 ADS(0x48);
ADS1115 ADS2(0x49);
Adafruit_DRV2605 drv;

// -------------------- Joystick HID Config --------------------
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_BUTTONS, 1,
                   true, true, true, true, true, true, false, false, false, false, false);

int lastButtonStates[NUM_BUTTONS] = {0};
bool G_only = false;
bool G_Buttons = false;

// -------------------- Persistent Memory Structures --------------------
struct CalibGimbal {
  uint16_t x_min, x_max, y_min, y_max, magic; 
};

struct CalibJS {
  uint16_t zmin, zmax, rxmin, rxmax, rymin, rymax, rzmin, rzmax;
  uint16_t disabled_z, disabled_rx, disabled_ry, disabled_rz, magic; 
};

struct SystemSettings {
  uint16_t invX, invY, invZ, invRx, invRy, invRz;
  float alpha1;               // Base Gimbal (X, Y)
  float alpha2;               // Grip (Z, Rx, Ry, Rz)
  uint8_t atb1;               // Ministick to Hat Mode
  uint8_t atb2;               // Brake to Button Mode
  uint16_t jitter_threshold;  // Hysteresis deadband
  uint16_t magic;
};

CalibGimbal calibGimbal;
CalibJS calibJS;
SystemSettings sysSettings;

// Defaults 
CalibGimbal defaultGimbal = {9700, 17000, 9000, 15850, 0xAA55};
CalibJS defaultJS = {7350, 8755, 0, 32767, 0, 32767, 13000, 22500, 0, 0, 0, 0, 0x55AA};

// Pre-configured based on user STATUS (Y inverted, Rz inverted, Alphas 0.35/0.15)
// Jitter is reset to 15 to test the new Adaptive Filters.
SystemSettings defaultSettings = {0, 1, 0, 0, 0, 1, 0.35, 0.15, 0, 0, 15, 0x1125}; 

bool at24_present = false;

const int EEPROM_GIMBAL_ADDR = 0; 
const int EEPROM_SETTINGS_ADDR = 50; 

// -------------------- AT24C32 Helpers (Grip Memory) --------------------
#define AT24_PAGE_SIZE 32

bool at24_writeBytes(uint16_t memAddress, const uint8_t *data, uint16_t len) {
  uint16_t written = 0;
  while (written < len) {
    uint16_t chunk = min((uint16_t)(AT24_PAGE_SIZE - (memAddress % AT24_PAGE_SIZE)), (uint16_t)(len - written));
    Wire.beginTransmission(AT24C32_ADDR);
    Wire.write((uint8_t)(memAddress >> 8)); 
    Wire.write((uint8_t)(memAddress & 0xFF)); 
    for (uint16_t i = 0; i < chunk; i++) Wire.write(data[written + i]);
    if (Wire.endTransmission() != 0) return false;
    delay(6);
    memAddress += chunk; written += chunk;
  }
  return true;
}

bool at24_readBytes(uint16_t memAddress, uint8_t *buf, uint16_t len) {
  uint16_t read = 0;
  while (read < len) {
    uint16_t chunk = min((uint16_t)32, (uint16_t)(len - read)); 
    Wire.beginTransmission(AT24C32_ADDR);
    Wire.write((uint8_t)(memAddress >> 8));
    Wire.write((uint8_t)(memAddress & 0xFF));
    if (Wire.endTransmission(false) != 0) return false; 
    Wire.requestFrom(AT24C32_ADDR, chunk);
    uint16_t idx = 0;
    while (Wire.available() && idx < chunk) buf[read + idx++] = Wire.read();
    if (idx != chunk) return false;
    read += chunk; memAddress += chunk;
  }
  return true;
}

bool saveCalibJSToAT24() {
  uint8_t buf[sizeof(CalibJS)];
  memcpy(buf, &calibJS, sizeof(CalibJS));
  return at24_writeBytes(0, buf, sizeof(CalibJS));
}

bool loadCalibJSFromAT24() {
  uint8_t buf[sizeof(CalibJS)];
  if (!at24_readBytes(0, buf, sizeof(CalibJS))) return false;
  memcpy(&calibJS, buf, sizeof(CalibJS));
  return (calibJS.magic == defaultJS.magic);
}

// -------------------- EEPROM Helpers (Base Memory) --------------------
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

void saveGimbalToEEPROM() { EEPROM.put(EEPROM_GIMBAL_ADDR, calibGimbal); }

bool loadGimbalFromEEPROM() {
  CalibGimbal tmp;
  EEPROM.get(EEPROM_GIMBAL_ADDR, tmp);
  if (tmp.magic == defaultGimbal.magic) { calibGimbal = tmp; return true; }
  return false;
}

// -------------------- Utilities & Advanced Filtering --------------------

void updateButtonState(int buttonIndex, bool state) {
  if (buttonIndex < 0 || buttonIndex >= NUM_BUTTONS) return;
  if (state != lastButtonStates[buttonIndex]) {
    Joystick.setButton(buttonIndex, state);
    lastButtonStates[buttonIndex] = state;
  }
}

// Hysteresis filter to eliminate micro-jitter in MSFS
int applyHysteresis(int current, int &last, int threshold) {
  if (abs(current - last) >= threshold) {
    last = current;
  }
  return last;
}

// Adaptive EMA Filter + Bit Truncation
void applyAdaptiveEMA(long rawValue, float &filteredValue, float baseAlpha) {
  // 1. Bit Shifting: Mute the lowest 3 bits (removes baseline static EMI noise)
  long truncatedRaw = (rawValue >> 3) << 3;
  
  // 2. Adaptive Alpha: Calculate the delta between new raw and current filtered
  float diff = abs(truncatedRaw - filteredValue);
  float dynAlpha = baseAlpha;
  
  if (diff < 32) {
    // Minimal movement: Drop alpha to 10% of its value to freeze the axis
    dynAlpha = baseAlpha * 0.1;
  } else if (diff > 256) {
    // Fast movement: Override alpha to near 1.0 for zero input lag
    dynAlpha = 0.85;
  }
  
  // 3. Apply the filter
  filteredValue = dynAlpha * truncatedRaw + (1.0 - dynAlpha) * filteredValue;
}

void printStatus() {
  Serial.println("\n=== JOYSTICK STATUS ===");
  Serial.println("--- Settings ---");
  Serial.print("ATB1 (Ministick->Hat): "); Serial.println(sysSettings.atb1 ? "ON" : "OFF");
  Serial.print("ATB2 (Brake->Button) : "); Serial.println(sysSettings.atb2 ? "ON" : "OFF");
  Serial.print("Filter 1 (Gimbal X,Y) Alpha : "); Serial.println(sysSettings.alpha1);
  Serial.print("Filter 2 (Grip Z,Rx,Ry,Rz) Alpha: "); Serial.println(sysSettings.alpha2);
  Serial.print("Jitter Threshold (Hysteresis) : "); Serial.println(sysSettings.jitter_threshold);
  
  Serial.println("\n--- Axis Inversions ---");
  Serial.print("X: "); Serial.print(sysSettings.invX ? "INV" : "NORM");
  Serial.print(" | Y: "); Serial.print(sysSettings.invY ? "INV" : "NORM");
  Serial.print(" | Z: "); Serial.println(sysSettings.invZ ? "INV" : "NORM");
  Serial.print("Rx: "); Serial.print(sysSettings.invRx ? "INV" : "NORM");
  Serial.print(" | Ry: "); Serial.print(sysSettings.invRy ? "INV" : "NORM");
  Serial.print(" | Rz: "); Serial.println(sysSettings.invRz ? "INV" : "NORM");

  Serial.println("\n--- Calibration Ranges ---");
  Serial.print("Gimbal X : ["); Serial.print(calibGimbal.x_min); Serial.print(", "); Serial.print(calibGimbal.x_max); Serial.println("]");
  Serial.print("Gimbal Y : ["); Serial.print(calibGimbal.y_min); Serial.print(", "); Serial.print(calibGimbal.y_max); Serial.println("]");
  
  if (!calibJS.disabled_z) { Serial.print("Twist  Z : ["); Serial.print(calibJS.zmin); Serial.print(", "); Serial.print(calibJS.zmax); Serial.println("]"); } else { Serial.println("Twist  Z : [DISABLED]"); }
  if (!calibJS.disabled_rx) { Serial.print("Mini  Rx : ["); Serial.print(calibJS.rxmin); Serial.print(", "); Serial.print(calibJS.rxmax); Serial.println("]"); } else { Serial.println("Mini  Rx : [DISABLED]"); }
  if (!calibJS.disabled_ry) { Serial.print("Mini  Ry : ["); Serial.print(calibJS.rymin); Serial.print(", "); Serial.print(calibJS.rymax); Serial.println("]"); } else { Serial.println("Mini  Ry : [DISABLED]"); }
  if (!calibJS.disabled_rz) { Serial.print("Brake Rz : ["); Serial.print(calibJS.rzmin); Serial.print(", "); Serial.print(calibJS.rzmax); Serial.println("]"); } else { Serial.println("Brake Rz : [DISABLED]"); }
  Serial.println("=======================\n");
}

// -------------------- Interactive Calibration --------------------
long readAxisRaw(const char *axis) {
  if (strcmp(axis, "X") == 0) return ADS2.readADC(0);
  if (strcmp(axis, "Y") == 0) return ADS2.readADC(1);
  if (strcmp(axis, "Z") == 0) return ADS.readADC(0);
  if (strcmp(axis, "RX") == 0) return ADS.readADC(1);
  if (strcmp(axis, "RY") == 0) return ADS.readADC(2);
  if (strcmp(axis, "RZ") == 0) return ADS.readADC(3);
  return 0;
}

bool calibrateSingleAxis(const char *axisName, long &out_min, long &out_max, bool &out_disabled) {
  Serial.print("Calibrating "); Serial.print(axisName);
  Serial.println(" -> Move min to max. Press ENTER to accept, 'd' to disable.");
  long current = readAxisRaw(axisName);
  long vmin = current, vmax = current;
  unsigned long startTime = millis();
  
  while(Serial.available()) Serial.read(); // Flush buffer
  
  while (true) {
    current = readAxisRaw(axisName);
    if (current < vmin) vmin = current;
    if (current > vmax) vmax = current;

    if (millis() - startTime > 250) {
      Serial.print(axisName); Serial.print(" min:"); Serial.print(vmin); Serial.print(" max:"); Serial.println(vmax);
      startTime = millis();
    }

    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) {
        out_min = vmin; out_max = vmax; out_disabled = false;
        Serial.println("Saved."); delay(200); return true;
      } else if (s.equalsIgnoreCase("d")) {
        out_disabled = true; out_min = 0; out_max = 0;
        Serial.println("Disabled."); delay(200); return true;
      }
    }
  }
  return false;
}

void calibrateGimbalInteractive() {
  Serial.println("\n== Calibrating Gimbal ==");
  long xmin, xmax, ymin, ymax; bool d;
  if (calibrateSingleAxis("X", xmin, xmax, d)) {
    calibGimbal.x_min = (uint16_t)max(0L, min(65535L, xmin)); calibGimbal.x_max = (uint16_t)max(0L, min(65535L, xmax));
  }
  if (calibrateSingleAxis("Y", ymin, ymax, d)) {
    calibGimbal.y_min = (uint16_t)max(0L, min(65535L, ymin)); calibGimbal.y_max = (uint16_t)max(0L, min(65535L, ymax));
  }
  calibGimbal.magic = defaultGimbal.magic;
  saveGimbalToEEPROM();
  Serial.println("Gimbal calibration saved.");
}

void calibrateJoystickInteractive() {
  if (!at24_present) { Serial.println("AT24C32 error."); return; }
  Serial.println("\n== Calibrating Joystick ==");
  long vmin, vmax; bool d;
  if(calibrateSingleAxis("Z", vmin, vmax, d)) { calibJS.zmin = vmin; calibJS.zmax = vmax; calibJS.disabled_z = d; }
  if(calibrateSingleAxis("RX", vmin, vmax, d)) { calibJS.rxmin = vmin; calibJS.rxmax = vmax; calibJS.disabled_rx = d; }
  if(calibrateSingleAxis("RY", vmin, vmax, d)) { calibJS.rymin = vmin; calibJS.rymax = vmax; calibJS.disabled_ry = d; }
  if(calibrateSingleAxis("RZ", vmin, vmax, d)) { calibJS.rzmin = vmin; calibJS.rzmax = vmax; calibJS.disabled_rz = d; }
  calibJS.magic = defaultJS.magic;
  saveCalibJSToAT24();
  Serial.println("Joystick calibration saved.");
}

void triggerCalibrationMenu() {
  Serial.println("\n=== CALIBRATION MENU ===");
  Serial.println("1 = Base (X,Y) | 2 = Joystick (Z,Rx,Ry,Rz)");
  while(Serial.available()) Serial.read(); 
  while (!Serial.available()) { delay(10); }
  String choice = Serial.readStringUntil('\n'); choice.trim();
  if (choice == "1") calibrateGimbalInteractive();
  else if (choice == "2") calibrateJoystickInteractive();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  Wire.begin();
  Wire.setClock(400000); // Set I2C to 400kHz Fast Mode for lower latency
  
  Wire.beginTransmission(AT24C32_ADDR);
  at24_present = (Wire.endTransmission() == 0);

  // Initialize ADCs and set Data Rate to 860 Samples Per Second
  ADS.begin();
  ADS.setDataRate(7); 
  ADS2.begin();
  ADS2.setDataRate(7); 

  pcf8575_1.begin(); pcf8575_2.begin();
  pcf8575_1.setButtonMask(0xFFFF); pcf8575_2.setButtonMask(0xFFFF);

  pinMode(SWITCH_POS1, INPUT_PULLUP);
  pinMode(SWITCH_POS2, INPUT_PULLUP);
  pinMode(LED_ATB_PIN, OUTPUT);
  pinMode(LED_HAPTIC_PIN, OUTPUT);

  if (drv.begin()) { drv.selectLibrary(1); drv.setMode(DRV2605_MODE_INTTRIG); }

  // Fixed Output Resolution +/- 16384 to prevent game engine overflow
  Joystick.setXAxisRange(-16384, 16384);
  Joystick.setYAxisRange(-16384, 16384);
  Joystick.setZAxisRange(-16384, 16384);
  Joystick.setRxAxisRange(-16384, 16384);
  Joystick.setRyAxisRange(-16384, 16384);
  Joystick.setRzAxisRange(-16384, 16384);

  Joystick.begin(false); // Enable manual state updates

  if (!loadGimbalFromEEPROM()) { calibGimbal = defaultGimbal; saveGimbalToEEPROM(); }
  if (at24_present && !loadCalibJSFromAT24()) { calibJS = defaultJS; saveCalibJSToAT24(); }
  loadSettings();
}

// -------------------- Main Loop --------------------
void loop() {
  
  // 1. Serial Command Parser
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();
    
    if (cmd == "CAL") {
      triggerCalibrationMenu();
    } else if (cmd == "STATUS") { 
      printStatus();
    } else if (cmd == "INV_X")  { sysSettings.invX = !sysSettings.invX; saveSettings(); Serial.println("X Inverted"); }
      else if (cmd == "INV_Y")  { sysSettings.invY = !sysSettings.invY; saveSettings(); Serial.println("Y Inverted"); }
      else if (cmd == "INV_Z")  { sysSettings.invZ = !sysSettings.invZ; saveSettings(); Serial.println("Z Inverted"); }
      else if (cmd == "INV_RX") { sysSettings.invRx = !sysSettings.invRx; saveSettings(); Serial.println("Rx Inverted"); }
      else if (cmd == "INV_RY") { sysSettings.invRy = !sysSettings.invRy; saveSettings(); Serial.println("Ry Inverted"); }
      else if (cmd == "INV_RZ") { sysSettings.invRz = !sysSettings.invRz; saveSettings(); Serial.println("Rz Inverted"); }
      else if (cmd == "ATB1")   { sysSettings.atb1 = !sysSettings.atb1; saveSettings(); Serial.println(sysSettings.atb1 ? "ATB1 ON" : "ATB1 OFF"); }
      else if (cmd == "ATB2")   { sysSettings.atb2 = !sysSettings.atb2; saveSettings(); Serial.println(sysSettings.atb2 ? "ATB2 ON" : "ATB2 OFF"); }
      else if (cmd.startsWith("FIL1 ")) { sysSettings.alpha1 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha1 (Gimbal) set to: "); Serial.println(sysSettings.alpha1); }
      else if (cmd.startsWith("FIL2 ")) { sysSettings.alpha2 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha2 (Grip) set to: "); Serial.println(sysSettings.alpha2); }
      else if (cmd.startsWith("JITTER ")) { sysSettings.jitter_threshold = cmd.substring(7).toInt(); saveSettings(); Serial.print("Jitter Deadband set to: "); Serial.println(sysSettings.jitter_threshold); }
  }

  // ATB Status LED
  digitalWrite(LED_ATB_PIN, (sysSettings.atb1 || sysSettings.atb2) ? HIGH : LOW);

  // -------------------- Axes Logic (Filtered, Mapped & Hysteresis applied) --------------------
  
  // X Axis (Gimbal)
  long xRaw = ADS2.readADC(0);
  applyAdaptiveEMA(xRaw, xFiltered, sysSettings.alpha1);
  int xConst = constrain((int)xFiltered, calibGimbal.x_min, calibGimbal.x_max);
  int xAxis = map(xConst, calibGimbal.x_min, calibGimbal.x_max, -16384, 16384);
  if(sysSettings.invX) xAxis *= -1;
  Joystick.setXAxis(applyHysteresis(xAxis, last_xAxis, sysSettings.jitter_threshold));

  // Y Axis (Gimbal)
  long yRaw = ADS2.readADC(1);
  applyAdaptiveEMA(yRaw, yFiltered, sysSettings.alpha1);
  int yConst = constrain((int)yFiltered, calibGimbal.y_min, calibGimbal.y_max);
  int yAxis = map(yConst, calibGimbal.y_min, calibGimbal.y_max, -16384, 16384);
  if(sysSettings.invY) yAxis *= -1;
  Joystick.setYAxis(applyHysteresis(yAxis, last_yAxis, sysSettings.jitter_threshold));
  
  // Haptic G force Calculation
  int G = abs(map(yAxis, -16384, 16384, -512, 512));

  // Z Axis (Grip)
  long zRaw = ADS.readADC(0);
  applyAdaptiveEMA(zRaw, zFiltered, sysSettings.alpha2);
  int zAxisVal = 0;
  if (!calibJS.disabled_z) {
    int zConst = constrain((int)zFiltered, calibJS.zmin, calibJS.zmax);
    zAxisVal = map(zConst, calibJS.zmin, calibJS.zmax, -16384, 16384);
    if(sysSettings.invZ) zAxisVal *= -1;
    Joystick.setZAxis(applyHysteresis(zAxisVal, last_zAxis, sysSettings.jitter_threshold));
  } else {
    Joystick.setZAxis(0);
  }

  // Rx Axis (Grip)
  long rxRaw = ADS.readADC(1);
  applyAdaptiveEMA(rxRaw, rxFiltered, sysSettings.alpha2);
  int rxAxisVal = 0;
  if (!calibJS.disabled_rx) {
    int rxConst = constrain((int)rxFiltered, calibJS.rxmin, calibJS.rxmax);
    rxAxisVal = map(rxConst, calibJS.rxmin, calibJS.rxmax, -16384, 16384);
    if(sysSettings.invRx) rxAxisVal *= -1;
  }

  // Ry Axis (Grip)
  long ryRaw = ADS.readADC(2);
  applyAdaptiveEMA(ryRaw, ryFiltered, sysSettings.alpha2);
  int ryAxisVal = 0;
  if (!calibJS.disabled_ry) {
    int ryConst = constrain((int)ryFiltered, calibJS.rymin, calibJS.rymax);
    ryAxisVal = map(ryConst, calibJS.rymin, calibJS.rymax, -16384, 16384);
    if(sysSettings.invRy) ryAxisVal *= -1;
  }

  // Rz Axis (Grip)
  long rzRaw = ADS.readADC(3);
  applyAdaptiveEMA(rzRaw, rzFiltered, sysSettings.alpha2);
  int rzAxisVal = 0;
  if (!calibJS.disabled_rz) {
    int rzConst = constrain((int)rzFiltered, calibJS.rzmin, calibJS.rzmax);
    rzAxisVal = map(rzConst, calibJS.rzmin, calibJS.rzmax, -16384, 16384);
    if(sysSettings.invRz) rzAxisVal *= -1;
  }

  // -------------------- ATB 1: Mini-stick to Hat --------------------
  if (sysSettings.atb1) {
    int hatDir = -1;
    bool up    = (ryAxisVal >  8000); 
    bool down  = (ryAxisVal < -8000);
    bool right = (rxAxisVal >  8000);
    bool left  = (rxAxisVal < -8000);

    if (up && !right && !left)         hatDir = 0;
    else if (up && right)              hatDir = 45;
    else if (!up && !down && right)    hatDir = 90;
    else if (down && right)            hatDir = 135;
    else if (down && !right && !left)  hatDir = 180;
    else if (down && left)             hatDir = 225;
    else if (!up && !down && left)     hatDir = 270;
    else if (up && left)               hatDir = 315;
    Joystick.setHatSwitch(0, hatDir);
    
    Joystick.setRxAxis(0);
    Joystick.setRyAxis(0);
  } else {
    Joystick.setHatSwitch(0, -1);
    Joystick.setRxAxis(applyHysteresis(rxAxisVal, last_rxAxis, sysSettings.jitter_threshold));
    Joystick.setRyAxis(applyHysteresis(ryAxisVal, last_ryAxis, sysSettings.jitter_threshold));
  }

  // -------------------- ATB 2: Brake to Button --------------------
  if (sysSettings.atb2) {
    static bool brake_lastState = false;
    if (rzAxisVal > 8000 && !brake_lastState) { 
      Joystick.setButton(27, 1);
      brake_lastState = true;
    } else if (rzAxisVal <= 8000 && brake_lastState) {
      Joystick.setButton(27, 0);
      brake_lastState = false;
    }
    Joystick.setRzAxis(0); 
  } else {
    Joystick.setButton(27, 0);
    Joystick.setRzAxis(applyHysteresis(rzAxisVal, last_rzAxis, sysSettings.jitter_threshold));
  }

  // -------------------- Read Buttons (PCF8575) --------------------
  uint16_t p1 = pcf8575_1.read16();
  for (int i=0; i<16; i++) updateButtonState(i, !((p1 >> i) & 1));
  
  uint16_t p2 = pcf8575_2.read16();
  updateButtonState(16, !((p2 >> 0) & 1));
  updateButtonState(17, !((p2 >> 1) & 1));
  updateButtonState(18, !((p2 >> 2) & 1));
  updateButtonState(19, !((p2 >> 3) & 1));
  updateButtonState(20, !((p2 >> 4) & 1));
  updateButtonState(21, !((p2 >> 5) & 1));
  updateButtonState(22, !((p2 >> 8) & 1));
  updateButtonState(23, !((p2 >> 9) & 1));
  updateButtonState(24, !((p2 >> 10) & 1));
  updateButtonState(25, !((p2 >> 11) & 1));
  updateButtonState(26, !((p2 >> 12) & 1));

  // -------------------- Haptic Feedback --------------------
  G_only = (digitalRead(SWITCH_POS1) == LOW);
  G_Buttons = (digitalRead(SWITCH_POS2) == LOW);

  if (G_only || G_Buttons) {
    if (G_only && !G_Buttons) analogWrite(LED_HAPTIC_PIN, 128); 
    else if (G_Buttons) analogWrite(LED_HAPTIC_PIN, 255); 

    if (G > 256 && G <= 288) { drv.setWaveform(0, 49); drv.go(); } 
    else if (G > 288 && G <= 320) { drv.setWaveform(0, 48); drv.go(); } 
    else if (G > 320) { drv.setWaveform(0, 47); drv.go(); }

    if (G_Buttons) {
      if (!((p1 >> 1) & 1) || !((p1 >> 2) & 1)) { drv.stop(); drv.setWaveform(0, 76); drv.go(); }
      if (!((p2 >> 0) & 1)) { drv.stop(); drv.setWaveform(0, 47); drv.go(); }
    }
  } else {
    analogWrite(LED_HAPTIC_PIN, 0);
  }

  // Push all updates to PC
  Joystick.sendState();
  delay(5);
}