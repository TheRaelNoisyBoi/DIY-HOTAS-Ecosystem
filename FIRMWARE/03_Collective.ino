/***************************************
  HOTAS Collective - Universal Firmware
  - Hardware: Arduino Pro Micro (ATmega32u4)
  - Sensors: Analog A1 -> Throttle Axis.
  - Expanders: 1x PCF8575 (0x20) + 9 Direct Pins (24 Buttons total).
  - Resolution: +/- 16384.
  - Serial Interface: Calibration, Filters, Hysteresis, Inversions.
***************************************/

#include <Wire.h>
#include <PCF8575.h>
#include <Joystick.h>
#include <EEPROM.h>

// -------------------- Hardware Config --------------------
#define NUM_BUTTONS 24
#define PCF8575_ADDR 0x20
#define SERIAL_BAUD 115200
#define THROTTLE_PIN A1

// Direct button pins mapped to Joystick buttons 0-8
const int directPins[] = {4, 5, 6, 7, 8, 9, 10, 14, 16}; 
const int numDirectPins = 9;

// -------------------- Filters & Smoothing --------------------
float throttleFiltered = 0;
int last_throttleAxis = 0;

// -------------------- I2C Devices --------------------
PCF8575 pcf(PCF8575_ADDR);

// -------------------- Joystick HID --------------------
// HID: 24 buttons, 0 hats, only Throttle axis enabled
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_BUTTONS, 0,        
                   false, false, false,   // X, Y, Z
                   false, false, false,   // Rx, Ry, Rz
                   false, true, false,    // Rudder, Throttle, Accelerator
                   false, false);         // Brake, Steering

int lastButtonStates[NUM_BUTTONS] = {0};

// -------------------- Persistent Memory Structures --------------------
struct CollectiveCalibration {
  uint16_t min;
  uint16_t max;
  uint16_t magic;  
};

struct SystemSettings {
  uint16_t invThr;            // Invert Throttle logic
  float alpha1;               // Filter for Analog A1
  uint16_t jitter_threshold;  // Hysteresis deadband
  uint16_t magic;
};

CollectiveCalibration currentCalib;
SystemSettings sysSettings;

// Defaults (Using your old hardcoded values as safe default baseline)
CollectiveCalibration defaultCalib = {318, 513, 0xCAFE}; 
SystemSettings defaultSettings = {0, 0.35, 15, 0x112B}; 

const int EEPROM_CALIB_ADDR = 0;
const int EEPROM_SETTINGS_ADDR = 20;

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

// Adaptive EMA Filter (0 bit-shift for 10-bit analog pin)
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
  Serial.println("--- Settings ---");
  Serial.print("Filter 1 (Analog A1) Alpha : "); Serial.println(sysSettings.alpha1);
  Serial.print("Jitter Threshold (Hysteresis) : "); Serial.println(sysSettings.jitter_threshold);
  
  Serial.println("\n--- Axis Inversions ---");
  Serial.print("Throttle: "); Serial.println(sysSettings.invThr ? "INV" : "NORM");

  Serial.println("\n--- Calibration Ranges ---");
  Serial.print("Throttle A1 : ["); Serial.print(currentCalib.min); Serial.print(", "); Serial.print(currentCalib.max); Serial.println("]");
  Serial.println("=======================\n");
}

// -------------------- Interactive Calibration --------------------
void calibrateCollective() {
  Serial.println("\n=== CALIBRATING COLLECTIVE ===");
  Serial.println(">>> Move Throttle from MIN to MAX. Press ENTER to save. <<<");
  
  int vmin = 32767;
  int vmax = -32768;
  unsigned long startTime = millis();

  while(Serial.available()) Serial.read(); 

  while (true) {
    int raw = analogRead(THROTTLE_PIN);
    if (raw < vmin) vmin = raw;
    if (raw > vmax) vmax = raw;

    if (millis() - startTime > 250) {
      Serial.print("Throttle min="); Serial.print(vmin); Serial.print(" max="); Serial.println(vmax);
      startTime = millis();
    }

    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) {
        currentCalib.min = vmin;
        currentCalib.max = vmax;
        saveCalibration();
        Serial.println("Saved. Resuming..."); delay(200); return;
      }
    }
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000); 

  pcf.begin();
  pcf.setButtonMask(0xFFFF); 

  // Initialize Direct Pins
  for (int i = 0; i < numDirectPins; i++) {
    pinMode(directPins[i], INPUT_PULLUP);
  }

  // Protected 16-bit mapped range 
  Joystick.setThrottleRange(-16384, 16384);
  Joystick.begin(false);
  
  loadCalibration();
  loadSettings();
  
  throttleFiltered = analogRead(THROTTLE_PIN);
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
    } else if (cmd == "INV_THR") { sysSettings.invThr = !sysSettings.invThr; saveSettings(); Serial.println("Throttle Inverted"); }
      else if (cmd.startsWith("FIL1 ")) { sysSettings.alpha1 = cmd.substring(5).toFloat(); saveSettings(); Serial.print("Alpha1 set to: "); Serial.println(sysSettings.alpha1); }
      else if (cmd.startsWith("JITTER ")) { sysSettings.jitter_threshold = cmd.substring(7).toInt(); saveSettings(); Serial.print("Jitter Deadband set to: "); Serial.println(sysSettings.jitter_threshold); }
  }

  // 2. Read Direct Buttons (Pins 4 to 10, 14, 16) mapped to Joy 0-8
  for (int i = 0; i < numDirectPins; i++) {
    updateButtonState(i, digitalRead(directPins[i]) == LOW);
  }

  // 3. Read PCF8575 Buttons mapped to Joy 9-23
  uint16_t pcfStates = pcf.read16();
  for (int i = 0; i < 15; i++) {
    updateButtonState(i + numDirectPins, !((pcfStates >> i) & 1));
  }

  // 4. Axis Logic (Adaptive Filter, Mapping & Hysteresis)
  long rawThr = analogRead(THROTTLE_PIN);
  applyAdaptiveEMA(rawThr, throttleFiltered, sysSettings.alpha1, 0); 
  
  int thrConst = constrain((int)throttleFiltered, currentCalib.min, currentCalib.max);
  int thrAxis = map(thrConst, currentCalib.min, currentCalib.max, -16384, 16384);
  
  if (sysSettings.invThr) thrAxis *= -1;
  
  Joystick.setThrottle(applyHysteresis(thrAxis, last_throttleAxis, sysSettings.jitter_threshold));

  Joystick.sendState();
  delay(5);
}