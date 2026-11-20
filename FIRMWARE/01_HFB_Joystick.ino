/***************************************
  HOTAS Universal Firmware - HFB Joystick (v2.1 - Low Latency, Low RAM  20.11.2026)
  - Serial Calibration & Settings Interface.
  - Axes Resolution: +/- 16384 (Overflow protected).
  - ADS1115: ADS (0x48) -> Z,Rx,Ry,Rz ; ADS2 (0x49) -> X,Y.
  - Calibration: Send "CAL" via Serial.
  - High-Speed I2C (400kHz). ADS1115 at 475 SPS, non-blocking round-robin
    scan: both ADCs convert in parallel while the loop keeps running.
  - Filtering: Median-of-3 spike rejection + One Euro adaptive low-pass
    (strong smoothing at rest, minimal lag when moving) + Hysteresis.
  - Ratiometric supply compensation: USB Vcc measured via internal bandgap,
    all ADS readings normalized to VCC_NOMINAL_MV.
  - HID reports at 1 kHz, buttons scanned every 2 ms.
  - Haptic Modes: Local (G-Force + Buttons) & RTP (Real-Time GUI Control).
    DRV2605 only receives I2C traffic when something actually changes.
  - RAM: all text lives in flash (F()/PSTR), no String class (fixed buffer).

  Serial commands (v2):
    CAL, STATUS, VCC
    INV_X, INV_Y, INV_Z, INV_RX, INV_RY, INV_RZ, ATB1, ATB2
    FIL1 <Hz>   Gimbal (X,Y) One Euro min cutoff  [0.05 - 20]  lower = smoother at rest
    FIL2 <Hz>   Grip (Z,Rx,Ry,Rz) min cutoff      [0.05 - 20]
    BETA1 <v>   Gimbal speed coefficient          [0 - 50]     higher = less lag when moving
    BETA2 <v>   Grip speed coefficient            [0 - 50]
    JITTER <n>  Output hysteresis (HID units)
    HAPTIC_RTP_ON, HAPTIC_RTP_OFF, RTP <0-127>
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

// -------------------- Timing --------------------
#define ADS_DATA_RATE       6        // 475 SPS: less internal noise than 860 SPS
#define ADS_CONV_GATE_US    2000     // don't poll an ADS before its conversion can be done
#define REPORT_INTERVAL_US  1000     // HID report period (1 kHz)
#define BUTTON_INTERVAL_US  2000     // PCF8575 scan period
#define HAPTIC_INTERVAL_US  12000    // local haptic retrigger cadence (matches v1 feel)
#define D_CUTOFF_HZ         1.0      // One Euro derivative low-pass cutoff

#define CMD_BUF_SIZE        24       // longest command is "HAPTIC_RTP_OFF" / "FIL1 12.345"

// -------------------- Axis Indices --------------------
enum { AX_X = 0, AX_Y, AX_Z, AX_RX, AX_RY, AX_RZ, NUM_AXES };

// -------------------- Supply Compensation (Ratiometric) --------------------
// SS49E output scales with Vcc (center = Vcc/2), but ADS1115 measures against
// its fixed internal reference. Vcc is measured with the ATmega32U4 internal
// 1.1V bandgap and every ADS reading is rescaled as if Vcc were nominal.
#define VCC_NOMINAL_MV  5000.0
#define BANDGAP_MV      1100.0
#define VCC_EMA_ALPHA   0.005
float vccFilt = VCC_NOMINAL_MV;

// -------------------- I2C Devices --------------------
PCF8575 pcf8575_1(PCF8575_ADDR_1);
PCF8575 pcf8575_2(PCF8575_ADDR_2);
ADS1115 ADS(0x48);
ADS1115 ADS2(0x49);
Adafruit_DRV2605 drv;

// -------------------- Joystick HID Config --------------------
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NUM_BUTTONS, 1,
                   true, true, true, true, true, true, false, false, false, false, false);

uint32_t buttonStates = 0;             // bitfield instead of int[28] (saves 52 bytes)
uint16_t p1 = 0xFFFF, p2 = 0xFFFF;     // last PCF8575 reads (active low)

int last_xAxis = 0, last_yAxis = 0, last_zAxis = 0;
int last_rxAxis = 0, last_ryAxis = 0, last_rzAxis = 0;
int yAxisOut = 0;                      // used by local G-force haptics

char cmdBuf[CMD_BUF_SIZE];

// -------------------- Haptic State --------------------
enum { HAPTIC_OFF = 0, HAPTIC_LOCAL, HAPTIC_RTP };
bool mode_g_buttons = false;
bool mode_rtp = false;
uint8_t rtp_target_val = 0;
uint8_t lastRtpSent = 0xFF;
uint8_t lastHapticMode = 0xFF;

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
  float minCut1;              // Gimbal (X, Y): One Euro min cutoff (Hz)
  float beta1;                // Gimbal (X, Y): One Euro speed coefficient
  float minCut2;              // Grip (Z, Rx, Ry, Rz): min cutoff (Hz)
  float beta2;                // Grip: speed coefficient
  uint8_t atb1;               // Ministick to Hat Mode
  uint8_t atb2;               // Brake to Button Mode
  uint16_t jitter_threshold;  // Output hysteresis (HID units)
  uint16_t magic;
};

CalibGimbal calibGimbal;
CalibJS calibJS;
SystemSettings sysSettings;

// Defaults kept in flash, copied to RAM only when needed
const CalibGimbal defaultGimbal PROGMEM = {9700, 17000, 9000, 15850, 0xAA55};
const CalibJS defaultJS PROGMEM = {7350, 8755, 0, 32767, 0, 32767, 13000, 22500, 0, 0, 0, 0, 0x55AA};
const SystemSettings defaultSettings PROGMEM = {0, 1, 0, 0, 0, 1, 1.5, 3.0, 1.0, 2.0, 0, 0, 4, 0x2125};

#define GIMBAL_MAGIC    0xAA55
#define JS_MAGIC        0x55AA
#define SETTINGS_MAGIC  0x2125

bool at24_present = false;
const int EEPROM_GIMBAL_ADDR = 0; 
const int EEPROM_SETTINGS_ADDR = 50; 

// -------------------- Axis Filter State --------------------
struct AxisState {
  long hist[3];               // median-of-3 history
  bool histInit;
  float x;                    // filtered value, normalized -1..1
  float dx;                   // filtered derivative (units/s)
  bool init;
  unsigned long lastUs;
};
AxisState axes[NUM_AXES];

// -------------------- Async ADS Scanners --------------------
struct AdsScanner {
  ADS1115 *ads;
  uint8_t firstAxis;          // ADS input channel n -> axis (firstAxis + n)
  uint8_t numCh;
  uint8_t ch;
  bool pending;
  unsigned long t0;
};
AdsScanner scanGimbal = {&ADS2, AX_X, 2, 0, false, 0};   // ch0 -> X, ch1 -> Y
AdsScanner scanGrip   = {&ADS,  AX_Z, 4, 0, false, 0};   // ch0..3 -> Z, Rx, Ry, Rz

unsigned long lastReportUs = 0, lastButtonsUs = 0, lastHapticUs = 0;

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
  return at24_writeBytes(0, (const uint8_t *)&calibJS, sizeof(CalibJS));
}

bool loadCalibJSFromAT24() {
  if (!at24_readBytes(0, (uint8_t *)&calibJS, sizeof(CalibJS))) return false;
  return (calibJS.magic == JS_MAGIC);
}

// -------------------- EEPROM Helpers (Base Memory) --------------------
void saveSettings() {
  sysSettings.magic = SETTINGS_MAGIC;
  EEPROM.put(EEPROM_SETTINGS_ADDR, sysSettings);
  Serial.println(F("System settings saved."));
}

void loadSettings() {
  EEPROM.get(EEPROM_SETTINGS_ADDR, sysSettings);
  if (sysSettings.magic != SETTINGS_MAGIC) {
    memcpy_P(&sysSettings, &defaultSettings, sizeof(SystemSettings));  // v1 layout incompatible: reset
    saveSettings();
  }
}

void saveGimbalToEEPROM() { EEPROM.put(EEPROM_GIMBAL_ADDR, calibGimbal); }

bool loadGimbalFromEEPROM() {
  EEPROM.get(EEPROM_GIMBAL_ADDR, calibGimbal);
  return (calibGimbal.magic == GIMBAL_MAGIC);
}

// -------------------- Supply Compensation Helpers --------------------
// ATmega32U4 only. analogRead() is never used in this firmware, so the ADC
// mux stays parked on the bandgap channel permanently.
void vccMonitorBegin() {
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // AVcc ref, 1.1V bandgap
  ADCSRB &= ~_BV(MUX5);
  delay(2);                                        // let bandgap settle

  ADCSRA |= _BV(ADSC);                             // discard first conversion
  while (ADCSRA & _BV(ADSC));

  float sum = 0;
  for (int i = 0; i < 64; i++) {
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC));
    sum += BANDGAP_MV * 1023.0 / ADC;
  }
  vccFilt = sum / 64.0;

  ADCSRA |= _BV(ADSC);                             // start first background conversion
}

// Non-blocking: harvest a finished conversion (if any) and start the next one
void vccMonitorUpdate() {
  if (ADCSRA & _BV(ADSC)) return;
  uint16_t adc = ADC;
  if (adc > 0) vccFilt += VCC_EMA_ALPHA * ((BANDGAP_MV * 1023.0 / adc) - vccFilt);
  ADCSRA |= _BV(ADSC);
}

// Rescale a raw ADS reading as if supply were exactly VCC_NOMINAL_MV
long compensate(long raw) {
  return lround((float)raw * VCC_NOMINAL_MV / vccFilt);
}

uint16_t clampU16(long v) {
  return (uint16_t)constrain(v, 0L, 65535L);
}

// -------------------- Filtering (Median-of-3 + One Euro) --------------------
long median3(long a, long b, long c) {
  long lo = min(a, b), hi = max(a, b);
  return max(lo, min(hi, c));
}

float lpAlpha(float cutoffHz, float dt) {
  float tau = 1.0f / (2.0f * PI * cutoffHz);
  return 1.0f / (1.0f + tau / dt);
}

void axisCalib(uint8_t i, long &mn, long &mx) {
  switch (i) {
    case AX_X:  mn = calibGimbal.x_min; mx = calibGimbal.x_max; break;
    case AX_Y:  mn = calibGimbal.y_min; mx = calibGimbal.y_max; break;
    case AX_Z:  mn = calibJS.zmin;      mx = calibJS.zmax;      break;
    case AX_RX: mn = calibJS.rxmin;     mx = calibJS.rxmax;     break;
    case AX_RY: mn = calibJS.rymin;     mx = calibJS.rymax;     break;
    default:    mn = calibJS.rzmin;     mx = calibJS.rzmax;     break;
  }
}

// Feed one new raw sample into an axis: spike rejection, normalization
// through calibration, then One Euro filter (cutoff rises with speed).
void axisPushSample(uint8_t i, long raw) {
  AxisState &a = axes[i];

  if (!a.histInit) { a.hist[0] = a.hist[1] = a.hist[2] = raw; a.histInit = true; }
  a.hist[0] = a.hist[1]; a.hist[1] = a.hist[2]; a.hist[2] = raw;
  long med = median3(a.hist[0], a.hist[1], a.hist[2]);

  long mn, mx;
  axisCalib(i, mn, mx);
  float norm = (mx > mn) ? (2.0f * (float)(med - mn) / (float)(mx - mn) - 1.0f) : 0.0f;

  unsigned long now = micros();
  if (!a.init) { a.x = norm; a.dx = 0; a.init = true; a.lastUs = now; return; }

  float dt = (now - a.lastUs) * 1e-6f;
  a.lastUs = now;
  dt = constrain(dt, 1e-4f, 0.1f);

  bool gimbal = (i == AX_X || i == AX_Y);
  float minCut = gimbal ? sysSettings.minCut1 : sysSettings.minCut2;
  float beta   = gimbal ? sysSettings.beta1   : sysSettings.beta2;

  float dRaw = (norm - a.x) / dt;
  a.dx += lpAlpha(D_CUTOFF_HZ, dt) * (dRaw - a.dx);
  float cutoff = minCut + beta * fabs(a.dx);
  a.x += lpAlpha(cutoff, dt) * (norm - a.x);
}

int axisOutput(uint8_t i, uint16_t invert) {
  float v = constrain(axes[i].x, -1.0f, 1.0f);
  int out = (int)lround(v * 16384.0f);
  return invert ? -out : out;
}

void resetAxisFilters() {
  for (uint8_t i = 0; i < NUM_AXES; i++) { axes[i].histInit = false; axes[i].init = false; }
}

// -------------------- Async ADS Scanning --------------------
void scannerRequest(AdsScanner &s) {
  s.ads->requestADC(s.ch);
  s.t0 = micros();
  s.pending = true;
}

// Collect a finished conversion (if any) and immediately start the next channel
void scannerService(AdsScanner &s) {
  if (!s.pending) { scannerRequest(s); return; }
  if (micros() - s.t0 < ADS_CONV_GATE_US) return;   // too early, skip I2C poll
  if (s.ads->isBusy()) return;
  long raw = s.ads->getValue();
  axisPushSample(s.firstAxis + s.ch, compensate(raw));
  s.ch = (s.ch + 1) % s.numCh;
  scannerRequest(s);
}

void scannerReset(AdsScanner &s) {
  s.ch = 0;
  s.pending = false;
}

// -------------------- Serial Line Input (no String class) --------------------
// Reads one line into cmdBuf, trimmed and upper-cased. Returns its length.
uint8_t readLine() {
  uint8_t n = Serial.readBytesUntil('\n', cmdBuf, CMD_BUF_SIZE - 1);
  cmdBuf[n] = '\0';
  while (n > 0 && (cmdBuf[n - 1] == '\r' || cmdBuf[n - 1] == ' ')) cmdBuf[--n] = '\0';
  uint8_t start = 0;
  while (cmdBuf[start] == ' ') start++;
  if (start) { memmove(cmdBuf, cmdBuf + start, n - start + 1); n -= start; }
  for (uint8_t i = 0; i < n; i++) cmdBuf[i] = toupper(cmdBuf[i]);
  return n;
}

bool cmdIs(const char *pstr)     { return strcmp_P(cmdBuf, pstr) == 0; }
bool cmdStarts(const char *pstr) { return strncmp_P(cmdBuf, pstr, strlen_P(pstr)) == 0; }

void flushSerialInput() { while (Serial.available()) Serial.read(); }

// -------------------- Utilities --------------------
void updateButtonState(uint8_t buttonIndex, bool state) {
  if (buttonIndex >= NUM_BUTTONS) return;
  bool last = (buttonStates >> buttonIndex) & 1UL;
  if (state != last) {
    Joystick.setButton(buttonIndex, state);
    if (state) buttonStates |= (1UL << buttonIndex);
    else       buttonStates &= ~(1UL << buttonIndex);
  }
}

int applyHysteresis(int current, int &last, int threshold) {
  if (abs(current - last) >= threshold) {
    last = current;
  }
  return last;
}

void printOnOff(bool v) { Serial.println(v ? F("ON") : F("OFF")); }
void printInv(bool v)   { Serial.print(v ? F("INV") : F("NORM")); }

void printStatus() {
  Serial.println(F("\n=== JOYSTICK STATUS ==="));
  Serial.print(F("Haptic Mode: "));
  if (mode_rtp) Serial.println(F("RTP (Real-Time GUI)"));
  else if (mode_g_buttons) Serial.println(F("Local (G-Force + Buttons)"));
  else Serial.println(F("OFF / Neutral"));

  Serial.println(F("--- Settings ---"));
  Serial.print(F("ATB1 (Ministick->Hat): ")); printOnOff(sysSettings.atb1);
  Serial.print(F("ATB2 (Brake->Button) : ")); printOnOff(sysSettings.atb2);
  Serial.print(F("Filter 1 (Gimbal X,Y) MinCutoff Hz : ")); Serial.println(sysSettings.minCut1);
  Serial.print(F("Filter 1 (Gimbal X,Y) Beta         : ")); Serial.println(sysSettings.beta1);
  Serial.print(F("Filter 2 (Grip Z,Rx,Ry,Rz) MinCutoff Hz: ")); Serial.println(sysSettings.minCut2);
  Serial.print(F("Filter 2 (Grip Z,Rx,Ry,Rz) Beta        : ")); Serial.println(sysSettings.beta2);
  Serial.print(F("Jitter Threshold (Hysteresis) : ")); Serial.println(sysSettings.jitter_threshold);
  
  Serial.println(F("\n--- Axis Inversions ---"));
  Serial.print(F("X: "));    printInv(sysSettings.invX);
  Serial.print(F(" | Y: ")); printInv(sysSettings.invY);
  Serial.print(F(" | Z: ")); printInv(sysSettings.invZ); Serial.println();
  Serial.print(F("Rx: "));    printInv(sysSettings.invRx);
  Serial.print(F(" | Ry: ")); printInv(sysSettings.invRy);
  Serial.print(F(" | Rz: ")); printInv(sysSettings.invRz); Serial.println();
  Serial.println(F("=======================\n"));
}

// -------------------- Interactive Calibration --------------------
// Blocking compensated reads (async scanners are paused during calibration)
long readAxisRaw(uint8_t axis) {
  vccMonitorUpdate();
  switch (axis) {
    case AX_X:  return compensate(ADS2.readADC(0));
    case AX_Y:  return compensate(ADS2.readADC(1));
    case AX_Z:  return compensate(ADS.readADC(0));
    case AX_RX: return compensate(ADS.readADC(1));
    case AX_RY: return compensate(ADS.readADC(2));
    default:    return compensate(ADS.readADC(3));
  }
}

bool calibrateSingleAxis(const __FlashStringHelper *axisName, uint8_t axis,
                         long &out_min, long &out_max, bool &out_disabled) {
  Serial.print(F("Calibrating ")); Serial.print(axisName);
  Serial.println(F(" -> Move min to max. Press ENTER to accept, 'd' to disable."));
  long current = readAxisRaw(axis);
  long vmin = current, vmax = current;
  unsigned long startTime = millis();
  
  flushSerialInput();
  
  while (true) {
    current = readAxisRaw(axis);
    if (current < vmin) vmin = current;
    if (current > vmax) vmax = current;

    if (millis() - startTime > 250) {
      Serial.print(axisName); Serial.print(F(" min:")); Serial.print(vmin);
      Serial.print(F(" max:")); Serial.println(vmax);
      startTime = millis();
    }

    if (Serial.available()) {
      uint8_t n = readLine();
      if (n == 0) {
        out_min = vmin; out_max = vmax; out_disabled = false;
        Serial.println(F("Saved.")); delay(200); return true;
      } else if (cmdIs(PSTR("D"))) {
        out_disabled = true; out_min = 0; out_max = 0;
        Serial.println(F("Disabled.")); delay(200); return true;
      }
    }
  }
  return false;
}

void calibrateGimbalInteractive() {
  Serial.println(F("\n== Calibrating Gimbal =="));
  long vmin, vmax; bool d;
  if (calibrateSingleAxis(F("X"), AX_X, vmin, vmax, d)) {
    calibGimbal.x_min = clampU16(vmin); calibGimbal.x_max = clampU16(vmax);
  }
  if (calibrateSingleAxis(F("Y"), AX_Y, vmin, vmax, d)) {
    calibGimbal.y_min = clampU16(vmin); calibGimbal.y_max = clampU16(vmax);
  }
  calibGimbal.magic = GIMBAL_MAGIC;
  saveGimbalToEEPROM();
  Serial.println(F("Gimbal calibration saved."));
}

void calibrateJoystickInteractive() {
  if (!at24_present) { Serial.println(F("AT24C32 error.")); return; }
  Serial.println(F("\n== Calibrating Joystick =="));
  long vmin, vmax; bool d;
  if (calibrateSingleAxis(F("Z"),  AX_Z,  vmin, vmax, d)) { calibJS.zmin  = clampU16(vmin); calibJS.zmax  = clampU16(vmax); calibJS.disabled_z  = d; }
  if (calibrateSingleAxis(F("RX"), AX_RX, vmin, vmax, d)) { calibJS.rxmin = clampU16(vmin); calibJS.rxmax = clampU16(vmax); calibJS.disabled_rx = d; }
  if (calibrateSingleAxis(F("RY"), AX_RY, vmin, vmax, d)) { calibJS.rymin = clampU16(vmin); calibJS.rymax = clampU16(vmax); calibJS.disabled_ry = d; }
  if (calibrateSingleAxis(F("RZ"), AX_RZ, vmin, vmax, d)) { calibJS.rzmin = clampU16(vmin); calibJS.rzmax = clampU16(vmax); calibJS.disabled_rz = d; }
  calibJS.magic = JS_MAGIC;
  saveCalibJSToAT24();
  Serial.println(F("Joystick calibration saved."));
}

void triggerCalibrationMenu() {
  Serial.println(F("\n=== CALIBRATION MENU ==="));
  Serial.println(F("1 = Base (X,Y) | 2 = Joystick (Z,Rx,Ry,Rz)"));
  flushSerialInput();
  while (!Serial.available()) { delay(10); }
  readLine();
  if (cmdIs(PSTR("1")))      calibrateGimbalInteractive();
  else if (cmdIs(PSTR("2"))) calibrateJoystickInteractive();

  // Restart async pipeline and filters with the new calibration
  scannerReset(scanGimbal);
  scannerReset(scanGrip);
  resetAxisFilters();
}

// -------------------- Serial Command Parser --------------------
void toggleSetting(uint16_t &flag, const __FlashStringHelper *msg) {
  flag = !flag; saveSettings(); Serial.println(msg);
}

void handleSerial() {
  if (Serial.available() <= 0) return;
  if (readLine() == 0) return;
  
  if (cmdIs(PSTR("CAL"))) {
    triggerCalibrationMenu();
  } else if (cmdIs(PSTR("STATUS"))) { 
    printStatus();
  } else if (cmdIs(PSTR("VCC"))) {
    Serial.print(F("VCC (mV): ")); Serial.println(vccFilt, 0);
  } else if (cmdIs(PSTR("HAPTIC_RTP_ON"))) {
    mode_rtp = true;
    drv.setMode(DRV2605_MODE_REALTIME);
    lastRtpSent = 0xFF;                       // force resend of current value
    Serial.println(F("Haptic Mode set to RTP (GUI Real-Time)"));
  } else if (cmdIs(PSTR("HAPTIC_RTP_OFF"))) {
    mode_rtp = false;
    drv.setMode(DRV2605_MODE_INTTRIG);
    Serial.println(F("Haptic Mode RTP disabled"));
  } else if (cmdStarts(PSTR("RTP "))) {
    rtp_target_val = (uint8_t)constrain(atoi(cmdBuf + 4), 0, 127);
  }
  else if (cmdIs(PSTR("INV_X")))  toggleSetting(sysSettings.invX,  F("X Inverted"));
  else if (cmdIs(PSTR("INV_Y")))  toggleSetting(sysSettings.invY,  F("Y Inverted"));
  else if (cmdIs(PSTR("INV_Z")))  toggleSetting(sysSettings.invZ,  F("Z Inverted"));
  else if (cmdIs(PSTR("INV_RX"))) toggleSetting(sysSettings.invRx, F("Rx Inverted"));
  else if (cmdIs(PSTR("INV_RY"))) toggleSetting(sysSettings.invRy, F("Ry Inverted"));
  else if (cmdIs(PSTR("INV_RZ"))) toggleSetting(sysSettings.invRz, F("Rz Inverted"));
  else if (cmdIs(PSTR("ATB1"))) { sysSettings.atb1 = !sysSettings.atb1; saveSettings(); Serial.println(sysSettings.atb1 ? F("ATB1 ON") : F("ATB1 OFF")); }
  else if (cmdIs(PSTR("ATB2"))) { sysSettings.atb2 = !sysSettings.atb2; saveSettings(); Serial.println(sysSettings.atb2 ? F("ATB2 ON") : F("ATB2 OFF")); }
  else if (cmdStarts(PSTR("FIL1 ")))  { sysSettings.minCut1 = constrain(atof(cmdBuf + 5), 0.05, 20.0); saveSettings(); Serial.print(F("MinCutoff1 (Gimbal) Hz: ")); Serial.println(sysSettings.minCut1); }
  else if (cmdStarts(PSTR("FIL2 ")))  { sysSettings.minCut2 = constrain(atof(cmdBuf + 5), 0.05, 20.0); saveSettings(); Serial.print(F("MinCutoff2 (Grip) Hz: ")); Serial.println(sysSettings.minCut2); }
  else if (cmdStarts(PSTR("BETA1 "))) { sysSettings.beta1 = constrain(atof(cmdBuf + 6), 0.0, 50.0); saveSettings(); Serial.print(F("Beta1 (Gimbal): ")); Serial.println(sysSettings.beta1); }
  else if (cmdStarts(PSTR("BETA2 "))) { sysSettings.beta2 = constrain(atof(cmdBuf + 6), 0.0, 50.0); saveSettings(); Serial.print(F("Beta2 (Grip): ")); Serial.println(sysSettings.beta2); }
  else if (cmdStarts(PSTR("JITTER "))) { sysSettings.jitter_threshold = atoi(cmdBuf + 7); saveSettings(); Serial.print(F("Jitter Deadband set to: ")); Serial.println(sysSettings.jitter_threshold); }
}

// -------------------- Buttons --------------------
void readButtons() {
  p1 = pcf8575_1.read16();
  for (uint8_t i = 0; i < 16; i++) updateButtonState(i, !((p1 >> i) & 1));
  
  p2 = pcf8575_2.read16();
  for (uint8_t i = 0; i < 6; i++) updateButtonState(16 + i, !((p2 >> i) & 1));        // P0..P5  -> 16..21
  for (uint8_t i = 0; i < 5; i++) updateButtonState(22 + i, !((p2 >> (8 + i)) & 1));  // P10..P14 -> 22..26
}

// -------------------- Axis Outputs (Mapping, ATB, Hysteresis) --------------------
void updateAxisOutputs() {
  // X / Y (Gimbal)
  int xAxis = axisOutput(AX_X, sysSettings.invX);
  Joystick.setXAxis(applyHysteresis(xAxis, last_xAxis, sysSettings.jitter_threshold));
  int yAxis = axisOutput(AX_Y, sysSettings.invY);
  Joystick.setYAxis(applyHysteresis(yAxis, last_yAxis, sysSettings.jitter_threshold));
  yAxisOut = yAxis;

  // Z (Grip)
  if (!calibJS.disabled_z) {
    int zAxisVal = axisOutput(AX_Z, sysSettings.invZ);
    Joystick.setZAxis(applyHysteresis(zAxisVal, last_zAxis, sysSettings.jitter_threshold));
  } else {
    Joystick.setZAxis(0);
  }

  int rxAxisVal = calibJS.disabled_rx ? 0 : axisOutput(AX_RX, sysSettings.invRx);
  int ryAxisVal = calibJS.disabled_ry ? 0 : axisOutput(AX_RY, sysSettings.invRy);
  int rzAxisVal = calibJS.disabled_rz ? 0 : axisOutput(AX_RZ, sysSettings.invRz);

  // ATB 1: Mini-stick to Hat
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

  // ATB 2: Brake to Button
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
}

// -------------------- Haptics --------------------
// I2C traffic to the DRV2605 only on mode transitions, RTP value changes,
// or at the local-mode retrigger cadence.
void serviceHaptics(unsigned long now) {
  mode_g_buttons = (digitalRead(SWITCH_POS2) == LOW);
  uint8_t mode = mode_rtp ? HAPTIC_RTP : (mode_g_buttons ? HAPTIC_LOCAL : HAPTIC_OFF);

  if (mode != lastHapticMode) {
    if (mode == HAPTIC_OFF)        { drv.stop(); analogWrite(LED_HAPTIC_PIN, 0); }
    else if (mode == HAPTIC_LOCAL) { analogWrite(LED_HAPTIC_PIN, 128); }
    else                           { analogWrite(LED_HAPTIC_PIN, 255); lastRtpSent = 0xFF; }
    lastHapticMode = mode;
  }

  if (mode == HAPTIC_RTP) {
    // MODE 2: Real-Time Playback, value pushed only when GUI changes it
    if (rtp_target_val != lastRtpSent) {
      drv.setRealtimeValue(rtp_target_val);
      lastRtpSent = rtp_target_val;
    }
  } else if (mode == HAPTIC_LOCAL) {
    // MODE 1: Local Autonomous (G-Force + Buttons), fixed retrigger cadence
    if (now - lastHapticUs < HAPTIC_INTERVAL_US) return;
    lastHapticUs = now;

    int G = abs(map(yAxisOut, -16384, 16384, -512, 512));
    if (G > 256 && G <= 288)      { drv.setWaveform(0, 49); drv.go(); } 
    else if (G > 288 && G <= 320) { drv.setWaveform(0, 48); drv.go(); } 
    else if (G > 320)             { drv.setWaveform(0, 47); drv.go(); }

    if (!((p1 >> 1) & 1) || !((p1 >> 2) & 1)) { drv.stop(); drv.setWaveform(0, 76); drv.go(); }
    if (!((p2 >> 0) & 1)) { drv.stop(); drv.setWaveform(0, 47); drv.go(); }
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50);       // never stall the loop on a partial command
  
  Wire.begin();
  Wire.setClock(400000); 
  
  Wire.beginTransmission(AT24C32_ADDR);
  at24_present = (Wire.endTransmission() == 0);

  ADS.begin();
  ADS.setDataRate(ADS_DATA_RATE); 
  ADS2.begin();
  ADS2.setDataRate(ADS_DATA_RATE); 

  vccMonitorBegin();           // must run before first compensated read

  pcf8575_1.begin(); pcf8575_2.begin();
  pcf8575_1.setButtonMask(0xFFFF); pcf8575_2.setButtonMask(0xFFFF);

  pinMode(SWITCH_POS1, INPUT_PULLUP);
  pinMode(SWITCH_POS2, INPUT_PULLUP);
  pinMode(LED_ATB_PIN, OUTPUT);
  pinMode(LED_HAPTIC_PIN, OUTPUT);

  if (drv.begin()) { 
    drv.selectLibrary(1); 
    drv.setMode(DRV2605_MODE_INTTRIG); // Default startup mode
  }

  Joystick.setXAxisRange(-16384, 16384);
  Joystick.setYAxisRange(-16384, 16384);
  Joystick.setZAxisRange(-16384, 16384);
  Joystick.setRxAxisRange(-16384, 16384);
  Joystick.setRyAxisRange(-16384, 16384);
  Joystick.setRzAxisRange(-16384, 16384);

  Joystick.begin(false); 

  if (!loadGimbalFromEEPROM()) {
    memcpy_P(&calibGimbal, &defaultGimbal, sizeof(CalibGimbal));
    saveGimbalToEEPROM();
  }
  if (at24_present && !loadCalibJSFromAT24()) {
    memcpy_P(&calibJS, &defaultJS, sizeof(CalibJS));
    saveCalibJSToAT24();
  } else if (!at24_present) {
    memcpy_P(&calibJS, &defaultJS, sizeof(CalibJS));   // grip absent: safe defaults
  }
  loadSettings();

  resetAxisFilters();
}

// -------------------- Main Loop (non-blocking) --------------------
void loop() {
  handleSerial();
  vccMonitorUpdate();

  // Both ADS1115 convert in parallel; each call only does I2C when needed
  scannerService(scanGimbal);
  scannerService(scanGrip);

  unsigned long now = micros();

  if (now - lastButtonsUs >= BUTTON_INTERVAL_US) {
    lastButtonsUs = now;
    readButtons();
  }

  if (now - lastReportUs >= REPORT_INTERVAL_US) {
    lastReportUs = now;
    digitalWrite(LED_ATB_PIN, (sysSettings.atb1 || sysSettings.atb2) ? HIGH : LOW);
    updateAxisOutputs();
    Joystick.sendState();
  }

  serviceHaptics(now);
}

