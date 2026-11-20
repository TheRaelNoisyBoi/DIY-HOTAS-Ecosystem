/***************************************
  HOTAS Throttle Universal Firmware (v2.1 - Low Latency, Low RAM 20.11.2026)
  - Hardware: Arduino Pro Micro (ATmega32u4)
  - Sensors: ADS1115 (0x4A) -> Z, Rx, Ry, Rz. Analog A0/A1 -> X, Y.
  - Expanders: 2x PCF8575 (0x20, 0x21) for 31 Buttons.
  - Resolution: +/- 16384.
  - High-Speed I2C (400kHz). ADS1115 at 475 SPS, non-blocking round-robin scan.
  - Analog X/Y: 16x oversampling (10-bit -> 12-bit effective, 0..4092),
    driven by a non-blocking internal ADC state machine that also
    measures Vcc via the bandgap (A0 -> A1 -> Vcc -> A0 ...).
  - Filtering: Median-of-3 spike rejection + One Euro adaptive low-pass
    (strong smoothing at rest, minimal lag when moving) + Hysteresis.
  - Ratiometric supply compensation for ADS axes (A0/A1 are already
    ratiometric through the AVcc reference).
  - HID reports at 1 kHz, buttons scanned every 2 ms.
  - RAM: all text lives in flash (F()/PSTR), no String class (fixed buffer).

  Serial commands (v2):
    CAL, STATUS, VCC
    INV_X, INV_Y, INV_Z, INV_RX, INV_RY, INV_RZ, ATB1
    FIL1 <Hz>   Analog X,Y One Euro min cutoff   [0.05 - 20]  lower = smoother at rest
    FIL2 <Hz>   ADS Z,Rx,Ry,Rz min cutoff        [0.05 - 20]
    BETA1 <v>   Analog X,Y speed coefficient     [0 - 50]     higher = less lag when moving
    BETA2 <v>   ADS Z,Rx,Ry,Rz speed coefficient [0 - 50]
    JITTER <n>  Output hysteresis (HID units)
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

// Pro Micro internal ADC channels: A0 = ADC7, A1 = ADC6
#define ADC_CH_X 7
#define ADC_CH_Y 6

// -------------------- Timing --------------------
#define ADS_DATA_RATE       6        // 475 SPS: less internal noise than 860 SPS
#define ADS_CONV_GATE_US    2000     // don't poll the ADS before its conversion can be done
#define REPORT_INTERVAL_US  1000     // HID report period (1 kHz)
#define BUTTON_INTERVAL_US  2000     // PCF8575 scan period
#define D_CUTOFF_HZ         1.0      // One Euro derivative low-pass cutoff

#define OVERSAMPLE_N        16       // 16 x 10-bit -> 12-bit effective
#define BANDGAP_SETTLE_US   1000     // bandgap settling after mux switch
#define VCC_SAMPLES         4

#define CMD_BUF_SIZE        24

// -------------------- Axis Indices --------------------
enum { AX_X = 0, AX_Y, AX_Z, AX_RX, AX_RY, AX_RZ, NUM_AXES };

// -------------------- Supply Compensation (Ratiometric) --------------------
#define VCC_NOMINAL_MV  5000.0
#define BANDGAP_MV      1100.0
#define VCC_EMA_ALPHA   0.01
#define BANDGAP_ADMUX   (_BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))
float vccFilt = VCC_NOMINAL_MV;

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

uint32_t buttonStates = 0;             // bitfield instead of int[31]

int last_xAxis = 0, last_yAxis = 0, last_zAxis = 0;
int last_rxAxis = 0, last_ryAxis = 0, last_rzAxis = 0;

char cmdBuf[CMD_BUF_SIZE];

// -------------------- Persistent Memory Structures --------------------
struct ThrottleCalibration {
  uint16_t min[6]; // X, Y (12-bit oversampled), Z, Rx, Ry, Rz (ADS counts)
  uint16_t max[6];
  uint16_t magic;  
};

struct SystemSettings {
  uint16_t invX, invY, invZ, invRx, invRy, invRz;
  float minCut1;              // Analog X, Y: One Euro min cutoff (Hz)
  float beta1;                // Analog X, Y: speed coefficient
  float minCut2;              // ADS Z, Rx, Ry, Rz: min cutoff (Hz)
  float beta2;                // ADS: speed coefficient
  uint8_t atb1;               // ATB Mode status
  uint16_t jitter_threshold;  // Output hysteresis (HID units)
  uint16_t magic;
};

ThrottleCalibration currentCalib;
SystemSettings sysSettings;

// Defaults kept in flash (both magics bumped for v2: X/Y now 12-bit, settings layout changed)
const ThrottleCalibration defaultCalib PROGMEM = {
  {0, 0, 0, 13350, 12700, 11080},          
  {4092, 4092, 26000, 21350, 21850, 23500},  
  0xCB01                                   
};
const SystemSettings defaultSettings PROGMEM = {0, 0, 0, 0, 0, 0, 1.5, 3.0, 1.0, 2.0, 0, 4, 0x212A}; 

#define CALIB_MAGIC     0xCB01
#define SETTINGS_MAGIC  0x212A

const int EEPROM_CALIB_ADDR = 0;
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

// -------------------- Async ADS Scanner --------------------
// ADS input channel n -> axis (AX_Z + n)
uint8_t adsCh = 0;
bool adsPending = false;
unsigned long adsT0 = 0;

// -------------------- Internal ADC State Machine --------------------
enum { IADC_X = 0, IADC_Y, IADC_VCC, IADC_STAGES };
uint8_t iadcStage = IADC_X;
int8_t iadcCount = -1;        // -1 = first conversion after mux switch is discarded
uint32_t iadcSum = 0;
bool iadcRunning = false;
unsigned long iadcStageUs = 0;

unsigned long lastReportUs = 0, lastButtonsUs = 0;

// -------------------- EEPROM Helpers --------------------
void saveCalibration() {
  currentCalib.magic = CALIB_MAGIC;
  EEPROM.put(EEPROM_CALIB_ADDR, currentCalib);
  Serial.println(F("Calibration saved to Internal EEPROM."));
}

void loadCalibration() {
  EEPROM.get(EEPROM_CALIB_ADDR, currentCalib);
  if (currentCalib.magic != CALIB_MAGIC) {
    memcpy_P(&currentCalib, &defaultCalib, sizeof(ThrottleCalibration));
    saveCalibration(); 
  }
}

void saveSettings() {
  sysSettings.magic = SETTINGS_MAGIC;
  EEPROM.put(EEPROM_SETTINGS_ADDR, sysSettings);
  Serial.println(F("System settings saved."));
}

void loadSettings() {
  EEPROM.get(EEPROM_SETTINGS_ADDR, sysSettings);
  if (sysSettings.magic != SETTINGS_MAGIC) {
    memcpy_P(&sysSettings, &defaultSettings, sizeof(SystemSettings));
    saveSettings();
  }
}

// -------------------- Supply Compensation Helpers --------------------
// Blocking bandgap read, only used once at boot to prime the filter
void vccMonitorBegin() {
  ADMUX = BANDGAP_ADMUX;
  ADCSRB &= ~_BV(MUX5);
  delay(2);
  ADCSRA |= _BV(ADSC); while (ADCSRA & _BV(ADSC));   // discard first conversion
  float sum = 0;
  for (int i = 0; i < 64; i++) {
    ADCSRA |= _BV(ADSC); while (ADCSRA & _BV(ADSC));
    sum += BANDGAP_MV * 1023.0 / ADC;
  }
  vccFilt = sum / 64.0;
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

// Feed one new raw sample into an axis: spike rejection, normalization
// through calibration, then One Euro filter (cutoff rises with speed).
void axisPushSample(uint8_t i, long raw) {
  AxisState &a = axes[i];

  if (!a.histInit) { a.hist[0] = a.hist[1] = a.hist[2] = raw; a.histInit = true; }
  a.hist[0] = a.hist[1]; a.hist[1] = a.hist[2]; a.hist[2] = raw;
  long med = median3(a.hist[0], a.hist[1], a.hist[2]);

  long mn = currentCalib.min[i], mx = currentCalib.max[i];
  float norm = (mx > mn) ? (2.0f * (float)(med - mn) / (float)(mx - mn) - 1.0f) : 0.0f;

  unsigned long now = micros();
  if (!a.init) { a.x = norm; a.dx = 0; a.init = true; a.lastUs = now; return; }

  float dt = (now - a.lastUs) * 1e-6f;
  a.lastUs = now;
  dt = constrain(dt, 1e-4f, 0.1f);

  bool analogAxis = (i == AX_X || i == AX_Y);
  float minCut = analogAxis ? sysSettings.minCut1 : sysSettings.minCut2;
  float beta   = analogAxis ? sysSettings.beta1   : sysSettings.beta2;

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
void adsRequest() {
  ADS.requestADC(adsCh);
  adsT0 = micros();
  adsPending = true;
}

// Collect a finished conversion (if any) and immediately start the next channel
void adsService() {
  if (!adsPending) { adsRequest(); return; }
  if (micros() - adsT0 < ADS_CONV_GATE_US) return;   // too early, skip I2C poll
  if (ADS.isBusy()) return;
  long raw = ADS.getValue();
  axisPushSample(AX_Z + adsCh, compensate(raw));
  adsCh = (adsCh + 1) % 4;
  adsRequest();
}

// -------------------- Internal ADC State Machine --------------------
// Cycles A0 (16 samples) -> A1 (16 samples) -> bandgap (4 samples) without
// ever blocking: each call either harvests a finished conversion or starts one.
void iadcSelectStage(uint8_t st) {
  iadcStage = st;
  if (st == IADC_X)      ADMUX = _BV(REFS0) | (ADC_CH_X & 0x07);
  else if (st == IADC_Y) ADMUX = _BV(REFS0) | (ADC_CH_Y & 0x07);
  else                   ADMUX = BANDGAP_ADMUX;
  ADCSRB &= ~_BV(MUX5);
  iadcCount = -1;
  iadcSum = 0;
  iadcStageUs = micros();
}

void iadcService() {
  if (iadcRunning) {
    if (ADCSRA & _BV(ADSC)) return;            // conversion in progress
    iadcRunning = false;
    uint16_t v = ADC;
    if (iadcCount >= 0) iadcSum += v;          // first sample after switch discarded
    iadcCount++;

    uint8_t needed = (iadcStage == IADC_VCC) ? VCC_SAMPLES : OVERSAMPLE_N;
    if (iadcCount >= needed) {
      if (iadcStage == IADC_VCC) {
        float avg = iadcSum / (float)VCC_SAMPLES;
        if (avg > 0) vccFilt += VCC_EMA_ALPHA * ((BANDGAP_MV * 1023.0 / avg) - vccFilt);
      } else {
        // Sum of 16 x 10-bit, >> 2 -> 12-bit effective (0..4092)
        axisPushSample(iadcStage == IADC_X ? AX_X : AX_Y, (long)(iadcSum >> 2));
      }
      iadcSelectStage((iadcStage + 1) % IADC_STAGES);
    }
  }

  // Give the bandgap time to settle before converting it
  if (iadcStage == IADC_VCC && (micros() - iadcStageUs) < BANDGAP_SETTLE_US) return;

  ADCSRA |= _BV(ADSC);
  iadcRunning = true;
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

void printInv(bool v) { Serial.print(v ? F("INV") : F("NORM")); }

void printRange(const __FlashStringHelper *label, uint8_t i) {
  Serial.print(label); Serial.print(F(" : ["));
  Serial.print(currentCalib.min[i]); Serial.print(F(", "));
  Serial.print(currentCalib.max[i]); Serial.println(F("]"));
}

void printStatus() {
  Serial.println(F("\n=== THROTTLE STATUS ==="));
  Serial.println(F("--- Settings ---"));
  Serial.print(F("ATB Mode (X/Y -> Hat): ")); Serial.println(sysSettings.atb1 ? F("ON") : F("OFF"));
  Serial.print(F("Filter 1 (Analog X,Y) MinCutoff Hz   : ")); Serial.println(sysSettings.minCut1);
  Serial.print(F("Filter 1 (Analog X,Y) Beta           : ")); Serial.println(sysSettings.beta1);
  Serial.print(F("Filter 2 (ADS Z,Rx,Ry,Rz) MinCutoff Hz: ")); Serial.println(sysSettings.minCut2);
  Serial.print(F("Filter 2 (ADS Z,Rx,Ry,Rz) Beta        : ")); Serial.println(sysSettings.beta2);
  Serial.print(F("Jitter Threshold (Hysteresis) : ")); Serial.println(sysSettings.jitter_threshold);
  
  Serial.println(F("\n--- Axis Inversions ---"));
  Serial.print(F("X: "));    printInv(sysSettings.invX);
  Serial.print(F(" | Y: ")); printInv(sysSettings.invY);
  Serial.print(F(" | Z: ")); printInv(sysSettings.invZ); Serial.println();
  Serial.print(F("Rx: "));    printInv(sysSettings.invRx);
  Serial.print(F(" | Ry: ")); printInv(sysSettings.invRy);
  Serial.print(F(" | Rz: ")); printInv(sysSettings.invRz); Serial.println();

  Serial.println(F("\n--- Calibration Ranges ---"));
  printRange(F("Analog X"), 0);
  printRange(F("Analog Y"), 1);
  printRange(F("ADS Z   "), 2);
  printRange(F("ADS Rx  "), 3);
  printRange(F("ADS Ry  "), 4);
  printRange(F("ADS Rz  "), 5);
  Serial.println(F("=======================\n"));
}

// -------------------- Interactive Calibration --------------------
// Blocking reads (async pipelines are paused during calibration)
long readAnalogOversampledBlocking(uint8_t pin) {
  analogRead(pin);                             // discard after mux switch
  uint32_t sum = 0;
  for (uint8_t i = 0; i < OVERSAMPLE_N; i++) sum += analogRead(pin);
  return (long)(sum >> 2);                     // 12-bit effective, same scale as runtime
}

long readRawHardware(uint8_t axisIndex) {
  switch (axisIndex) {
    case 0: return readAnalogOversampledBlocking(A0);
    case 1: return readAnalogOversampledBlocking(A1);
    case 2: return compensate(ADS.readADC(0));
    case 3: return compensate(ADS.readADC(1));
    case 4: return compensate(ADS.readADC(2));
    default: return compensate(ADS.readADC(3));
  }
}

void calibrateSingleAxis(const __FlashStringHelper *axisName, uint8_t axisIndex) {
  Serial.print(F("Calibrating Axis: ")); Serial.println(axisName);
  Serial.println(F(">>> Move axis from MIN to MAX. Press ENTER to save. <<<"));
  
  long vmin = 2147483647L;
  long vmax = -2147483647L;
  unsigned long startTime = millis();

  flushSerialInput();

  while (true) {
    long raw = readRawHardware(axisIndex);
    if (raw < vmin) vmin = raw;
    if (raw > vmax) vmax = raw;

    if (millis() - startTime > 250) {
      Serial.print(axisName); Serial.print(F(" min=")); Serial.print(vmin);
      Serial.print(F(" max=")); Serial.println(vmax);
      startTime = millis();
    }

    if (Serial.available()) {
      if (readLine() == 0) {
        currentCalib.min[axisIndex] = clampU16(vmin);
        currentCalib.max[axisIndex] = clampU16(vmax);
        Serial.println(F("Saved.")); delay(200); return;
      }
    }
  }
}

void runCalibrationMenu() {
  Serial.println(F("\n=== CALIBRATION MODE ==="));
  calibrateSingleAxis(F("X"), 0);
  calibrateSingleAxis(F("Y"), 1);
  calibrateSingleAxis(F("Z"), 2);
  calibrateSingleAxis(F("Rx"), 3);
  calibrateSingleAxis(F("Ry"), 4);
  calibrateSingleAxis(F("Rz"), 5);
  saveCalibration();

  // Restart async pipelines and filters with the new calibration
  adsCh = 0; adsPending = false;
  iadcRunning = false;
  iadcSelectStage(IADC_X);
  resetAxisFilters();
  Serial.println(F("Done. Resuming..."));
}

// -------------------- Serial Command Parser --------------------
void toggleSetting(uint16_t &flag, const __FlashStringHelper *msg) {
  flag = !flag; saveSettings(); Serial.println(msg);
}

void handleSerial() {
  if (Serial.available() <= 0) return;
  if (readLine() == 0) return;
  
  if (cmdIs(PSTR("CAL"))) {
    runCalibrationMenu();
  } else if (cmdIs(PSTR("STATUS"))) { 
    printStatus();
  } else if (cmdIs(PSTR("VCC"))) {
    Serial.print(F("VCC (mV): ")); Serial.println(vccFilt, 0);
  }
  else if (cmdIs(PSTR("INV_X")))  toggleSetting(sysSettings.invX,  F("X Inverted"));
  else if (cmdIs(PSTR("INV_Y")))  toggleSetting(sysSettings.invY,  F("Y Inverted"));
  else if (cmdIs(PSTR("INV_Z")))  toggleSetting(sysSettings.invZ,  F("Z Inverted"));
  else if (cmdIs(PSTR("INV_RX"))) toggleSetting(sysSettings.invRx, F("Rx Inverted"));
  else if (cmdIs(PSTR("INV_RY"))) toggleSetting(sysSettings.invRy, F("Ry Inverted"));
  else if (cmdIs(PSTR("INV_RZ"))) toggleSetting(sysSettings.invRz, F("Rz Inverted"));
  else if (cmdIs(PSTR("ATB1"))) { sysSettings.atb1 = !sysSettings.atb1; saveSettings(); Serial.println(sysSettings.atb1 ? F("ATB1 ON") : F("ATB1 OFF")); }
  else if (cmdStarts(PSTR("FIL1 ")))  { sysSettings.minCut1 = constrain(atof(cmdBuf + 5), 0.05, 20.0); saveSettings(); Serial.print(F("MinCutoff1 (Analog) Hz: ")); Serial.println(sysSettings.minCut1); }
  else if (cmdStarts(PSTR("FIL2 ")))  { sysSettings.minCut2 = constrain(atof(cmdBuf + 5), 0.05, 20.0); saveSettings(); Serial.print(F("MinCutoff2 (ADS) Hz: ")); Serial.println(sysSettings.minCut2); }
  else if (cmdStarts(PSTR("BETA1 "))) { sysSettings.beta1 = constrain(atof(cmdBuf + 6), 0.0, 50.0); saveSettings(); Serial.print(F("Beta1 (Analog): ")); Serial.println(sysSettings.beta1); }
  else if (cmdStarts(PSTR("BETA2 "))) { sysSettings.beta2 = constrain(atof(cmdBuf + 6), 0.0, 50.0); saveSettings(); Serial.print(F("Beta2 (ADS): ")); Serial.println(sysSettings.beta2); }
  else if (cmdStarts(PSTR("JITTER "))) { sysSettings.jitter_threshold = atoi(cmdBuf + 7); saveSettings(); Serial.print(F("Jitter Deadband set to: ")); Serial.println(sysSettings.jitter_threshold); }
}

// -------------------- Buttons --------------------
void readButtons() {
  uint16_t p1 = pcf8575_1.read16();
  for (uint8_t i = 0; i < 16; i++) updateButtonState(i, !((p1 >> i) & 1));
  
  uint16_t p2 = pcf8575_2.read16();
  for (uint8_t i = 0; i < 15; i++) updateButtonState(i + 16, !((p2 >> i) & 1)); // 31 buttons total
}

// -------------------- Axis Outputs (Mapping, ATB, Hysteresis) --------------------
void updateAxisOutputs() {
  int xAxis = axisOutput(AX_X, sysSettings.invX);
  int yAxis = axisOutput(AX_Y, sysSettings.invY);

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

  Joystick.setZAxis (applyHysteresis(axisOutput(AX_Z,  sysSettings.invZ),  last_zAxis,  sysSettings.jitter_threshold));
  Joystick.setRxAxis(applyHysteresis(axisOutput(AX_RX, sysSettings.invRx), last_rxAxis, sysSettings.jitter_threshold));
  Joystick.setRyAxis(applyHysteresis(axisOutput(AX_RY, sysSettings.invRy), last_ryAxis, sysSettings.jitter_threshold));
  Joystick.setRzAxis(applyHysteresis(axisOutput(AX_RZ, sysSettings.invRz), last_rzAxis, sysSettings.jitter_threshold));
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(50);       // never stall the loop on a partial command
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast I2C

  if (!ADS.begin()) {
    Serial.println(F("Error: ADS1115 not connected"));
  } else {
    ADS.setGain(1); // 1 = 4.096V
    ADS.setDataRate(ADS_DATA_RATE);
    Serial.println(F("ADS1115 Connected & Optimized."));
  }

  vccMonitorBegin();           // prime Vcc before first compensated read
  iadcSelectStage(IADC_X);     // start internal ADC state machine

  pcf8575_1.begin();
  pcf8575_2.begin();
  pcf8575_1.setButtonMask(0xFFFF);
  pcf8575_2.setButtonMask(0xFFFF);

  Joystick.setXAxisRange(-16384, 16384);
  Joystick.setYAxisRange(-16384, 16384);
  Joystick.setZAxisRange(-16384, 16384);
  Joystick.setRxAxisRange(-16384, 16384);
  Joystick.setRyAxisRange(-16384, 16384);
  Joystick.setRzAxisRange(-16384, 16384);

  Joystick.begin(false);
  
  loadCalibration();
  loadSettings();
  resetAxisFilters();
}

// -------------------- Main Loop (non-blocking) --------------------
void loop() {
  handleSerial();
  iadcService();
  adsService();

  unsigned long now = micros();

  if (now - lastButtonsUs >= BUTTON_INTERVAL_US) {
    lastButtonsUs = now;
    readButtons();
  }

  if (now - lastReportUs >= REPORT_INTERVAL_US) {
    lastReportUs = now;
    updateAxisOutputs();
    Joystick.sendState();
  }
}
