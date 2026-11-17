/***************************************************************************************
  Noisy Boi FFB base - FFB_Setup
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Noisy Boi Designs

  Setup assistant for a new (or repaired) base. Upload it, open the Serial Monitor
  (115200 baud, line ending "Newline") and type SETUP, or use the SETUP page of the
  Tactical MFD Control Hub. It runs the steps that are not done yet, in order, and stores
  every result in the Pro Micro EEPROM:

    Step 1  Electronics   I2C devices, encoder magnets, encoder reads, grip    motors OFF
    Step 2  Calibration   centre, travel limits, encoder directions             motors OFF
    Step 3  Motors        E-Stop, motor directions, encoder noise               max 25 %
    Step 4  Measurement   IBT-2 dead time and friction -> DTX / DTY             max 30 %

  If something fails, fix it and type SETUP again: it continues where it stopped.
  When the four steps are done, SETUP offers a travel test (a visual check: a short push
  per axis checks the motor direction, then the motors move the stick slowly from end to
  end). Then upload FFB_Joystick.

  Later changes:
    - Recalibration (a part was moved or replaced): upload this sketch, type CAL, then
      upload FFB_Joystick again. If an encoder direction changed, CAL offers a short
      re-check of the motor directions (step 3 is not done until it passes). The axis
      inversion for the sim (INV_AX / INV_AY of FFB_Joystick) is kept.
    - After ANY change to the motors, the IBT-2 drivers or their wiring: type MOTORS.

  Commands:
    SETUP      run the steps that are not done yet
    CHECK      step 1    CAL    step 2    MOTORS    step 3    MEASURE    step 4
    GRIP       grip test (buttons, axes, vibration)
    TRAVEL     travel test: direction check, then the motors move the stick slowly from
               end to end, Y then X (also offered at the end of SETUP). Every 100 ms it
               prints @TP target stick duty (HID, HID, duty counts).
    MON        live values: encoders, X / Y, grip axes, buttons (any key stops)
    STATUS     progress and stored configuration (copy it for support)
    RESET      erase the stored configuration and the curves (asks for YES)

  C + Enter at any prompt cancels the running command: nothing is stored and the motors
  are switched off.

  The motors are enabled only inside MOTORS, MEASURE and TRAVEL, after you type YES, and
  are disabled again at the end. Any key stops a running test. Lines that start with '@'
  are for the GUI and can be ignored in the Serial Monitor.

  No extra library: the grip chips (ADS1115, PCF8575) are driven directly.
  Board: Arduino Micro (or SparkFun Pro Micro 5 V / 16 MHz).
***************************************************************************************/
#include <Wire.h>
#include <EEPROM.h>
#include <setjmp.h>

#define FW_NAME "FFB_Setup"

// ######################################################################################
// SHARED CODE - identical in FFB_Setup and FFB_Joystick
// ######################################################################################

// ================================================================
// HARDWARE (reference build)
// ================================================================
//
// FFB_Setup and FFB_Joystick are written and tested for this exact hardware. Other motors,
// drivers or mechanics may work, but are the user's responsibility.
//
//   MCU        Arduino Pro Micro (ATmega32U4, 5 V, 16 MHz)
//   Motors     2x RS-775, 24 V, 4500 rpm, 5.3 A max at stall
//   Drivers    2x IBT-2 (BTS7960), PWM 20 kHz, both EN pins on D7
//   Reduction  GT2 belt, 12T / 190T (1:15.83)
//   Encoders   2x AS5600 on the gimbal, behind a TCA9548A mux
//   Power      24 V 15 A supply, E-Stop relay (40 A) on the 24 V line
//   Grip       ADS1115, 2x PCF8575, AT24C32, DRV2605L

#define NB_VERSION       "1.1.5"   // FFB_Setup and FFB_Joystick must have the same version

// ---------------- Pins ----------------
#define NB_PIN_X_RPWM    5     // Timer3 OC3A
#define NB_PIN_X_LPWM    6     // Timer4 OC4D
#define NB_PIN_SAFETY    7     // IBT-2 EN (both drivers), HIGH = enabled
#define NB_PIN_Y_RPWM    9     // Timer1 OC1A
#define NB_PIN_Y_LPWM    10    // Timer1 OC1B

// ---------------- I2C ----------------
#define NB_I2C_CLOCK     400000UL
#define NB_ADDR_TCA      0x70  // TCA9548A mux (ch0 = X encoder, ch1 = Y encoder)
#define NB_ADDR_AS5600   0x36  // behind the mux
#define NB_ADDR_PCF1     0x20  // grip buttons 1-16
#define NB_ADDR_PCF2     0x21  // grip buttons 17-27
#define NB_ADDR_ADS      0x48  // grip axes Rx, Ry, Rz
#define NB_ADDR_AT24     0x50  // grip calibration EEPROM
#define NB_ADDR_DRV      0x5A  // DRV2605L haptic driver (optional)
#define NB_CH_X          0
#define NB_CH_Y          1

// ---------------- Misc ----------------
#define NB_SERIAL_BAUD   115200
#define NB_PWM_TOP       799   // 16 MHz / 800 = 20 kHz; duty counts 0..799 everywhere
#define NB_AXIS_MAX      1023  // HID axis range 0..1023, centre 511
#define NB_NUM_BUTTONS   28


// ================================================================
// CONFIGURATION (internal EEPROM)
// ================================================================
//
// One block with a CRC. FFB_Setup writes the calibration, the motor directions and the
// measurements; FFB_Joystick reads them and stores your tuning (gains, drive values).
// A step refuses to run until the step before it is done, and FFB_Joystick refuses START
// until all four setup steps are done.
//
// The block lives at NB_EE_ADDR (64): the data of the old v10.x firmware (addresses 0-49)
// is left untouched. FFB_Joystick keeps its spring gain curves in a second block with its
// own CRC at NB_EE_CURVE (FFB_Setup RESET invalidates it).
//
// Three separate directions per axis:
//   invAX / invAY   encoder direction (DIR_X / DIR_Y): set by calibration so that the
//                   calibration frame follows the Windows convention (X right = up,
//                   Y forward = down). Never set by hand.
//   invMX / invMY   motor direction (MDIR_X / MDIR_Y): measured by setup step 3 in the
//                   calibration frame. Never set by hand.
//   uinvX / uinvY   axis inversion for the sim (INV_AX / INV_AY in FFB_Joystick, CTRL > INV
//                   in the GUI). Applied by FFB_Joystick only, on top of the calibration
//                   frame: it flips the HID axis and the force of that axis together.

#define NB_EE_ADDR   64
#define NB_EE_CURVE  128
#define NB_MAGIC     0x4E42      // "NB"
#define NB_LAYOUT    1

// Setup steps: bit n of cfg.steps = step n+1 done
#define NB_ST_ELECTRONICS   0     // step 1
#define NB_ST_CALIBRATION   1     // step 2
#define NB_ST_MOTORS        2     // step 3
#define NB_ST_MEASURE       3     // step 4
#define NB_STEPS            4
#define NB_ST_ALL           0x0F

struct NbConfig {
  uint16_t magic;
  uint8_t  layout;
  uint8_t  steps;             // bit n = setup step n+1 done

  // Setup step 2: calibration
  uint16_t centerX, centerY;  // raw AS5600 angle at the centre of travel (0..4095)
  uint16_t minX, maxX;        // travel limits, unwrapped counts (centre = 2048)
  uint16_t minY, maxY;
  uint8_t  invAX, invAY;      // encoder direction (calibration frame = Windows convention)

  // Setup step 3: motors
  uint8_t  invMX, invMY;      // motor direction (positive force = towards increasing HID)

  // Setup step 4: measurements (shown by STATUS; the results go to dtX / dtY)
  uint16_t tdNs[2];           // IBT-2 dead time per axis, ns
  uint8_t  fric10[2][2];      // friction, 0.1 % duty, [axis][0 = + direction, 1 = - direction]

  // Drive (dtX / dtY measured by setup step 4; everything adjustable in FFB_Joystick)
  uint16_t dtX, dtY;          // dead-time offset, 0.1 % duty (140 = 14.0 %)
  uint8_t  dutyCap;           // hard cap of the final duty, %
  uint8_t  minEff;            // effective duty below this is zero, 0.1 % (hysteresis: off at half)
  uint8_t  slew;              // force slew limit, force units per ms (0 = off)
  uint8_t  ffbMs;             // force update period, ms
  uint8_t  velDamp;           // velocity damping (0 = off)

  // FFB gains, % of what the sim sends (100 = unchanged)
  uint8_t  tg, sg, dg, ig, fg;   // total, spring, damper, inertia, friction
  uint8_t  pg, cg;               // periodic effects, constant force
  uint8_t  pf;                   // periodic frequency, % (100 = unchanged)
  uint8_t  atb1, atb2;           // grip mini-stick -> HAT, brake axis -> button 28
  uint8_t  uinvX, uinvY;         // axis inversion for the sim (FFB_Joystick only)

  // Grip axes Rx / Ry / Rz (FFB_Joystick only), One Euro filter as in the HFB joystick:
  uint8_t  gFil;                 // min cutoff, 0.1 Hz (10 = 1.0 Hz; 0 = block older than 1.1.2)
  uint8_t  gBeta;                // speed coefficient, 0.1 (20 = 2.0, the BETA2 of the HFB)
  uint8_t  gJit;                 // output hysteresis, HID counts of 1023

  uint8_t  reserved[3];
  uint16_t crc;               // CRC-16/CCITT of everything above
};

NbConfig cfg;

// Reference-hardware defaults. The drive and gain values are the ones tuned on the
// reference base (Nov 2026); setup step 4 replaces dtX / dtY with the values measured
// on each build.
static const NbConfig defaultCfg PROGMEM = {
  NB_MAGIC, NB_LAYOUT, 0,
  2048, 2048, 1024, 3072, 1024, 3072, 0, 0,     // calibration (placeholder until step 2)
  0, 0,                                          // motor inversion (step 3)
  {0, 0}, {{0, 0}, {0, 0}},                      // measurements (step 4)
  140, 140, 30, 5, 4, 2, 0,                      // dtX dtY dutyCap minEff slew ffbMs velDamp
  100, 50, 100, 100, 0,                          // tg sg dg ig fg
  10, 35, 100,                                   // pg cg pf
  1, 1,                                          // atb1 atb2
  0, 0,                                          // uinvX uinvY
  10, 20, 2,                                     // gFil gBeta gJit
  {0, 0, 0},
  0
};

uint16_t nbCrc16(const uint8_t* p, uint16_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static uint16_t cfgCrc() {
  return nbCrc16((const uint8_t*)&cfg, sizeof(NbConfig) - sizeof(uint16_t));
}

// Reference-hardware defaults for the WHOLE block (calibration included, no step done).
void nbConfigDefaults() {
  memcpy_P(&cfg, &defaultCfg, sizeof(NbConfig));
}

// Reference defaults for the drive and gain fields only (keeps calibration, steps and the
// axis inversion for the sim).
void nbConfigDefaultsTuning() {
  NbConfig d;
  memcpy_P(&d, &defaultCfg, sizeof(NbConfig));
  cfg.dtX = d.dtX;  cfg.dtY = d.dtY;  cfg.dutyCap = d.dutyCap;  cfg.minEff = d.minEff;
  cfg.slew = d.slew;  cfg.ffbMs = d.ffbMs;  cfg.velDamp = d.velDamp;
  cfg.tg = d.tg;  cfg.sg = d.sg;  cfg.dg = d.dg;  cfg.ig = d.ig;  cfg.fg = d.fg;
  cfg.pg = d.pg;  cfg.cg = d.cg;  cfg.pf = d.pf;
  cfg.atb1 = d.atb1;  cfg.atb2 = d.atb2;
  memcpy(&cfg.gFil, &d.gFil, 3);                 // grip filter
}

// Loads the block. Returns false (and loads the reference defaults, no step done)
// when the EEPROM holds no valid block.
bool nbConfigLoad() {
  uint8_t* p = (uint8_t*)&cfg;
  for (uint16_t i = 0; i < sizeof(NbConfig); i++) p[i] = EEPROM.read(NB_EE_ADDR + i);
  if (cfg.magic == NB_MAGIC && cfg.layout == NB_LAYOUT && cfg.crc == cfgCrc()) return true;
  nbConfigDefaults();
  return false;
}

// Recomputes the CRC and writes the block (EEPROM.update: only changed bytes are written).
void nbConfigSave() {
  cfg.magic = NB_MAGIC;
  cfg.layout = NB_LAYOUT;
  cfg.crc = cfgCrc();
  const uint8_t* p = (const uint8_t*)&cfg;
  for (uint16_t i = 0; i < sizeof(NbConfig); i++) EEPROM.update(NB_EE_ADDR + i, p[i]);
}

bool nbStepDone(uint8_t s) { return (cfg.steps >> s) & 1; }

// Marks setup step s+1 done or not done and saves the whole block.
void nbStepSet(uint8_t s, bool done) {
  if (done) cfg.steps |= (uint8_t)(1 << s);
  else      cfg.steps &= (uint8_t)~(1 << s);
  nbConfigSave();
}

const char nbStepNames[] PROGMEM = "Electronics\0Calibration\0Motors\0Measurement\0";

// Prints "step 2 (Calibration)" for s = 1.
void nbPrintStep(uint8_t s) {
  PGM_P p = nbStepNames;
  for (uint8_t i = 0; i < s; i++) p += strlen_P(p) + 1;
  Serial.print(F("step ")); Serial.print(s + 1);
  Serial.print(F(" (")); Serial.print((const __FlashStringHelper*)p); Serial.print(')');
}

// Prints "ERROR: step N (name) is not done." for the first missing step in mask.
// Returns true when every step in mask is done.
bool nbRequireSteps(uint8_t mask) {
  for (uint8_t s = 0; s < NB_STEPS; s++) {
    if (((mask >> s) & 1) && !nbStepDone(s)) {
      Serial.print(F("ERROR: setup ")); nbPrintStep(s); Serial.println(F(" is not done."));
      return false;
    }
  }
  return true;
}

// ================================================================
// ENCODERS (AS5600 behind the TCA9548A)
// ================================================================
#define NB_ENC_MAX_JUMP    60   // raw counts per read; full travel is ~430 counts and takes >100 ms
#define NB_ENC_MAX_REJECT  6    // consecutive rejected reads before a new value is accepted

uint16_t nbI2cErr = 0;
uint16_t nbGlitch[2] = {0, 0};
bool     nbEncFilter = true;

static uint16_t lastGood[2] = {2048, 2048};
static uint8_t  rejects[2]  = {0, 0};

// I2C at 400 kHz with a 5 ms timeout: a disturbed bus (motor noise, a loose wire) makes a
// read fail (the last good value is kept) instead of freezing the firmware with the
// motors running.
void nbI2cBegin() {
  Wire.begin();
  Wire.setClock(NB_I2C_CLOCK);
  Wire.setWireTimeout(5000, true);
}

bool nbMuxSelect(uint8_t ch) {
  Wire.beginTransmission(NB_ADDR_TCA);
  Wire.write((uint8_t)(1 << ch));
  return Wire.endTransmission() == 0;
}

// Reads n bytes starting at register reg of the AS5600 on mux channel ch.
static bool as5600Read(uint8_t ch, uint8_t reg, uint8_t* buf, uint8_t n) {
  if (!nbMuxSelect(ch)) return false;
  Wire.beginTransmission(NB_ADDR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom((uint8_t)NB_ADDR_AS5600, n);
  if (Wire.available() != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Raw angle 0..4095. On any I2C failure, or a physically impossible jump, the last good
// angle is returned (a wrong angle would become a force spike).
uint16_t nbReadAngle(uint8_t ch) {
  uint8_t c = ch & 1;
  uint8_t b[2];
  // The mux select is checked: if it were lost (motor EMI) the read would still succeed
  // but return the OTHER axis angle.
  if (!as5600Read(ch, 0x0E, b, 2)) { nbI2cErr++; return lastGood[c]; }
  uint16_t angle = (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);

  if (nbEncFilter) {
    int16_t d = (int16_t)angle - (int16_t)lastGood[c];
    if (d >  2048) d -= 4096;
    if (d < -2048) d += 4096;
    if ((d > NB_ENC_MAX_JUMP || d < -NB_ENC_MAX_JUMP) && rejects[c] < NB_ENC_MAX_REJECT) {
      rejects[c]++;
      nbGlitch[c]++;
      return lastGood[c];
    }
  }
  rejects[c] = 0;
  lastGood[c] = angle;
  return angle;
}

// Seeds the jump filter with one real read per axis (call once after Wire.begin()).
void nbEncBegin() {
  bool f = nbEncFilter;
  nbEncFilter = false;
  nbReadAngle(NB_CH_X);
  nbReadAngle(NB_CH_Y);
  nbEncFilter = f;
}

// Angle relative to an anchor, 0..4096 with the anchor at 2048 (no wrap inside the travel).
int16_t nbUnwrap(uint16_t raw, uint16_t anchor) {
  int16_t diff = (int16_t)raw - (int16_t)anchor;
  if (diff > 2048)  diff -= 4096;
  if (diff < -2048) diff += 4096;
  return diff + 2048;
}

// Unwrapped, calibrated position -> HID 0..1023 in the calibration frame (Windows
// convention). The axis inversion for the sim is applied by FFB_Joystick on top of this.
int16_t nbHidFromCont(uint8_t axis, int16_t cont) {
  uint16_t lo = axis ? cfg.minY : cfg.minX;
  uint16_t hi = axis ? cfg.maxY : cfg.maxX;
  if (hi <= lo + 10) return 511;
  if (cont < (int16_t)lo) cont = lo;
  if (cont > (int16_t)hi) cont = hi;
  int16_t h = (int16_t)map(cont, lo, hi, 0, NB_AXIS_MAX);
  return (axis ? cfg.invAY : cfg.invAX) ? (NB_AXIS_MAX - h) : h;
}

// Convenience: one read of an axis straight to HID (no filtering).
int16_t nbReadHid(uint8_t axis) {
  uint16_t center = axis ? cfg.centerY : cfg.centerX;
  return nbHidFromCont(axis, nbUnwrap(nbReadAngle(axis ? NB_CH_Y : NB_CH_X), center));
}

// AS5600 magnet diagnostics (setup step 1).
//   status bit 5 MD = magnet detected, bit 4 ML = too weak, bit 3 MH = too strong
//   agc 0..255 (5 V): best in the middle of the range; magnitude 0..4095
bool nbMagnetInfo(uint8_t ch, uint8_t* status, uint8_t* agc, uint16_t* magnitude) {
  uint8_t b[2];
  if (!as5600Read(ch, 0x0B, b, 1)) return false;
  *status = b[0];
  if (!as5600Read(ch, 0x1A, b, 1)) return false;
  *agc = b[0];
  if (!as5600Read(ch, 0x1B, b, 2)) return false;
  *magnitude = (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
  return true;
}

// ================================================================
// MOTORS (IBT-2, 20 kHz PWM)
// ================================================================
//
// Duty is always given in counts 0..NB_PWM_TOP (799 = 100 %), signed: positive drives the
// R input of the IBT-2, negative the L input. Motor inversion (cfg.invMX/MY) is applied
// by the CALLER, never here.
//
// IBT-2 / BTS7960 dead time: the driver loses ~6.5 us of every PWM pulse, i.e. ~13 % of
// duty at 20 kHz. nbDriveCounts() adds that offset (cfg.dtX / dtY) to every non-zero
// command so the torque follows the force from almost zero:
//     D = DT + e * (1 - DT),  e = |force| / 255,  D <= dutyCap

bool nbMotorOn = false;
int16_t nbDtCnt[2] = {0, 0}, nbCapCnt = 0, nbMinCnt = 0;

static uint16_t topT13 = NB_PWM_TOP;   // Timer1/Timer3 TOP (changes with nbSetPwmFreq)
static uint8_t  freqKHz = 20;

static void setupTimers() {
  // Timer1: D9 (OC1A), D10 (OC1B), fast PWM, TOP = ICR1
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1 = NB_PWM_TOP; OCR1A = 0; OCR1B = 0;
  // Timer3: D5 (OC3A)
  TCCR3A = (1 << WGM31);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS30);
  ICR3 = NB_PWM_TOP; OCR3A = 0;
  // Timer4 (10 bit): D6 (OC4D), TOP = OCR4C
  TCCR4A = 0;
  TCCR4C = (1 << PWM4D);
  TCCR4B = (1 << CS40);
  TCCR4D = 0;
  TC4H = (NB_PWM_TOP >> 8) & 0x03;
  OCR4C = NB_PWM_TOP & 0xFF;
  TC4H = 0;
  OCR4D = 0;
}

// idx: 0 X_R (D5), 1 X_L (D6), 2 Y_R (D9), 3 Y_L (D10). v: 0..799
static void pwmOut(uint8_t idx, uint16_t v) {
  if (v == 0) {
    // Disconnect the timer and drive the pin LOW (pure 0 V). Direct port writes:
    // D5 = PC6, D6 = PD7, D9 = PB5, D10 = PB6
    switch (idx) {
      case 0: TCCR3A &= ~(1 << COM3A1); PORTC &= ~(1 << PC6); break;
      case 1: TCCR4C &= ~(1 << COM4D1); PORTD &= ~(1 << PD7); break;
      case 2: TCCR1A &= ~(1 << COM1A1); PORTB &= ~(1 << PB5); break;
      case 3: TCCR1A &= ~(1 << COM1B1); PORTB &= ~(1 << PB6); break;
    }
    return;
  }
  if (v > NB_PWM_TOP) v = NB_PWM_TOP;
  uint16_t a = v;   // Timer1/3 counts
  if (topT13 != NB_PWM_TOP) a = (uint16_t)(((uint32_t)v * (topT13 + 1UL)) / (NB_PWM_TOP + 1UL));
  // 16-bit timer registers share a TEMP register with interrupts: write with IRQs off.
  uint8_t sreg = SREG; cli();
  switch (idx) {
    case 0: TCCR3A |= (1 << COM3A1); OCR3A = a; break;
    case 1: TCCR4C |= (1 << COM4D1); TC4H = (uint8_t)((v >> 8) & 0x03); OCR4D = (uint8_t)(v & 0xFF); break;
    case 2: TCCR1A |= (1 << COM1A1); OCR1A = a; break;
    case 3: TCCR1A |= (1 << COM1B1); OCR1B = a; break;
  }
  SREG = sreg;
}

void nbStopMotors() {
  for (uint8_t i = 0; i < 4; i++) pwmOut(i, 0);
}

// pins, D7 LOW, timers at 20 kHz, outputs off
void nbMotorsBegin() {
  // D7 = PE6: LOW first (motors disabled), then output.
  PORTE &= ~(1 << PE6);
  pinMode(NB_PIN_SAFETY, OUTPUT);
  nbMotorOn = false;
  pinMode(NB_PIN_X_RPWM, OUTPUT); pinMode(NB_PIN_X_LPWM, OUTPUT);
  pinMode(NB_PIN_Y_RPWM, OUTPUT); pinMode(NB_PIN_Y_LPWM, OUTPUT);
  setupTimers();
  nbStopMotors();
}

// D7; disabling also forces every output to 0
void nbMotorEnable(bool en) {
  nbMotorOn = en;
  if (en) PORTE |= (1 << PE6); else PORTE &= ~(1 << PE6);
  if (!en) nbStopMotors();
}

// axis 0 = X (D5/D6), 1 = Y (D9/D10); counts -799..799, 0 = both inputs LOW (brake)
void nbSetMotor(uint8_t axis, int16_t counts) {
  uint8_t r = axis ? 2 : 0;
  if (counts > NB_PWM_TOP) counts = NB_PWM_TOP;
  if (counts < -NB_PWM_TOP) counts = -NB_PWM_TOP;
  if (counts > 0)      { pwmOut(r + 1, 0); pwmOut(r, (uint16_t)counts); }
  else if (counts < 0) { pwmOut(r, 0); pwmOut(r + 1, (uint16_t)(-counts)); }
  else                 { pwmOut(r, 0); pwmOut(r + 1, 0); }
}

// 20, 10 or 5 kHz (setup step 4 only). Counts keep their meaning (0..799 = 0..100 %).
bool nbSetPwmFreq(uint8_t khz) {
  uint16_t top; uint8_t cs;
  if      (khz == 20) { top = NB_PWM_TOP; cs = 0x01; }
  else if (khz == 10) { top = 1599;       cs = 0x02; }
  else if (khz == 5)  { top = 3199;       cs = 0x03; }
  else return false;
  nbStopMotors();
  uint8_t sreg = SREG; cli();
  ICR1 = top; ICR3 = top;
  TCNT1 = 0;  TCNT3 = 0;
  TCCR4B = (uint8_t)((TCCR4B & 0xF0) | cs);
  SREG = sreg;
  topT13 = top;
  freqKHz = khz;
  return true;
}

uint8_t nbPwmFreq() { return freqKHz; }

void nbDriveDerive() {
  // 16-bit approximations of x * 799 / 1000 and x * 799 / 100 (error <= 1 count = 0.13 %)
  nbDtCnt[0] = (int16_t)(cfg.dtX * 4 / 5);
  nbDtCnt[1] = (int16_t)(cfg.dtY * 4 / 5);
  nbCapCnt   = (int16_t)(cfg.dutyCap * 8);
  nbMinCnt   = (int16_t)(cfg.minEff * 4 / 5);
}

// force: library scale +-255 (library frame, inversion NOT applied). on: per-axis
// hysteresis state of the MINE threshold (switch on above MINE, off below MINE / 2).
int16_t nbDriveCounts(int16_t force, uint8_t axis, uint8_t* on) {
  int16_t mag = (force < 0) ? -force : force;
  if (mag > 255) mag = 255;
  // eff = mag * 799 / 255 (0..799), compared without the division.
  int32_t m = (int32_t)mag * 799L;
  if (*on) { if (m <= (int32_t)(nbMinCnt / 2) * 255L) *on = 0; }
  else     { if (m >  (int32_t)nbMinCnt * 255L)       *on = 1; }
  if (!*on) return 0;
  int16_t dt = nbDtCnt[axis & 1];
  int16_t d = dt + (int16_t)(((int32_t)mag * (799 - dt)) / 255L);
  if (d > nbCapCnt) d = nbCapCnt;
  return (force > 0) ? d : -d;
}

// ================================================================
// GRIP (ADS1115, PCF8575, AT24C32, DRV2605L)
// ================================================================
// Grip axis calibration stored in the grip's own AT24C32 (v7.0 layout).
struct NbGripCal {
  uint16_t zmin,  zmax;
  uint16_t rxmin, rxmax;
  uint16_t rymin, rymax;
  uint16_t rzmin, rzmax;
  uint16_t disabled_z,  disabled_rx;
  uint16_t disabled_ry, disabled_rz;
  uint16_t magic;
};
#define NB_GRIP_MAGIC 0x55AA

// DRV2605L (direct I2C, RTP mode only)
#define NB_DRV_REG_RTPIN 0x02

// ADS1115 and PCF8575 are driven directly (a few registers only): no extra libraries.
// ADS1115 config: start single shot, AIN(ch) vs GND, +-6.144 V, single-shot mode, 475 SPS,
// comparator off: the settings of the HFB joystick firmware (ADS1X15 library, gain 0, data
// rate 6), which calibrates the grip. 475 SPS has less internal noise than 860 SPS.
#define NB_ADS_CONFIG(ch)  (0x81CBu | ((uint16_t)(4 + (ch)) << 12))
#define NB_ADS_GATE_US     2000    // as ADS_CONV_GATE_US of the HFB: no I2C poll before

NbGripCal nbGripCal;
bool  nbAt24Present  = false;
bool  nbGripCalValid = false;
bool  nbDrvPresent   = false;
int16_t nbGripRaw[4] = {0, 0, 0, 0};      // last sample per channel, counts at 5.000 V supply

static const NbGripCal defaultGripCal PROGMEM = {
  7350, 8755, 0, 32767, 0, 32767, 13000, 22500, 0, 0, 0, 0, NB_GRIP_MAGIC
};


// ---------------- DRV2605L ----------------
#define DRV_REG_MODE      0x01
#define DRV_REG_LIBRARY   0x03
#define DRV_REG_FEEDBACK  0x1A
#define DRV_REG_CONTROL3  0x1D
#define DRV_MODE_INTTRIG  0x00
#define DRV_MODE_RTP      0x05

// Writes 1 or 2 data bytes after a register / first byte (DRV2605L, ADS1115, PCF8575).
// b1 < 0: one data byte only. (No default argument: the Arduino IDE copies function
// signatures into its generated prototypes.)
static void i2cWrite(uint8_t addr, uint8_t reg, uint8_t b0, int16_t b1) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(b0);
  if (b1 >= 0) Wire.write((uint8_t)b1);
  Wire.endTransmission();
}

void nbDrvWrite(uint8_t reg, uint8_t val) {
  i2cWrite(NB_ADDR_DRV, reg, val, -1);
}

// ADS1115 register, 0 on an I2C error
static uint16_t adsRead(uint8_t reg) {
  Wire.beginTransmission(NB_ADDR_ADS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)NB_ADDR_ADS, (uint8_t)2) != 2) return 0;
  uint16_t v = (uint16_t)Wire.read() << 8;
  return v | Wire.read();
}

static void adsStart(uint8_t ch) {
  uint16_t c = NB_ADS_CONFIG(ch);
  i2cWrite(NB_ADDR_ADS, 0x01, (uint8_t)(c >> 8), (uint8_t)c);
}

// PCF8575 #i: 16 pins, P0-P7 in the low byte. On an I2C error the last value is kept
// (all released at power-up).
static uint16_t pcfRead(uint8_t i) {
  static uint16_t last[2] = {0xFFFF, 0xFFFF};
  if (Wire.requestFrom((uint8_t)(NB_ADDR_PCF1 + i), (uint8_t)2) == 2) {
    uint8_t lo = Wire.read();
    last[i] = lo | ((uint16_t)Wire.read() << 8);
  }
  return last[i];
}

static uint8_t drvRead(uint8_t reg) {
  Wire.beginTransmission(NB_ADDR_DRV);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)NB_ADDR_DRV, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

static bool drvBegin() {
  Wire.beginTransmission(NB_ADDR_DRV);
  if (Wire.endTransmission() != 0) return false;                 // not present
  nbDrvWrite(DRV_REG_MODE, 0x00);                                // leave standby
  nbDrvWrite(NB_DRV_REG_RTPIN, 0x00);                            // no vibration
  nbDrvWrite(DRV_REG_FEEDBACK, drvRead(DRV_REG_FEEDBACK) & 0x7F);  // ERM mode
  nbDrvWrite(DRV_REG_LIBRARY, 0x01);                             // ERM library A
  nbDrvWrite(DRV_REG_CONTROL3, drvRead(DRV_REG_CONTROL3) | 0x20);  // ERM open loop
  nbDrvWrite(DRV_REG_MODE, DRV_MODE_INTTRIG);
  return true;
}

void nbDrvSetRtp(bool enable) {
  if (!nbDrvPresent) return;
  if (enable) {
    nbDrvWrite(DRV_REG_MODE, DRV_MODE_RTP);
  } else {
    nbDrvWrite(NB_DRV_REG_RTPIN, 0);
    nbDrvWrite(DRV_REG_MODE, DRV_MODE_INTTRIG);
  }
}

// ---------------- AT24C32 ----------------
static bool at24Read(uint16_t addr, uint8_t* buf, uint16_t len) {
  uint16_t done = 0;
  while (done < len) {
    uint8_t chunk = (uint8_t)min((uint16_t)32, (uint16_t)(len - done));
    Wire.beginTransmission(NB_ADDR_AT24);
    Wire.write((uint8_t)(addr >> 8));
    Wire.write((uint8_t)(addr & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)NB_ADDR_AT24, chunk);
    uint8_t i = 0;
    while (Wire.available() && i < chunk) buf[done + i++] = Wire.read();
    if (i != chunk) return false;
    done += chunk; addr += chunk;
  }
  return true;
}

// ---------------- Grip ----------------
void nbGripBegin() {
  Wire.beginTransmission(NB_ADDR_AT24);
  nbAt24Present = (Wire.endTransmission() == 0);
  nbGripCalValid = nbAt24Present && at24Read(0, (uint8_t*)&nbGripCal, sizeof(NbGripCal))
                   && nbGripCal.magic == NB_GRIP_MAGIC;
  if (!nbGripCalValid) memcpy_P(&nbGripCal, &defaultGripCal, sizeof(NbGripCal));

  nbDrvPresent = drvBegin();

  // PCF8575: all pins high = inputs with the weak pull-up (buttons pull them low)
  i2cWrite(NB_ADDR_PCF1, 0xFF, 0xFF, -1);
  i2cWrite(NB_ADDR_PCF2, 0xFF, 0xFF, -1);
}

// ---------------- Supply compensation (ratiometric grip sensors) ----------------
// The SS49E Hall sensors of the grip scale with the USB supply (centre = Vcc / 2), the
// ADS1115 measures against its own reference. As in the HFB joystick firmware (which
// calibrates the grip and stores the ranges in its AT24C32), every sample is rescaled as
// if Vcc were 5.000 V, Vcc being measured with the ATmega32U4 1.1 V bandgap. Without this,
// a grip calibrated on another base reads off by the supply difference.
// Integer math: no float code in FFB_Setup. The ADC is never used for anything else.
//   counts * 5000 / Vcc, Vcc = 1100 mV * 1023 / bandgap  ->  counts * bandgap / 225.06
// The bandgap is converted in the background without pause (one conversion per call of
// nbGripPoll, i.e. every loop pass) and averaged with alpha 1/256, as the HFB does (EMA
// 0.005 per loop pass): the compensation itself must not add noise to the grip axes.
static uint32_t bgAcc;                 // bandgap ADC reading x 65536, EMA (alpha 1/256)
#define NB_BG_DIV  28808L              // 225.06 x 128 (bgAcc >> 9 = reading x 128)

void nbVccBegin() {
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);   // AVcc ref, bandgap
  ADCSRB &= ~_BV(MUX5);
  for (uint8_t i = 0; i < 16; i++) {           // ~1.7 ms: the bandgap settles, the last seeds
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) { }
  }
  bgAcc = (uint32_t)ADC << 16;
  ADCSRA |= _BV(ADSC);                         // first background conversion
}

// Non-blocking round robin over channels 1..3, call every loop pass. Returns the channel
// that got a new sample in nbGripRaw[] (counts at 5.000 V), or 0. The ADS1115 is only
// polled once its conversion can be finished, to keep the I2C bus free for the encoders.
uint8_t nbGripPoll() {
  static uint8_t ch = 1;
  static bool pending = false;
  static uint32_t t0 = 0;
  if (!(ADCSRA & _BV(ADSC))) {                 // bandgap conversion done: average it in
    bgAcc += (int32_t)(((uint32_t)ADC << 16) - bgAcc) >> 8;
    ADCSRA |= _BV(ADSC);
  }
  if (!pending) { adsStart(ch); t0 = micros(); pending = true; return 0; }
  if ((uint32_t)(micros() - t0) < NB_ADS_GATE_US) return 0;
  if (!(adsRead(0x01) & 0x8000)) return 0;        // OS bit: conversion still running
  // a pot never exceeds Vcc, so the compensated value stays below ~26700 (fits int16)
  nbGripRaw[ch] = (int16_t)((int32_t)(int16_t)adsRead(0x00) * (uint16_t)(bgAcc >> 9) / NB_BG_DIV);
  uint8_t done = ch;
  ch = (ch >= 3) ? 1 : (uint8_t)(ch + 1);
  adsStart(ch);
  t0 = micros();
  return done;
}

// Calibration range of channel 1 (Rx), 2 (Ry) or 3 (Rz). False when the axis is disabled
// in the grip calibration or the range is empty (the axis then reads centred).
//   NbGripCal as words: [0..7] = z, rx, ry, rz min / max, [8..11] = disabled z, rx, ry, rz
bool nbGripRange(uint8_t ch, uint16_t* lo, uint16_t* hi) {
  const uint16_t* w = (const uint16_t*)&nbGripCal;
  *lo = w[ch * 2];
  *hi = w[ch * 2 + 1];
  return !w[8 + ch] && *hi > *lo;
}

// 27 grip buttons, bit n = button n+1 pressed (PCF #2 uses pins 0-5 and 8-12).
uint32_t nbReadButtons() {
  static const uint8_t pcf2Pins[11] = {0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12};
  uint16_t p1 = pcfRead(0);
  uint16_t p2 = pcfRead(1);
  uint32_t b = (uint16_t)~p1;                     // buttons 1-16 (active low)
  for (uint8_t i = 0; i < 11; i++) {
    if (!((p2 >> pcf2Pins[i]) & 1)) b |= (uint32_t)1 << (16 + i);
  }
  return b;
}

// ================================================================
// SERIAL HELPERS
// ================================================================

// Reads one command line if one is waiting: upper case, trailing spaces / CR removed.
// Returns its length (0 = nothing). Needs Serial.setTimeout(20) (done by nbBegin()).
uint8_t nbReadCmd(char* buf, uint8_t size) {
  if (!Serial.available()) return 0;
  int len = Serial.readBytesUntil('\n', buf, size - 1);
  while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == ' ')) len--;
  buf[len] = '\0';
  for (int i = 0; i < len; i++) buf[i] = toupper((unsigned char)buf[i]);
  return (uint8_t)len;
}

// Splits "NAME ARG": buf keeps the name, the return value points to ARG (or NULL).
char* nbSplitArg(char* buf) {
  char* arg = strchr(buf, ' ');
  if (!arg) return NULL;
  *arg++ = '\0';
  while (*arg == ' ') arg++;
  return *arg ? arg : NULL;
}

// Blocks until the user sends a line (Enter). Returns its first character, upper case.
char nbWaitLine() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(5);
  delay(30);                                   // let the rest of the line arrive
  char c = toupper((unsigned char)Serial.read());
  while (Serial.available()) Serial.read();
  return c;
}

// True (and the input is flushed) when the user typed anything: used to abort tests.
bool nbKeyAbort() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

// "KEY:value" on its own line
void nbKV(const __FlashStringHelper* k, long v) {
  Serial.print(k); Serial.print(':'); Serial.println(v);
}

// Prints x10 / 10 with one decimal: 153 -> "15.3"
void nbPrintX10(int16_t x10) {
  if (x10 < 0) { Serial.print('-'); x10 = -x10; }
  Serial.print(x10 / 10); Serial.print('.'); Serial.print(x10 % 10);
}

// ######################################################################################
// END OF SHARED CODE
// ######################################################################################

// ######################################################################################
// SETUP ASSISTANT (FFB_Setup only)
// ######################################################################################

// ================================================================
// MOTOR TEST PRIMITIVES
// ================================================================
//
// All tests run at a fixed 1 kHz pace, abort on any key, and abort when the stick leaves
// the window 511 +/- NB_TEST_WINDOW (HID). There are no limit switches: keep a hand near
// the stick. Duty is given in the MOTOR frame (positive = IBT-2 R input), so these
// primitives also work before the motor inversion is known.

#define NB_TEST_WINDOW     435     // abort beyond 511 +/- this (85 % of the travel)
#define NB_RAMP_PCT_S      4       // breakaway ramp, duty % per second
#define NB_MOVE_THR        5       // raw counts that count as "moved" (3 ms in a row)

// Result codes (negative = no measurement)
#define NB_T_NOMOVE   0
#define NB_T_KEY     -1
#define NB_T_WINDOW  -2

// ---------------- Lines for the GUI ----------------
// Lines starting with '@' are for the GUI (the Serial Monitor shows them too):
//   @STEP n RUN|PASS|FAIL      a setup step starts / ends          @STEPS mask   steps done
//   @ASK ENTER|YES <code>      the sketch waits for Enter / YES    @MOTORS ON|OFF
//   @TEST <code> RUN|OK|FAIL   a test of step 3 or of the travel test
//   @TP <target> <stick>       travel test, moving axis, every 100 ms (HID)
//   @READY                     the sketch is back at the command prompt
static void at(const __FlashStringHelper* s) {
  Serial.print('@'); Serial.println(s);
}

static void ask(const __FlashStringHelper* code) {
  Serial.print(F("@ASK ")); Serial.println(code);
}

static void atSteps() {
  Serial.print(F("@STEPS ")); Serial.println(cfg.steps & NB_ST_ALL);
}

// "@TEST code RUN|OK|FAIL"; returns ok (state: 0 RUN, 1 OK, 2 FAIL)
static bool atTest(const __FlashStringHelper* code, uint8_t state) {
  Serial.print(F("@TEST ")); Serial.print(code);
  Serial.println(state == 0 ? F(" RUN") : state == 1 ? F(" OK") : F(" FAIL"));
  return state == 1;
}

// The only places the motors are switched on or off in this sketch.
static void motors(bool on) {
  nbMotorEnable(on);
  at(on ? F("MOTORS ON") : F("MOTORS OFF"));
}

// ---------------- Cancel ----------------
// "C" (or CANCEL) at any Enter prompt, or while a sweep / grip test waits for Enter, leaves
// the running command at once: loop() gets control back through longjmp, switches the
// motors off and reloads the stored configuration, so a cancelled step stores nothing and
// keeps its previous state (done / not done). YES prompts already cancel on any other answer.
static jmp_buf cancelJmp;

static void cancelOn(char c) {
  if (toupper((unsigned char)c) == 'C') longjmp(cancelJmp, 1);
}

// Enter prompt (after ask()): returns on Enter, cancels on C
static void waitEnter() {
  cancelOn(nbWaitLine());
}

static uint32_t paceNext = 0;

void nbPaceReset() { paceNext = micros(); }

// 1 ms steps
void nbPaceWait() {
  paceNext += 1000UL;
  if ((int32_t)(micros() - paceNext) > 1500) paceNext = micros();   // fell behind: resync
  while ((int32_t)(micros() - paceNext) < 0) { }
}

static int16_t readCont(uint8_t axis) {
  return nbUnwrap(nbReadAngle(axis ? NB_CH_Y : NB_CH_X), axis ? cfg.centerY : cfg.centerX);
}

// Last window check, for the abort message
static uint8_t abortAxis = 0;
static int16_t abortHid = 511;

static bool hidOut(uint8_t axis, int16_t h) {
  abortAxis = axis;
  abortHid = h;
  return h < 511 - NB_TEST_WINDOW || h > 511 + NB_TEST_WINDOW;
}

static bool outOfWindow(uint8_t axis, int16_t cont) {
  return hidOut(axis, nbHidFromCont(axis, cont));
}

static int16_t baseline(uint8_t axis) {
  int32_t s = 0;
  for (uint8_t i = 0; i < 8; i++) { s += readCont(axis); delay(1); }
  return (int16_t)(s / 8);
}

// Starts at 0 and raises the duty of one axis at NB_RAMP_PCT_S until the encoder moves,
// up to capCounts. Motor off again on return.
//   returns duty counts at breakaway (> 0), NB_T_NOMOVE, NB_T_KEY or NB_T_WINDOW
//   hidMove: HID change at the moment the motion is detected, while the motor still pushes
//   (sign = direction of the motion). Measured there, not after the stop: once the motor
//   stops, the grip weight or the cables may pull the stick back past its start point, and
//   a reading taken later could show the opposite direction.
int16_t nbRampUntilMove(uint8_t axis, int8_t dir, int16_t capCounts, int16_t* hidMove) {
  *hidMove = 0;
  nbSetMotor(axis, 0);
  delay(300);                                  // let the stick settle
  int16_t base = baseline(axis);
  int16_t h0 = nbHidFromCont(axis, base);
  int8_t hits = 0;                             // consecutive reads moved, signed (+ / -)
  int16_t result = NB_T_NOMOVE;
  uint32_t t = 0;
  nbPaceReset();
  while (true) {
    nbPaceWait();
    t++;
    int16_t counts = (int16_t)((t * (uint32_t)NB_RAMP_PCT_S * NB_PWM_TOP) / 100000UL);
    if (counts > capCounts) break;
    nbSetMotor(axis, dir > 0 ? counts : -counts);
    int16_t c = readCont(axis);
    if (nbKeyAbort()) { result = NB_T_KEY; break; }
    if (outOfWindow(axis, c)) { result = NB_T_WINDOW; break; }
    int16_t d = c - base;
    // 3 reads in a row moved the SAME way (1 ms apart)
    if (d >= NB_MOVE_THR)       hits = (hits > 0) ? hits + 1 : 1;
    else if (d <= -NB_MOVE_THR) hits = (hits < 0) ? hits - 1 : -1;
    else                        hits = 0;
    if (hits >= 3 || hits <= -3) {
      result = counts;
      *hidMove = nbHidFromCont(axis, c) - h0;
      break;
    }
  }
  nbSetMotor(axis, 0);
  return result;
}

// Constant duty for up to ms milliseconds, used where the motor must NOT move (E-Stop
// engaged). Stops at the first motion (NB_MOVE_THR raw counts, 3 ms in a row), so a motor
// that still has power only twitches.
//   returns the raw-count excursion when it moved (>= NB_MOVE_THR), 0 when it did not,
//   or NB_T_KEY / NB_T_WINDOW. Motor off again on return.
int16_t nbHoldDuty(uint8_t axis, int16_t counts, uint16_t ms) {
  int16_t base = baseline(axis);
  int16_t maxDev = 0;
  int16_t result = 0;
  uint8_t hits = 0;
  nbPaceReset();
  nbSetMotor(axis, counts);
  for (uint16_t i = 0; i < ms; i++) {
    nbPaceWait();
    int16_t c = readCont(axis);
    if (nbKeyAbort()) { result = NB_T_KEY; break; }
    if (outOfWindow(axis, c)) { result = NB_T_WINDOW; break; }
    int16_t d = abs(c - base);
    if (d > maxDev) maxDev = d;
    if (d >= NB_MOVE_THR) hits++; else hits = 0;
    if (hits >= 3) break;
  }
  nbSetMotor(axis, 0);
  if (result) return result;
  return (hits >= 3) ? maxDev : 0;
}

// True when both axes are within +/- maxOff HID of the centre.
bool nbNearCentre(int16_t maxOff) {
  int16_t x = 511, y = 511;
  // 8 reads per axis: the jump filter may hold the old value for up to 6 reads after the
  // stick was moved by hand
  for (uint8_t i = 0; i < 8; i++) { x = nbReadHid(0); y = nbReadHid(1); }
  return abs(x - 511) <= maxOff && abs(y - 511) <= maxOff;
}

// Asks the user to centre the stick and let go, until nbNearCentre(maxOff).
void nbAskCentre(int16_t maxOff) {
  while (!nbNearCentre(maxOff)) {
    Serial.println(F("Centre the stick, let go, press Enter."));
    ask(F("ENTER CENTRE"));
    waitEnter();
  }
}

// "ABORTED: ..." message for a negative result code
void nbPrintAbort(int16_t code) {
  if (code == NB_T_KEY) { Serial.println(F("ABORTED: key pressed")); return; }
  Serial.print(F("ABORTED: ")); Serial.print(abortAxis ? 'Y' : 'X');
  Serial.print(F(" moved too far (HID ")); Serial.print(abortHid);
  Serial.print(F(", safe window ")); Serial.print(511 - NB_TEST_WINDOW); Serial.print('-');
  Serial.print(511 + NB_TEST_WINDOW); Serial.println(F("). Motor stopped."));
}

// Limits for the reference hardware. PROVISIONAL: to be confirmed with more builds.
#define MIN_SPAN      200     // step 2: travel per axis, raw AS5600 counts (reference ~430-460)
#define MAX_SPAN      2000
#define MOTOR_CAP     200     // step 3: duty cap, counts (25 %)
#define MOTOR_PUSH    160     // step 3: E-Stop and noise pushes, counts (20 %)
#define MEAS_CAP      240     // step 4: duty cap, counts (30 %)
#define MEAS_REPS     3       // step 4: measurements per axis, direction and frequency
#define REF_B20_MIN   110     // step 4 expected ranges, 0.1 % duty and ns. Outside a
#define REF_B20_MAX   200     // range the result is still saved, with a warning.
#define REF_TD_MIN    4000
#define REF_TD_MAX    9000
#define REF_F_MAX     50
#define REF_ASYM_MAX  15

struct Dev { uint8_t addr; uint8_t level; const char* name; };   // level: 2 required, 1 recommended, 0 optional

struct AxisResult {
  int16_t b20[2], b5[2];      // median breakaway, 0.1 %, [0 = + direction, 1 = -]
  int16_t tdNs;               // IBT-2 dead time, ns
  int16_t fric[2];            // friction, 0.1 %
  int16_t dt;                 // suggested dead-time offset, 0.1 %
  bool    valid;
};

const char devN0[] PROGMEM = "TCA9548A mux (encoders)";
const char devN1[] PROGMEM = "PCF8575 #1 (buttons 1-16)";
const char devN2[] PROGMEM = "PCF8575 #2 (buttons 17-27)";
const char devN3[] PROGMEM = "ADS1115 (grip axes)";
const char devN4[] PROGMEM = "AT24C32 (grip calibration)";
const char devN5[] PROGMEM = "DRV2605L (grip vibration)";
const Dev devs[] PROGMEM = {
  { NB_ADDR_TCA,  2, devN0 }, { NB_ADDR_PCF1, 2, devN1 }, { NB_ADDR_PCF2, 2, devN2 },
  { NB_ADDR_ADS,  2, devN3 }, { NB_ADDR_AT24, 1, devN4 }, { NB_ADDR_DRV,  0, devN5 },
};

AxisResult res[2];

// ================================================================
// Console helpers
// ================================================================

// Waits for one line; true only when it is YES (any case).
static bool waitYes() {
  char buf[8];
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(5);
  delay(30);
  uint8_t n = nbReadCmd(buf, sizeof(buf));
  while (Serial.available()) Serial.read();
  return n == 3 && !strcmp_P(buf, PSTR("YES"));
}

static void stepTitle(uint8_t s, const __FlashStringHelper* note) {
  Serial.println();
  Serial.print(F("===== SETUP ")); nbPrintStep(s);
  Serial.print(F(" - ")); Serial.print(note); Serial.println(F(" ====="));
  Serial.println(F("(C at a prompt cancels)"));
  Serial.print(F("@STEP ")); Serial.print(s + 1); Serial.println(F(" RUN"));
}

// A step that fails is marked not done (FFB_Joystick refuses START until it passes again).
static bool stepResult(uint8_t s, bool ok) {
  Serial.println();
  nbStepSet(s, ok);
  Serial.print(ok ? F("PASS: ") : F("FAIL: ")); nbPrintStep(s);
  Serial.println(ok ? F(" done.") : F(" is not done. Fix the FAIL above, type SETUP."));
  Serial.print(F("@STEP ")); Serial.print(s + 1); Serial.println(ok ? F(" PASS") : F(" FAIL"));
  atSteps();
  return ok;
}

static void printProgress() {
  for (uint8_t s = 0; s < NB_STEPS; s++) {
    Serial.print(F("  ")); nbPrintStep(s);
    Serial.println(nbStepDone(s) ? F(": done") : F(": not done"));
  }
  atSteps();
}

static void printNext() {
  for (uint8_t s = 0; s < NB_STEPS; s++) {
    if (!nbStepDone(s)) {
      Serial.print(F("Next: type SETUP to run ")); nbPrintStep(s); Serial.println('.');
      return;
    }
  }
  Serial.println(F("All 4 setup steps are done: upload FFB_Joystick."));
}

// The motors are enabled only here, after a typed YES. Every step disables them at the end.
static bool enableMotors() {
  Serial.println(F("24 V ON, one hand near the stick, E-Stop within reach."));
  Serial.println(F("Type YES to enable the motors."));
  ask(F("YES MOTORS"));
  if (!waitYes()) { Serial.println(F("Cancelled, motors off.")); return false; }
  motors(true);
  Serial.println(F("Motors ENABLED. Any key stops a test."));
  return true;
}

// Button list " 1 5 17" for a bit mask (bit n = button n+1)
static void printButtons(uint32_t mask) {
  for (uint8_t i = 0; i < 27; i++) {
    if ((mask >> i) & 1) { Serial.print(' '); Serial.print(i + 1); }
  }
  Serial.println();
}

// Live X / Y HID values until a key is pressed (jump filter off: the stick moves by hand).
static void monitorXY() {
  nbEncFilter = false;
  while (!nbKeyAbort()) {
    Serial.print(F("  X ")); Serial.print(nbReadHid(0));
    Serial.print(F("   Y ")); Serial.println(nbReadHid(1));
    delay(150);
  }
  nbEncFilter = true;
  nbEncBegin();
}

// ================================================================
// Step 1 - Electronics (motors off)
// ================================================================
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool checkDevices() {
  Serial.println(F("\n[1] I2C devices"));
  Serial.print(F("    found:"));
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.print(F(" 0x")); Serial.print(a, HEX); }
  }
  Serial.println();
  bool ok = true;
  for (uint8_t i = 0; i < sizeof(devs) / sizeof(devs[0]); i++) {
    Dev d; memcpy_P(&d, &devs[i], sizeof(Dev));
    bool p = i2cPresent(d.addr);
    Serial.print(p ? F("    OK    ") : d.level == 2 ? F("    FAIL  ") : d.level == 1 ? F("    WARN  ") : F("    INFO  "));
    Serial.print(F("0x")); Serial.print(d.addr, HEX); Serial.print(' ');
    Serial.print((const __FlashStringHelper*)d.name);
    if (!p) Serial.print(d.level == 2 ? F(" - not found") : d.level == 1 ? F(" - not found, default grip axes") : F(" - not found (optional)"));
    Serial.println();
    if (!p && d.level == 2) ok = false;
  }
  return ok;
}

static bool checkMagnets() {
  Serial.println(F("\n[2] AS5600 encoders and magnets"));
  bool ok = true;
  for (uint8_t ch = 0; ch < 2; ch++) {
    Serial.print(F("    ")); Serial.print(ch ? 'Y' : 'X'); Serial.print(F(": "));
    uint8_t st, agc; uint16_t mag;
    if (!nbMagnetInfo(ch, &st, &agc, &mag)) {
      Serial.println(F("FAIL - no answer from the AS5600 (wiring, mux channel)"));
      ok = false; continue;
    }
    bool md = st & 0x20, ml = st & 0x10, mh = st & 0x08;
    bool good = md && !ml && !mh;
    Serial.print(good ? F("OK  ") : F("FAIL"));
    Serial.print(F(" magnet ")); Serial.print(md ? F("detected") : F("NOT detected"));
    if (ml) Serial.print(F(", too WEAK (closer)"));
    if (mh) Serial.print(F(", too STRONG (farther)"));
    Serial.print(F(", AGC ")); Serial.print(agc);
    Serial.print(F(" (best mid 0-255), magnitude ")); Serial.println(mag);
    if (!good) ok = false;
  }
  return ok;
}

static bool checkReads() {
  Serial.println(F("\n[3] Encoder reads (1000 per axis, hands off)"));
  bool ok = true;
  for (uint8_t ch = 0; ch < 2; ch++) {
    uint16_t e0 = nbI2cErr, g0 = nbGlitch[ch];
    uint16_t lo = 4095, hi = 0;
    for (uint16_t i = 0; i < 1000; i++) {
      uint16_t a = nbReadAngle(ch);
      if (a < lo) lo = a;
      if (a > hi) hi = a;
    }
    uint16_t err = nbI2cErr - e0, gl = nbGlitch[ch] - g0;
    bool good = (err == 0 && gl == 0);
    Serial.print(F("    ")); Serial.print(ch ? 'Y' : 'X');
    Serial.print(good ? F(": OK   ") : F(": FAIL "));
    Serial.print(F("I2C errors ")); Serial.print(err);
    Serial.print(F(", rejected jumps ")); Serial.print(gl);
    Serial.print(F(", angle ")); Serial.print(lo); Serial.print('-'); Serial.print(hi);
    Serial.println(hi - lo > 8 ? F(" (noisy: magnet, cable)") : F(""));
    if (!good) ok = false;
  }
  return ok;
}

// Grip axes for the grip test and MON: integer EMA of the compensated samples (x 256),
// 1/4 per sample (~150 Hz per axis). FFB_Joystick filters them like the HFB joystick.
static int32_t gripEma[4] = {0, 0, 0, 0};

static void gripPoll() {
  uint8_t ch = nbGripPoll();
  if (!ch) return;
  int32_t v = (int32_t)nbGripRaw[ch] << 8;
  gripEma[ch] = gripEma[ch] ? gripEma[ch] + ((v - gripEma[ch]) >> 2) : v;
}

// 0..1023 (Rz inverted as in v7.0), 511 when the axis is disabled
static int16_t gripAxis(uint8_t ch) {
  uint16_t lo, hi;
  if (!nbGripRange(ch, &lo, &hi)) return 511;
  int16_t r = (int16_t)(gripEma[ch] >> 8);
  uint16_t m = (r <= (int16_t)lo) ? 0 : (r >= (int16_t)hi) ? hi - lo : (uint16_t)(r - lo);
  int16_t v = (int16_t)((uint32_t)m * NB_AXIS_MAX / (hi - lo));
  return (ch == 3) ? (NB_AXIS_MAX - v) : v;
}

// Grip test, information only: vibration motor, buttons, travel of the grip axes. Grips are
// interchangeable and do not all have 27 buttons and 3 axes: the test only lists what was
// pressed and moved, nothing is expected. An axis disabled in the grip calibration (stored
// in the grip by the HFB joystick) is reported as not used.
static void gripTest() {
  Serial.println(F("\n[4] Grip"));
  Serial.print(F("    grip axis calibration: "));
  Serial.println(nbGripCalValid ? F("read from the grip EEPROM") : F("not found, default values in use"));
  if (nbDrvPresent) {
    Serial.println(F("    vibration motor: buzz"));
    nbDrvSetRtp(true);
    nbDrvWrite(NB_DRV_REG_RTPIN, 100);
    delay(500);
    nbDrvSetRtp(false);
  }
  uint32_t stuck = nbReadButtons();
  if (stuck) { Serial.print(F("    WARN pressed before the test:")); printButtons(stuck); }
  Serial.println(F("    Press the buttons of your grip, move its axes end to end,"));
  Serial.println(F("    then Enter (only what you use is listed)."));
  ask(F("ENTER GRIP"));
  while (Serial.available()) Serial.read();
  uint32_t seen = 0;
  int16_t lo[3] = {1023, 1023, 1023}, hi[3] = {0, 0, 0};
  uint32_t t0 = millis(), lastBtn = 0;
  while (!Serial.available()) {
    gripPoll();
    uint32_t now = millis();
    if (now - t0 > 300) {                       // let the axis filter settle first
      for (uint8_t i = 0; i < 3; i++) {
        int16_t v = gripAxis(i + 1);
        if (v < lo[i]) lo[i] = v;
        if (v > hi[i]) hi[i] = v;
      }
    }
    if (now - lastBtn >= 5) {
      lastBtn = now;
      uint32_t fresh = nbReadButtons() & ~stuck & ~seen;
      if (fresh) { seen |= fresh; Serial.print(F("    button")); printButtons(fresh); }
    }
  }
  cancelOn(Serial.peek());
  delay(30);
  while (Serial.available()) Serial.read();
  uint8_t n = 0;
  for (uint8_t i = 0; i < 27; i++) if ((seen >> i) & 1) n++;
  Serial.print(F("    buttons pressed: ")); Serial.println(n);
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t a, b;
    Serial.print(F("    R")); Serial.print((char)('x' + i));
    if (!nbGripRange(i + 1, &a, &b)) { Serial.println(F(" not used (disabled in grip cal)")); continue; }
    Serial.print(F(" travel ")); Serial.print(lo[i]); Serial.print('-'); Serial.print(hi[i]);
    Serial.println(hi[i] - lo[i] < 50 ? F("  (not moved)") : F(""));
  }
}

static bool stepElectronics() {
  stepTitle(NB_ST_ELECTRONICS, F("motors off"));
  Serial.println(F("Hands off the stick, then press Enter."));
  ask(F("ENTER HANDS_OFF"));
  waitEnter();
  bool ok = checkDevices();
  ok = checkMagnets() && ok;
  ok = checkReads() && ok;
  if (ok) gripTest();
  else Serial.println(F("\n(grip test: once the checks pass, or GRIP)"));
  return stepResult(NB_ST_ELECTRONICS, ok);
}

// ================================================================
// Step 2 - Calibration (motors off)
// ================================================================
//   Axis convention (what Windows and DCS expect):
//     X: stick to the RIGHT -> HID value goes UP (0..1023)
//     Y: stick FORWARD      -> HID value goes DOWN

// Average of 16 reads, HID frame.
static int16_t hidAvg(uint8_t axis) {
  int32_t s = 0;
  for (uint8_t i = 0; i < 16; i++) { s += nbReadHid(axis); delay(2); }
  return (int16_t)(s / 16);
}

// Asks the user to hold the stick at an end; returns the HID reading (no inversion), or
// -1 when the stick is still near the centre after 3 tries.
static int16_t holdAt(const __FlashStringHelper* where, const __FlashStringHelper* code, uint8_t axis) {
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("Hold the stick fully ")); Serial.print(where); Serial.println(F(" and press Enter."));
    ask(code);
    waitEnter();
    int16_t h = hidAvg(axis);
    if (h < 311 || h > 711) return h;
    Serial.println(F("  Not far enough."));
  }
  Serial.println(F("FAIL the reading does not follow the stick: encoder, magnet?"));
  return -1;
}

static bool calDirChanged = false;   // set by calibrate(): DIR_X / DIR_Y changed, step 3 cleared

static bool calibrate() {
  Serial.println(F("Move the stick slowly along ALL its limits (3-4 full circles),"));
  Serial.println(F("then press Enter."));
  ask(F("ENTER SWEEP"));
  while (Serial.available()) Serial.read();

  uint16_t anchorX = nbReadAngle(NB_CH_X);
  uint16_t anchorY = nbReadAngle(NB_CH_Y);
  int16_t minX = 2048, maxX = 2048, minY = 2048, maxY = 2048;
  uint32_t lastPrint = millis();
  while (!Serial.available()) {
    int16_t x = nbUnwrap(nbReadAngle(NB_CH_X), anchorX);
    int16_t y = nbUnwrap(nbReadAngle(NB_CH_Y), anchorY);
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (millis() - lastPrint > 250) {
      lastPrint = millis();
      Serial.print(F("  X travel ")); Serial.print(maxX - minX);
      Serial.print(F("   Y travel ")); Serial.println(maxY - minY);
    }
    delay(10);
  }
  cancelOn(Serial.peek());
  delay(30);
  while (Serial.available()) Serial.read();

  int16_t spanX = maxX - minX, spanY = maxY - minY;
  bool ok = true;
  if (spanX < MIN_SPAN || spanX > MAX_SPAN) { Serial.print(F("FAIL X travel ")); Serial.println(spanX); ok = false; }
  if (spanY < MIN_SPAN || spanY > MAX_SPAN) { Serial.print(F("FAIL Y travel ")); Serial.println(spanY); ok = false; }
  if (!ok) {
    Serial.print(F("Expected ")); Serial.print(MIN_SPAN); Serial.print('-'); Serial.print(MAX_SPAN);
    Serial.println(F(" counts. All limits reached? Magnet centred?"));
    return false;
  }

  // Centre = middle of the travel (always positive here, so & 4095 == % 4096)
  int16_t cX = (minX + maxX) / 2, cY = (minY + maxY) / 2;
  cfg.centerX = (uint16_t)((cX - 2048 + anchorX + 4096) & 4095);
  cfg.centerY = (uint16_t)((cY - 2048 + anchorY + 4096) & 4095);
  int16_t rX = spanX / 2, rY = spanY / 2;
  cfg.minX = (uint16_t)(2048 - rX + 5);  cfg.maxX = (uint16_t)(2048 + rX - 5);
  cfg.minY = (uint16_t)(2048 - rY + 5);  cfg.maxY = (uint16_t)(2048 + rY - 5);
  Serial.print(F("OK   travel X ")); Serial.print(spanX); Serial.print(F(", Y ")); Serial.print(spanY);
  Serial.println(F(" counts"));

  // Encoder directions (DIR_X / DIR_Y), measured with both set to 0. The axis inversion
  // for the sim (INV_AX / INV_AY of FFB_Joystick) is stored apart and is not touched here.
  uint8_t oldAX = cfg.invAX, oldAY = cfg.invAY;
  cfg.invAX = 0; cfg.invAY = 0;
  int16_t hx = holdAt(F("to the RIGHT"), F("ENTER RIGHT"), 0);
  int16_t hy = (hx < 0) ? -1 : holdAt(F("FORWARD (away from you)"), F("ENTER FORWARD"), 1);
  if (hy < 0) {                        // keep the old directions, calibration not done
    cfg.invAX = oldAX; cfg.invAY = oldAY;
    return false;
  }
  cfg.invAX = (hx < 511) ? 1 : 0;
  cfg.invAY = (hy > 511) ? 1 : 0;
  Serial.print(F("OK   DIR_X:")); Serial.print(cfg.invAX); Serial.print(F("  DIR_Y:")); Serial.println(cfg.invAY);

  // The motor directions (step 3) were measured in the OLD calibration frame. A changed
  // encoder direction comes from a moved magnet or encoder, or from INV_AX / INV_AY of
  // firmware 1.0.4 (it changed this same field): the firmware cannot tell which, and a
  // wrong motor direction pushes the stick away from the centre. Step 3 is cleared and
  // stepCalibration() offers the short direction re-check.
  if (nbStepDone(NB_ST_MOTORS) && (cfg.invAX != oldAX || cfg.invAY != oldAY)) {
    cfg.steps &= (uint8_t)~(1 << NB_ST_MOTORS);
    calDirChanged = true;
    Serial.println(F("NOTE an axis direction changed: motor directions to check again."));
  } else {
    Serial.println(F("     Directions unchanged: motor directions valid."));
  }

  Serial.println(F("\nCheck: X right = up to 1023, Y forward = down to 0."));
  Serial.println(F("Press Enter when done."));
  ask(F("ENTER CHECK"));
  delay(500);
  monitorXY();
  return true;
}

static void recheckDirections();      // step 3, below

static bool stepCalibration() {
  if (!nbRequireSteps(1 << NB_ST_ELECTRONICS)) return false;
  stepTitle(NB_ST_CALIBRATION, F("motors off"));
  nbEncFilter = false;                 // the stick is moved by hand between reads
  calDirChanged = false;
  bool ok = calibrate();
  nbEncFilter = true;
  nbEncBegin();
  stepResult(NB_ST_CALIBRATION, ok);
  if (ok && calDirChanged) recheckDirections();
  return ok;
}

// ================================================================
// Step 3 - Motors (max 25 %)
// ================================================================
static bool testEstop() {
  Serial.println(F("\n[1] E-STOP TEST (checked with a push)."));
  Serial.println(F("    PRESS the E-Stop now, then press Enter."));
  Serial.println(F("    The X motor gets a 20 % push: it must NOT move."));
  atTest(F("ESTOP"), 0);
  ask(F("ENTER ESTOP_PRESS"));
  waitEnter();
  nbAskCentre(300);
  int16_t m = nbHoldDuty(0, MOTOR_PUSH, 400);     // stops at the first twitch
  if (m < 0) { nbPrintAbort(m); return false; }
  if (m >= NB_MOVE_THR) {
    Serial.println(F("    FAIL the X motor moved with the E-Stop pressed."));
    Serial.println(F("         The E-Stop must cut the 24 V of BOTH IBT-2."));
    return false;
  }
  Serial.println(F("    OK   no motion with the E-Stop"));
  Serial.println(F("    RELEASE the E-Stop, wait 2 s, Enter: now X MUST move."));
  ask(F("ENTER ESTOP_RELEASE"));
  waitEnter();
  nbAskCentre(300);
  int16_t mv;
  int16_t r = nbRampUntilMove(0, +1, MOTOR_CAP, &mv);
  if (r < 0) { nbPrintAbort(r); return false; }
  Serial.println(r > 0 ? F("    OK   moves after the release")
                       : F("    FAIL no motion after the release: 24 V, relay, IBT-2 wiring, D7 to EN"));
  return atTest(F("ESTOP"), r > 0 ? 1 : 2);
}

// Positive force (library frame) must move the stick towards increasing HID. The test
// drives the IBT-2 R input (positive duty) and watches which way the HID value goes.
static bool testDirAxis(uint8_t axis, uint8_t* inv) {
  char a = axis ? 'Y' : 'X';
  nbAskCentre(300);
  int16_t mvP, mvN;
  int16_t rP = nbRampUntilMove(axis, +1, MOTOR_CAP, &mvP);
  if (rP < 0) { nbPrintAbort(rP); return false; }
  int16_t rN = nbRampUntilMove(axis, -1, MOTOR_CAP, &mvN);
  if (rN < 0) { nbPrintAbort(rN); return false; }
  Serial.print(F("    ")); Serial.print(a); Serial.print(F(": "));
  if (rP == 0 || rN == 0) {
    Serial.println(F("FAIL no motion up to 25 %: 24 V, E-Stop, motor and IBT-2 wiring"));
    return false;
  }
  if ((mvP > 0) == (mvN > 0) || mvP == 0 || mvN == 0) {
    Serial.println(F("FAIL inconsistent direction (belt slip? encoders swapped?)"));
    return false;
  }
  *inv = (mvP < 0) ? 1 : 0;
  Serial.print(F("OK   MDIR_")); Serial.print(a); Serial.print(':'); Serial.print(*inv);
  Serial.print(F("  (breakaway ")); nbPrintX10((int16_t)((int32_t)rP * 1000 / NB_PWM_TOP));
  Serial.print(F(" % / ")); nbPrintX10((int16_t)((int32_t)rN * 1000 / NB_PWM_TOP)); Serial.println(F(" %)"));
  return true;
}

static bool testDir() {
  Serial.println(F("\n[2] DIRECTIONS: hands off, each motor pushes both ways."));
  atTest(F("DIR"), 0);
  uint8_t ix = cfg.invMX, iy = cfg.invMY;
  if (!testDirAxis(0, &ix) || !testDirAxis(1, &iy)) return atTest(F("DIR"), 2);
  cfg.invMX = ix;
  cfg.invMY = iy;
  return atTest(F("DIR"), 1);
}

static bool testEmi() {
  Serial.println(F("\n[3] NOISE: hold the stick FIRMLY at the centre, press Enter."));
  Serial.println(F("    Both motors push back and forth for 5 s."));
  atTest(F("NOISE"), 0);
  ask(F("ENTER NOISE"));
  waitEnter();
  uint16_t e0 = nbI2cErr, g0 = nbGlitch[0] + nbGlitch[1];
  int16_t result = 0;
  nbPaceReset();
  for (uint16_t t = 0; t < 5000 && !result; t++) {
    nbPaceWait();
    int16_t s = ((t / 250) & 1) ? -MOTOR_PUSH : MOTOR_PUSH;
    nbSetMotor(0, s);
    nbSetMotor(1, s);
    int16_t hx = nbReadHid(0), hy = nbReadHid(1);
    if (nbKeyAbort()) result = NB_T_KEY;
    else if (hidOut(0, hx) || hidOut(1, hy)) result = NB_T_WINDOW;
  }
  nbStopMotors();
  if (result) { nbPrintAbort(result); return atTest(F("NOISE"), 2); }
  uint16_t err = nbI2cErr - e0, gl = nbGlitch[0] + nbGlitch[1] - g0;
  bool ok = err == 0 && gl == 0;
  Serial.print(ok ? F("    OK   ") : F("    FAIL "));
  Serial.print(F("I2C errors ")); Serial.print(err);
  Serial.print(F(", rejected jumps ")); Serial.println(gl);
  if (!ok) Serial.println(F("    Keep the I2C cables away from motor wires; 2.2-4.7k pull-ups."));
  return atTest(F("NOISE"), ok ? 1 : 2);
}

static bool stepMotors() {
  if (!nbRequireSteps(1 << NB_ST_CALIBRATION)) return false;
  stepTitle(NB_ST_MOTORS, F("max 25 % duty"));
  Serial.println(F("Tests: E-Stop, motor directions, encoder noise."));
  if (!enableMotors()) return false;
  bool ok = testEstop();
  ok = ok && testDir();
  ok = ok && testEmi();
  motors(false);
  Serial.println(F("Motors disabled."));
  return stepResult(NB_ST_MOTORS, ok);
}

// After a calibration that changed an encoder direction: only the motor direction test of
// step 3 (no E-Stop or noise test, the wiring did not change). Declined or failed: step 3
// stays not done and SETUP runs all of it.
static void recheckDirections() {
  Serial.println(F("\n===== STEP 3 - MOTOR DIRECTION RE-CHECK ====="));
  at(F("RECHECK"));
  if (!enableMotors()) return;
  bool ok = testDir();
  motors(false);
  Serial.println(F("Motors disabled."));
  stepResult(NB_ST_MOTORS, ok);
}

// ================================================================
// Step 4 - Measurement (max 30 %)
// ================================================================
// Breakaway duty per axis and direction, at 20 kHz and at 5 kHz PWM. The IBT-2 dead
// time grows with the PWM frequency and the friction does not, so the two separate:
//     b(f) = F + 100 * f * td            (b, F in % duty, td in s)
//     td   = (b20 - b5) / (100 * 15 kHz)
//     DT   = 100 * 20 kHz * td + F / 2   -> DTX / DTY
// (reference base: b20 ~15.5 %, b5 ~5.3 % -> td ~6.8 us, F ~1.9 %, DT ~14.5 %)

// One breakaway measurement in the FORCE frame (+ = towards increasing HID).
// Returns 0.1 % duty, or <= 0 (no motion / aborted, already reported).
static int16_t breakaway(uint8_t axis, int8_t dir) {
  if (!nbNearCentre(300)) nbAskCentre(300);
  uint8_t inv = axis ? cfg.invMY : cfg.invMX;
  int16_t mv;
  int16_t r = nbRampUntilMove(axis, inv ? -dir : dir, MEAS_CAP, &mv);
  if (r < 0) { nbPrintAbort(r); return r; }
  if (r == 0) { Serial.println(F("    no motion up to 30 %: 24 V, E-Stop?")); return 0; }
  if ((mv > 0) != (dir > 0)) {
    Serial.println(F("    FAIL wrong direction: type MOTORS"));
    return 0;
  }
  return (int16_t)((int32_t)r * 1000 / NB_PWM_TOP);
}

static int16_t median3(int16_t a, int16_t b, int16_t c) {
  if (a > b) { int16_t t = a; a = b; b = t; }
  if (b > c) { int16_t t = b; b = c; c = t; }
  if (a > b) { int16_t t = a; a = b; b = t; }
  return b;
}

static void warnIf(bool bad, char axis, const __FlashStringHelper* what) {
  if (!bad) return;
  Serial.print(F("    WARN ")); Serial.print(axis); Serial.print(' '); Serial.println(what);
}

static void compute(uint8_t axis) {
  AxisResult& r = res[axis];
  char a = axis ? 'Y' : 'X';
  int32_t td[2];
  for (uint8_t d = 0; d < 2; d++) td[d] = (int32_t)(r.b20[d] - r.b5[d]) * 200 / 3;   // 0.1 % -> ns
  r.tdNs = (int16_t)((td[0] + td[1]) / 2);
  for (uint8_t d = 0; d < 2; d++) r.fric[d] = r.b5[d] - r.tdNs / 200;               // 5 kHz share
  int16_t dt = r.tdNs / 50 + (r.fric[0] + r.fric[1]) / 4;                             // 20 kHz share + F/2
  r.dt = constrain(dt, 0, 250);
  r.valid = r.tdNs > 0;

  Serial.print(F("\n    ")); Serial.print(a);
  Serial.print(F(": breakaway 20 kHz ")); nbPrintX10(r.b20[0]); Serial.print('/'); nbPrintX10(r.b20[1]);
  Serial.print(F(" %, 5 kHz ")); nbPrintX10(r.b5[0]); Serial.print('/'); nbPrintX10(r.b5[1]); Serial.println(F(" % (+/-)"));
  Serial.print(F("       dead time ")); Serial.print(r.tdNs); Serial.print(F(" ns, friction "));
  nbPrintX10(r.fric[0]); Serial.print('/'); nbPrintX10(r.fric[1]);
  Serial.print(F(" %  ->  DT")); Serial.print(a); Serial.print(F(" = ")); Serial.print(r.dt);
  Serial.print(F(" (")); nbPrintX10(r.dt); Serial.println(F(" %)"));

  warnIf(!r.valid, a, F("dead time not measurable"));
  warnIf(r.b20[0] < REF_B20_MIN || r.b20[1] < REF_B20_MIN, a, F("breakaway at 20 kHz LOW"));
  warnIf(r.b20[0] > REF_B20_MAX || r.b20[1] > REF_B20_MAX, a, F("breakaway at 20 kHz HIGH: belt, bearings?"));
  warnIf(r.valid && (r.tdNs < REF_TD_MIN || r.tdNs > REF_TD_MAX), a, F("dead time outside 4-9 us: not an IBT-2?"));
  warnIf(r.fric[0] > REF_F_MAX || r.fric[1] > REF_F_MAX, a, F("friction above 5 %: belt, bearings, alignment"));
  warnIf(abs(r.fric[0] - r.fric[1]) > REF_ASYM_MAX, a, F("friction differs by direction: grip weight, belt"));
}

// All measurements; the motors are disabled at the end. True when both axes are valid.
static bool measureAll() {
  int16_t m[2][2][2][MEAS_REPS];       // [axis][freq 0 = 20 kHz, 1 = 5 kHz][dir][rep]
  static const uint8_t freqs[2] = {20, 5};
  for (uint8_t f = 0; f < 2; f++) {
    nbSetPwmFreq(freqs[f]);
    for (uint8_t axis = 0; axis < 2; axis++) {
      for (uint8_t rep = 0; rep < MEAS_REPS; rep++) {
        for (uint8_t d = 0; d < 2; d++) {
          int16_t v = breakaway(axis, d ? -1 : +1);
          if (v <= 0) {
            nbSetPwmFreq(20);
            motors(false);
            Serial.println(F("Measurement incomplete, motors off."));
            return false;
          }
          m[axis][f][d][rep] = v;
          Serial.print(F("    ")); Serial.print(axis ? 'Y' : 'X'); Serial.print(d ? '-' : '+');
          Serial.print(' '); Serial.print(freqs[f]); Serial.print(F(" kHz #")); Serial.print(rep + 1);
          Serial.print(F(": ")); nbPrintX10(v); Serial.println(F(" %"));
        }
      }
    }
  }
  nbSetPwmFreq(20);
  motors(false);
  Serial.println(F("Measurements done, motors off."));
  bool ok = true;
  for (uint8_t axis = 0; axis < 2; axis++) {
    for (uint8_t d = 0; d < 2; d++) {
      res[axis].b20[d] = median3(m[axis][0][d][0], m[axis][0][d][1], m[axis][0][d][2]);
      res[axis].b5[d]  = median3(m[axis][1][d][0], m[axis][1][d][1], m[axis][1][d][2]);
    }
    compute(axis);
    ok = ok && res[axis].valid;
  }
  return ok;
}

static bool stepMeasure() {
  if (!nbRequireSteps(1 << NB_ST_MOTORS)) return false;
  stepTitle(NB_ST_MEASURE, F("max 30 % duty, about 2 minutes"));
  Serial.println(F("24 slow pushes, HANDS OFF. 5 kHz pushes whine (normal)."));
  if (!enableMotors()) return false;
  bool ok = measureAll();
  if (ok) {
    Serial.print(F("\nDTX / DTY stored now: ")); Serial.print(cfg.dtX); Serial.print('/'); Serial.print(cfg.dtY);
    Serial.print(F(", measured: ")); Serial.print(res[0].dt); Serial.print('/'); Serial.println(res[1].dt);
    Serial.println(F("Type YES to save them."));
    ask(F("YES SAVE"));
    if (!waitYes()) {
      Serial.println(F("Discarded."));
      return false;
    }
    for (uint8_t a = 0; a < 2; a++) {
      cfg.tdNs[a] = (uint16_t)res[a].tdNs;
      for (uint8_t d = 0; d < 2; d++) cfg.fric10[a][d] = (uint8_t)constrain(res[a].fric[d], 0, 255);
    }
    cfg.dtX = (uint16_t)res[0].dt;
    cfg.dtY = (uint16_t)res[1].dt;
    nbDriveDerive();
  }
  return stepResult(NB_ST_MEASURE, ok);
}

// ================================================================
// Travel test (after step 4, information only)
// ================================================================
// Visual check like the one commercial FFB bases do after calibration: the motors move
// the stick slowly from end to end, Y first, then X, the other axis held at the centre.
// PI position control per axis with speed damping, 1 kHz, in duty counts:
//     duty = dead time + 3/4 friction (both measured in step 4, direction of travel)
//            + error / TRAVEL_KP_DIV + sum(error) / TRAVEL_KI_DIV
//            + (target speed - stick speed) * TRAVEL_KV / 16
// The stick behaves like a speed source (DC motor) with stick-slip friction: the P gain
// keeps the slips short, the speed term brakes each slip before it becomes a jump, the
// integral takes up what step 4 does not see (grip weight away from the centre, friction
// that changes along the travel). Aborts on any key or when the stick stays too far from
// its target (hand, belt, E-Stop).
#define TRAVEL_END      60      // HID from each end (travel 60 .. 963)
#define TRAVEL_SPEED    5       // 1/32 HID per ms (~156 HID per second)
#define TRAVEL_DWELL    500     // ms stopped at each end and at the centre
#define TRAVEL_KP_DIV   4       // P: 1 duty count (0.125 %) per 4 HID of error
#define TRAVEL_KV       4       // speed damping: counts per 16 HID / 40 ms of speed error
#define TRAVEL_KI_DIV   1000    // I: sum of error/4 per ms -> 1 count per 4000 HID*ms
#define TRAVEL_I_LIM    32000   // integral limit: +-32 counts (4 %)
#define TRAVEL_HOLD_DB  10      // HID: no drive inside this error while the target stands still
#define TRAVEL_EFF_MAX  48      // counts above 3/4 friction (6 %): speed limit
#define TRAVEL_ERR_MAX  150     // HID from the target ...
#define TRAVEL_ERR_MS   300     // ... for this long aborts
#define NB_T_FOLLOW     -3      // abort code: the stick did not follow

int16_t travTgt[2];              // target per axis, 1/32 HID
int16_t travI[2];                // integral per axis, HID / 4 * ms
int16_t travTd[2];               // dead time per axis, duty counts at 20 kHz
int16_t travLo[2], travHi[2], travErrMax[2];
int16_t travHist[2][8];          // stick position every 5 ms (40 ms ring) for the speed
int16_t travVel[2];              // speed term per axis, counts
uint8_t travHead = 0;
int16_t travAbort[4];            // axis, target, stick, duty at the abort

// Signed motor duty (motor frame) for axis a with tracking error err (HID). vdir: direction
// the target moves (0 = standing still: no drive and no integration inside the dead band).
static int16_t travelDuty(uint8_t a, int16_t err, int8_t vdir) {
  int8_t dir = vdir;
  if (!dir) dir = (err > TRAVEL_HOLD_DB) ? 1 : (err < -TRAVEL_HOLD_DB) ? -1 : 0;
  if (!dir) return 0;
  travI[a] = constrain(travI[a] + err / 4, -TRAVEL_I_LIM, TRAVEL_I_LIM);
  int16_t fric = (int16_t)cfg.fric10[a][dir > 0 ? 0 : 1] * 3 / 5;      // 3/4 of 0.1 % -> counts
  int16_t lim = fric + TRAVEL_EFF_MAX;
  int16_t eff = dir * fric + err / TRAVEL_KP_DIV + travI[a] / TRAVEL_KI_DIV + travVel[a];
  eff = constrain(eff, -lim, lim);
  if (!eff) return 0;
  int16_t d = travTd[a] + abs(eff);
  if (d > MOTOR_CAP) d = MOTOR_CAP;
  if (eff < 0) d = -d;
  return (a ? cfg.invMY : cfg.invMX) ? -d : d;                         // force -> motor frame
}

// Moves the target of axis a to HID 'to' at TRAVEL_SPEED, then waits dwellMs; the other
// axis holds its target. Returns 0, NB_T_KEY or NB_T_FOLLOW. Motors off on abort.
static int16_t travelMove(uint8_t a, int16_t to, uint16_t dwellMs) {
  int16_t goal = to * 32;
  uint16_t still = 0, tick = 0;
  uint16_t far[2] = {0, 0};
  nbPaceReset();
  while (still < dwellMs) {
    nbPaceWait();
    tick++;
    int8_t vdir = 0;
    if (travTgt[a] < goal) {
      travTgt[a] += TRAVEL_SPEED; vdir = 1;
      if (travTgt[a] > goal) travTgt[a] = goal;
    } else if (travTgt[a] > goal) {
      travTgt[a] -= TRAVEL_SPEED; vdir = -1;
      if (travTgt[a] < goal) travTgt[a] = goal;
    } else {
      if (!still) travI[a] = 0;        // target arrived: drop what the ramp built (no overshoot)
      still++;
    }
    bool velTick = (tick % 5 == 0);
    for (uint8_t i = 0; i < 2; i++) {
      int16_t p = nbReadHid(i);
      if (velTick) {                   // speed over the last 40 ms vs the target speed
        int16_t want = (i == a) ? vdir * (TRAVEL_SPEED * 40 / 32) : 0;
        travVel[i] = (want - (p - travHist[i][travHead])) * TRAVEL_KV / 16;
        travHist[i][travHead] = p;
      }
      if (p < travLo[i]) travLo[i] = p;
      if (p > travHi[i]) travHi[i] = p;
      int16_t t = travTgt[i] / 32;
      int16_t err = t - p;
      int16_t e = abs(err);
      if (e > travErrMax[i]) travErrMax[i] = e;
      int16_t d = travelDuty(i, err, (i == a) ? vdir : 0);
      far[i] = (e > TRAVEL_ERR_MAX) ? far[i] + 1 : 0;
      if (far[i] >= TRAVEL_ERR_MS) {
        nbStopMotors();
        travAbort[0] = i; travAbort[1] = t; travAbort[2] = p; travAbort[3] = d;
        return NB_T_FOLLOW;
      }
      nbSetMotor(i, d);
      if (i == a && tick % 100 == 0) {            // target, stick, duty (counts) every 100 ms
        Serial.print(F("@TP ")); Serial.print(t); Serial.print(' '); Serial.print(p);
        Serial.print(' '); Serial.println(d);
      }
    }
    if (velTick) travHead = (travHead + 1) & 7;
    if (nbKeyAbort()) { nbStopMotors(); return NB_T_KEY; }
  }
  return 0;
}

static void printRange(char axis, uint8_t i) {
  Serial.print(F("    ")); Serial.print(axis);
  Serial.print(F(": reached ")); Serial.print(travLo[i]); Serial.print('-'); Serial.print(travHi[i]);
  Serial.print(F(" (target ")); Serial.print(TRAVEL_END); Serial.print('-'); Serial.print(NB_AXIS_MAX - TRAVEL_END);
  Serial.print(F("), largest lag ")); Serial.print(travErrMax[i]); Serial.println(F(" HID"));
}

// Direction check before the travel: a slow push per axis in the FORCE frame (+ = towards
// increasing HID) must move the stick that way. Stops at the first motion, so a motor
// turning the wrong way only moves the stick a few HID. Never changes the stored motor
// direction: that is step 3's job.
static bool travelDirCheck() {
  for (uint8_t axis = 0; axis < 2; axis++) {
    const __FlashStringHelper* code = axis ? F("DIRY") : F("DIRX");
    atTest(code, 0);
    nbAskCentre(300);
    int16_t mv;
    int16_t r = nbRampUntilMove(axis, (axis ? cfg.invMY : cfg.invMX) ? -1 : 1, MOTOR_CAP, &mv);
    if (r < 0) { nbPrintAbort(r); return atTest(code, 2); }
    Serial.print(F("  ")); Serial.print(axis ? 'Y' : 'X');
    if (r == 0) {
      Serial.println(F(" motor: FAIL no motion: 24 V, E-Stop?"));
      return atTest(code, 2);
    }
    if (mv <= 0) {
      Serial.println(F(" motor: FAIL wrong direction, type MOTORS."));
      return atTest(code, 2);
    }
    Serial.println(F(" motor: direction OK"));
    atTest(code, 1);
  }
  return true;
}

// Y: forward, back, centre; then X: left, right, centre (HID: Y forward = down, X right = up)
static void travelTest() {
  if (!nbRequireSteps((1 << NB_ST_MOTORS) | (1 << NB_ST_MEASURE))) return;
  Serial.println(F("\n===== TRAVEL TEST (visual check) ====="));
  Serial.println(F("Direction check, then end to end, Y then X (30 s)."));
  Serial.println(F("Hands off, E-Stop within reach. Any key stops it."));
  Serial.println(F("Type YES to start (anything else skips it)."));
  ask(F("YES TRAVEL"));
  if (!waitYes()) { Serial.println(F("Travel test skipped.")); return; }
  at(F("TRAVEL RUN"));
  motors(true);
  if (!travelDirCheck()) {
    motors(false);
    Serial.println(F("Travel test stopped, motors off."));
    at(F("TRAVEL FAIL"));
    return;
  }
  nbEncBegin();                        // fresh reads: the stick may have been moved by hand
  for (uint8_t i = 0; i < 2; i++) {
    int16_t p = nbReadHid(i);
    travTgt[i] = p * 32;
    travI[i] = 0;
    travVel[i] = 0;
    for (uint8_t k = 0; k < 8; k++) travHist[i][k] = p;
    travLo[i] = travHi[i] = p;
    travErrMax[i] = 0;
    travTd[i] = (int16_t)(cfg.tdNs[i] / 63);       // ns -> counts: 799 / 50000 ns = 1 / 62.6
  }
  static const uint8_t  segAxis[8] PROGMEM = { 1, 0, 1, 1, 1, 0, 0, 0 };
  static const uint16_t segTo[8]   PROGMEM = { 511, 511, TRAVEL_END, NB_AXIS_MAX - TRAVEL_END, 511,
                                               TRAVEL_END, NB_AXIS_MAX - TRAVEL_END, 511 };
  static const char segNames[] PROGMEM =
    "centre\0centre\0Y forward\0Y back\0Y centre\0X left\0X right\0X centre\0";
  int16_t r = 0;
  PGM_P nm = segNames;
  for (uint8_t k = 0; k < 8 && !r; k++) {
    if (k >= 2) { Serial.print(F("  ")); Serial.println((const __FlashStringHelper*)nm); }
    r = travelMove(pgm_read_byte(&segAxis[k]), (int16_t)pgm_read_word(&segTo[k]), TRAVEL_DWELL);
    nm += strlen_P(nm) + 1;
  }
  motors(false);
  if (r == NB_T_KEY) Serial.println(F("ABORTED: key pressed."));
  else if (r == NB_T_FOLLOW) {
    Serial.print(F("ABORTED: ")); Serial.print(travAbort[0] ? 'Y' : 'X');
    Serial.print(F(" did not follow: target ")); Serial.print(travAbort[1]);
    Serial.print(F(", stick ")); Serial.print(travAbort[2]);
    Serial.print(F(", duty ")); nbPrintX10((int16_t)((int32_t)travAbort[3] * 1000L / 799L));
    Serial.println(F(" %. Hand? Belt? E-Stop?"));
  }
  else Serial.println(F("Travel test done."));
  printRange('Y', 1);
  printRange('X', 0);
  Serial.println(F("Motors disabled."));
  at(r ? F("TRAVEL FAIL") : F("TRAVEL OK"));
}

// ================================================================
// Commands
// ================================================================
static bool runStep(uint8_t s) {
  switch (s) {
    case NB_ST_ELECTRONICS: return stepElectronics();
    case NB_ST_CALIBRATION: return stepCalibration();
    case NB_ST_MOTORS:      return stepMotors();
    default:                return stepMeasure();
  }
}

// SETUP: the steps that are not done yet, in order. Stops at the first one that fails.
static void wizard() {
  bool first = true;
  for (uint8_t s = 0; s < NB_STEPS; s++) {
    if (nbStepDone(s)) continue;
    if (!first) {
      Serial.print(F("\nPress Enter to continue with ")); nbPrintStep(s); Serial.println('.');
      ask(F("ENTER NEXT"));
      waitEnter();
    }
    first = false;
    if (!runStep(s)) return;
  }
  travelTest();                        // all 4 steps done: optional visual check
  Serial.println();
  printNext();
}

static void monitor() {
  Serial.println(F("MON: X_raw Y_raw | X Y (HID) | Rx Ry Rz | buttons (hex). Any key stops."));
  nbEncFilter = false;
  while (!nbKeyAbort()) {
    for (uint8_t i = 0; i < 30; i++) { gripPoll(); delay(5); }
    uint16_t ax = nbReadAngle(NB_CH_X), ay = nbReadAngle(NB_CH_Y);
    Serial.print(ax); Serial.print('\t'); Serial.print(ay); Serial.print(F("\t| "));
    Serial.print(nbHidFromCont(0, nbUnwrap(ax, cfg.centerX))); Serial.print('\t');
    Serial.print(nbHidFromCont(1, nbUnwrap(ay, cfg.centerY))); Serial.print(F("\t| "));
    for (uint8_t ch = 1; ch <= 3; ch++) { Serial.print(gripAxis(ch)); Serial.print('\t'); }
    Serial.print(F("| ")); Serial.println(nbReadButtons(), HEX);
  }
  nbEncFilter = true;
  nbEncBegin();
}

// Progress and the whole stored configuration, one KEY:value per line.
// One KEY:value line per stored value. Names packed in flash, same order as statPtrs[]:
// the first STAT_N8 point to 8-bit values, the rest to 16-bit values.
//   DIR_X/Y = encoder direction (step 2), MDIR_X/Y = motor direction (step 3),
//   INV_AX/AY = axis inversion for the sim (set in FFB_Joystick, kept here)
const char statNames[] PROGMEM =
  "STEPS\0DIR_X\0DIR_Y\0MDIR_X\0MDIR_Y\0INV_AX\0INV_AY\0FRIC_X+\0FRIC_X-\0FRIC_Y+\0FRIC_Y-\0"
  "DCAP\0MINE\0SLEW\0FFBMS\0VD\0TG\0SG\0DG\0IG\0FG\0PG\0CG\0PF\0ATB1\0ATB2\0"
  "CENTER_X\0CENTER_Y\0MIN_X\0MAX_X\0MIN_Y\0MAX_Y\0TD_NS_X\0TD_NS_Y\0DTX\0DTY\0I2CERR\0GLX\0GLY\0";
#define STAT_N8 26
const void* const statPtrs[] PROGMEM = {
  &cfg.steps, &cfg.invAX, &cfg.invAY, &cfg.invMX, &cfg.invMY, &cfg.uinvX, &cfg.uinvY,
  &cfg.fric10[0][0], &cfg.fric10[0][1], &cfg.fric10[1][0], &cfg.fric10[1][1],
  &cfg.dutyCap, &cfg.minEff, &cfg.slew, &cfg.ffbMs, &cfg.velDamp, &cfg.tg, &cfg.sg, &cfg.dg,
  &cfg.ig, &cfg.fg, &cfg.pg, &cfg.cg, &cfg.pf, &cfg.atb1, &cfg.atb2,
  &cfg.centerX, &cfg.centerY, &cfg.minX, &cfg.maxX, &cfg.minY, &cfg.maxY,
  &cfg.tdNs[0], &cfg.tdNs[1], &cfg.dtX, &cfg.dtY, &nbI2cErr, &nbGlitch[0], &nbGlitch[1],
};

static void printStatus() {
  Serial.println(F("\n=== FFB SETUP STATUS (copy this block for support) ==="));
  Serial.println(F("Firmware: " FW_NAME " " NB_VERSION));
  printProgress();
  PGM_P nm = statNames;
  for (uint8_t i = 0; i < sizeof(statPtrs) / sizeof(statPtrs[0]); i++) {
    const void* p = pgm_read_ptr(&statPtrs[i]);
    nbKV((const __FlashStringHelper*)nm, (i < STAT_N8) ? (long)*(const uint8_t*)p : (long)*(const uint16_t*)p);
    nm += strlen_P(nm) + 1;
  }
  Serial.println(F("=== END ==="));
}

static void resetConfig() {
  Serial.println(F("RESET erases calibration, measurements, tuning, curves and steps."));
  Serial.println(F("Type YES to confirm."));
  ask(F("YES RESET"));
  if (!waitYes()) { Serial.println(F("Cancelled.")); return; }
  nbConfigDefaults();
  nbConfigSave();
  EEPROM.update(NB_EE_CURVE, 0xFF);    // curve block magic: FFB_Joystick starts with flat curves
  nbDriveDerive();
  Serial.println(F("Erased."));
  printNext();
}

static void help() {
  Serial.println(F("SETUP, CHECK CAL MOTORS MEASURE (step 1-4), TRAVEL GRIP MON STATUS RESET"));
}

void setup() {
  nbMotorsBegin();                     // D7 LOW first: the motors cannot start here
  Serial.begin(NB_SERIAL_BAUD);
  Serial.setTimeout(20);
  nbI2cBegin();
  bool found = nbConfigLoad();
  nbDriveDerive();
  nbEncBegin();
  nbVccBegin();
  nbGripBegin();
  while (!Serial) { }                  // wait for the Serial Monitor
  delay(100);
  Serial.println();
  Serial.println(F("=== Noisy Boi FFB - " FW_NAME " " NB_VERSION " ==="));
  if (!found) Serial.println(F("No configuration stored: reference values."));
  printProgress();
  printNext();
  Serial.println(F("Type HELP for all commands."));
  at(F("READY"));
}

void loop() {
  char cmd[12];
  if (!nbReadCmd(cmd, sizeof(cmd))) return;
  nbSplitArg(cmd);
  if (setjmp(cancelJmp)) {             // C at a prompt (cancelOn): undo what is not stored
    motors(false);
    nbSetPwmFreq(20);
    nbConfigLoad();
    nbDriveDerive();
    nbEncFilter = true;
    nbEncBegin();
    Serial.println(F("CANCELLED: nothing stored."));
    atSteps();
  }
  else if (!strcmp_P(cmd, PSTR("SETUP")))   wizard();
  else if (!strcmp_P(cmd, PSTR("CHECK")))   { stepElectronics(); printNext(); }
  else if (!strcmp_P(cmd, PSTR("CAL")))     { stepCalibration(); printNext(); }
  else if (!strcmp_P(cmd, PSTR("MOTORS")))  { stepMotors(); printNext(); }
  else if (!strcmp_P(cmd, PSTR("MEASURE"))) { stepMeasure(); printNext(); }
  else if (!strcmp_P(cmd, PSTR("TRAVEL")))  travelTest();
  else if (!strcmp_P(cmd, PSTR("GRIP")))    gripTest();
  else if (!strcmp_P(cmd, PSTR("MON")))     monitor();
  else if (!strcmp_P(cmd, PSTR("STATUS")))  printStatus();
  else if (!strcmp_P(cmd, PSTR("RESET")))   resetConfig();
  else help();
  at(F("READY"));
}
