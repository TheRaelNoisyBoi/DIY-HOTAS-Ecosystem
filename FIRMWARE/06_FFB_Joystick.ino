/***************************************************************************************
  Noisy Boi FFB base - FFB_Joystick
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Noisy Boi Designs

  Joystick and force feedback firmware. The base must have finished FFB_Setup (steps
  1 to 4): until then the joystick works but START is refused.

  The motors stay off after power-up: type START in the Serial Monitor (115200 baud,
  "Newline") to enable them, STOP to disable them. Settings are stored at once.

  Force path, every FFBMS ms (2 ms tick):
    library force (+-255, spring gain shaped by the curves) -> velocity damping (VD)
    -> slew limit (SLEW) -> motor direction -> IBT-2 dead-time offset (DTX/DTY) with
    MINE hysteresis -> speed cap (DCAP) -> PWM

  Grip axes Rx / Ry / Rz: filtered like the HFB joystick (median of 3, One Euro low-pass,
  output hysteresis; GFIL / GBETA / GJIT), ADS1115 at 475 SPS with a 2 ms gate as in the HFB,
  readings compensated for the USB supply so that a grip calibrated on the HFB base reads the
  same here. X / Y (encoders) are unchanged.

  SAFETY STOP: if the spring pushes the stick AWAY from its centre (a wrong force
  feedback axis setting in the sim), the motors are switched off. Type START again
  after fixing the setting.

  Spring gain curves: per axis and per side of the spring centre sent by the sim (the
  trim), 9 points at 0, 1/8 ... 8/8 of the half travel, each a multiplier of SG in %
  (0-150, 100 = unchanged). Positive side = stick right (X) / pulled back (Y).
    effective spring gain = SG x curve(deflection) / 100
  All points at 100 = the spring of firmware 1.0.4.

  Commands:
    STATUS, START, STOP
    (STATUS also shows VCC_MV: the USB supply used by the grip compensation, as VCC on the HFB)
    DEFAULTS           drive values, gains and curves back to the reference values
    TG SG DG IG FG <n> gains, % of what the sim sends (0-100: total, spring, damper,
                       inertia, friction)
    PG <n>             gain of every periodic effect (buffet, vibrations), 0-100
    CG <n>             gain of the constant force, 0-100
    PF <n>             frequency of the periodic effects, % (100 = as sent, 25-255)
    CURVE <XP|XN|YP|YN> v0 .. v8    spring gain curve, 9 values 0-150
    SLEW <n>           force slew limit, force units per ms (0 = off)
    FFBMS <n>          force update period, ms (1-100)
    VD <n>             velocity damping (0-100, 0 = off)
    DTX <n> DTY <n>    IBT-2 dead-time offset, 0.1 % (0-250, measured by FFB_Setup)
    DCAP <n>           speed cap: hard limit of the motor duty, % (1-50)
    MINE <n>           effective duty below this is zero, 0.1 % (0-50)
    GFIL <n>           grip One Euro min cutoff, 0.1 Hz (1-200, 10 = 1.0 Hz = FIL2 of the HFB):
                       lower = smoother at rest
    GBETA <n>          grip One Euro speed coefficient, 0.1 (0-250, 20 = 2.0 = BETA2 of the
                       HFB): higher = less lag when the axis moves
    GJIT <n>           grip output hysteresis, HID counts of 1023 (0-20, 0 = off)
    INV_AX, INV_AY     flip an axis for the sim (its force flips with it, the spring still
                       pulls to the centre). Kept when the base is recalibrated.
    ATB1, ATB2         grip mini-stick -> HAT, brake axis -> button 28 (buttons 1+3 toggle both)
    HAPTIC_RTP_ON, HAPTIC_RTP_OFF, RTP <n>     grip vibration (used by the GUI)
    TRACE ON, TRACE OFF                        only when built with NB_TRACE 1
  Calibration, encoder and motor directions, measurements: FFB_Setup.

  Notes: DG and IG have no effect with this FFB library (it never receives the stick
  speed or acceleration); use VD for damping. Keep FG at 0: the library feeds the
  friction effect the stick position, so it acts as a second spring.

  TRACE line (100 Hz, NB_TRACE 1): "T:ms,X,FX,OX,Y,FY,OY,CX,CY,N,M,K,G" (values only, this order)
    X/Y HID position, FX/FY library force, OX/OY motor duty counts before the enable gate,
    CX/CY spring centre (-10000..10000), N spring condition blocks, M playing effect types
    (bit n = type n: 256 spring, +8 square, +16 sine), K spring coefficient, G encoder glitches

  Library: JoystickWithFFB by YukMingLaw (LGPL-3.0, unmodified, ZIP from GitHub). The grip
  chips (ADS1115, PCF8575) are driven directly: no other library is needed.
  Board: Arduino Micro (or SparkFun Pro Micro 5 V / 16 MHz).
***************************************************************************************/
#include <Wire.h>
#include <EEPROM.h>
#include <Joystick.h>

#define FW_NAME "FFB_Joystick"

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
// FLIGHT FIRMWARE (FFB_Joystick only)
// ######################################################################################

#define NB_TRACE         0          // 1 = TRACE command for debugging (simpler grip filter)
#define ALPHA_ENC_256    77         // encoder EMA per 2 ms tick, 77/256 = 0.30 (time constant ~5 ms)
#define CTRL_TICK_US     2000UL     // encoders + force path, 500 Hz
#define HID_SEND_MS      2          // HID report period
#define NB_PF_MIN_PERIOD 20         // ms, fastest periodic effect after PF scaling (50 Hz)
#define NB_BUTTON_POLL_MS 4

// The joystick seen by Windows. Do not change it: DCS keeps its bindings per device.
//   (report id, type, buttons, hats, X, Y, Z, Rx, Ry, Rz, rudder, throttle, accel, brake, steering)
#define NB_JOYSTICK_ARGS JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, NB_NUM_BUTTONS, 1, \
                         true, true, false, true, true, true, false, false, false, false, false

// Numeric settings "NAME <value>" (names packed in flash, same order as numSets[]).
struct NumSet { void* p; uint8_t lo, hi, flags; };

Joystick_ Joystick(NB_JOYSTICK_ARGS);

int32_t      forces[2] = {0, 0};
EffectParams ffbParams[2];
Gains        nbGains[2];
uint16_t     nbPerOrig[MAX_EFFECTS], nbPerSet[MAX_EFFECTS];
uint32_t     nbButtons = 0;
bool         hapticRtp = false;
#if NB_TRACE
bool         traceOn = false;
#endif
uint8_t      safetyStops = 0;
TEffectState* curSpring = 0;       // first playing spring of this force tick (or NULL)
uint16_t     curMask = 0;          // playing effect types of this force tick

// ================================================================
// Spring gain curves (EEPROM block at NB_EE_CURVE, own CRC)
// ================================================================
// curve[k][i]: k = 0 XP, 1 XN, 2 YP, 3 YN; point i at i/8 of the half travel (64 HID
// counts per step), value = multiplier of SG in % (0..150). Missing or corrupt block:
// flat curves (100), the 1.0.4 behaviour. A change is written one byte per loop pass,
// only while the EEPROM is idle, so the force loop never waits for the EEPROM.
#define NB_CRV_PTS    9
#define NB_CRV_MAX    150
#define NB_CRV_MAGIC  0x4356     // "CV"
struct NbCurves {
  uint16_t magic;
  uint8_t  v[4][NB_CRV_PTS];
  uint16_t crc;
};
NbCurves crv;
static uint8_t crvFlush = sizeof(NbCurves);   // next byte to write; sizeof = nothing pending

static void crvFlat() {
  memset(crv.v, 100, sizeof(crv.v));
}

static void crvLoad() {
  uint8_t* p = (uint8_t*)&crv;
  for (uint8_t i = 0; i < sizeof(NbCurves); i++) p[i] = EEPROM.read(NB_EE_CURVE + i);
  bool ok = crv.magic == NB_CRV_MAGIC && crv.crc == nbCrc16((const uint8_t*)&crv, sizeof(NbCurves) - 2);
  for (uint8_t i = 0; ok && i < sizeof(crv.v); i++) ok = crv.v[0][i] <= NB_CRV_MAX;
  if (!ok) crvFlat();
}

// New values in crv.v: recompute the CRC and start the background write.
static void crvSave() {
  crv.magic = NB_CRV_MAGIC;
  crv.crc = nbCrc16((const uint8_t*)&crv, sizeof(NbCurves) - 2);
  crvFlush = 0;
}

// One changed byte per call, never waiting for the EEPROM.
static void crvFlushStep() {
  while (crvFlush < sizeof(NbCurves) && eeprom_is_ready()) {
    uint8_t b = ((const uint8_t*)&crv)[crvFlush];
    uint16_t a = NB_EE_CURVE + crvFlush++;
    if (EEPROM.read(a) != b) { EEPROM.write(a, b); return; }
  }
}

// Stick - spring centre of axis a, in HID counts (sim frame). The centre comes from the
// spring condition block of that axis: cpOffset -10000..10000 = the whole travel
// (-512..512 HID), i.e. 0.0512 HID per unit (839 / 16384).
static int16_t springDisp(TEffectState* sp, uint8_t a, int16_t hid) {
  uint8_t c = (sp->conditionBlocksCount > 1) ? a : 0;
  return hid - 511 - (int16_t)(((int32_t)sp->conditions[c].cpOffset * 839L) >> 14);
}

// Effective spring gain of axis a for the spring playing now (cfg.sg when none plays).
//   d = stick - spring centre, in the calibration frame (+ = right / pulled back)
static uint8_t crvSpringGain(uint8_t a, int16_t hidUser) {
  if (!curSpring) return cfg.sg;
  int16_t d = springDisp(curSpring, a, hidUser);
  if (a ? cfg.uinvY : cfg.uinvX) d = -d;          // sim frame -> calibration frame
  const uint8_t* k = crv.v[a * 2 + (d < 0)];
  uint16_t m = (uint16_t)((d < 0) ? -d : d);
  uint8_t i = (uint8_t)(m >> 6);
  int16_t g = k[NB_CRV_PTS - 1];
  if (i < NB_CRV_PTS - 1) g = (int16_t)(k[i] * 64 + (k[i + 1] - k[i]) * (int16_t)(m & 63)) >> 6;
  uint16_t sg = (uint16_t)(cfg.sg * (uint16_t)g) / 100U;   // <= 100 x 150: no overflow
  return (uint8_t)sg;
}

// ================================================================
// FFB library helpers
// ================================================================
static TEffectState* nbEffectTable() {
  return (TEffectState*)DynamicHID().pidReportHandler.g_EffectStates;
}

// Call right after Joystick.getUSBPID(), before Joystick.getForce():
//  - library bug workaround: PIDReportHandler::SetCondition() increments
//    conditionBlocksCount on every Set Condition report and never resets it. DCS sends one
//    report per axis on every trim step, so the counter wraps; at 0 the Y axis used the X
//    block (wrong centre), at 1 both axes were scaled by the effect direction. Keeping it
//    at most 2 keeps a 2-block effect in "one block per axis" mode.
//  - PF (cfg.pf): rescales the period of periodic effects. nbPerSet[] holds the value
//    written here; a different value in the table means the sim sent a new period.
static void nbFixEffects() {
  TEffectState* tab = nbEffectTable();
  uint8_t pf = cfg.pf ? cfg.pf : 100;
  for (uint8_t id = 0; id < MAX_EFFECTS; id++) {
    TEffectState& e = tab[id];
    if (e.conditionBlocksCount > 2) e.conditionBlocksCount = 2;
    if (e.effectType < USB_EFFECT_SQUARE || e.effectType > USB_EFFECT_SAWTOOTHUP || !e.period) continue;
    if (e.period != nbPerSet[id]) nbPerOrig[id] = e.period;
    uint16_t p = (uint16_t)(((uint32_t)nbPerOrig[id] * 100UL) / pf);
    if (p < NB_PF_MIN_PERIOD) p = NB_PF_MIN_PERIOD;
    e.period = p;
    nbPerSet[id] = p;
  }
}

// Gains from cfg. PG drives every periodic effect.
static void nbApplyGains() {
  for (uint8_t i = 0; i < 2; i++) {
    Gains& g = nbGains[i];
    g.totalGain = cfg.tg;
    g.constantGain = cfg.cg;
    g.rampGain = 100;
    g.squareGain = g.sineGain = g.triangleGain = g.sawtoothdownGain = g.sawtoothupGain = cfg.pg;
    g.springGain = cfg.sg;
    g.damperGain = cfg.dg;
    g.inertiaGain = cfg.ig;
    g.frictionGain = cfg.fg;
    g.customGain = 100;
  }
  Joystick.setGains(nbGains);
}

// Playing effects, once per force tick: curSpring = first playing spring (or NULL),
// curMask bit n = effect type n playing.
static void nbFindSpring() {
  TEffectState* tab = nbEffectTable();
  TEffectState* sp = 0;
  uint16_t m = 0;
  for (uint8_t id = 0; id < MAX_EFFECTS; id++) {
    TEffectState& e = tab[id];
    if (e.state != MEFFECTSTATE_PLAYING) continue;
    m |= (uint16_t)1 << (e.effectType & 15);
    if (e.effectType == USB_EFFECT_SPRING && !sp) sp = &e;
  }
  curMask = m;
  curSpring = sp;
}

// ================================================================
// Grip axes Rx / Ry / Rz: the filter of the HFB joystick firmware
// ================================================================
// Per sample (~100-150 Hz per axis): median of 3 (spike rejection), position through the
// grip calibration, One Euro low-pass (strong smoothing at rest, little lag when the axis
// moves: cutoff = MINCUT + BETA * speed), then output hysteresis in HID counts. Same values
// as the HFB grip defaults: FIL2 1.0 Hz, BETA2 2.0 (on its -1..1 range), D cutoff 1 Hz.
// Here the position is in HID counts (0..1023), so BETA becomes 2.0 / (1023 / 2) per count.
// Min cutoff (GFIL), speed coefficient (GBETA) and hysteresis (GJIT) are settings, see
// cfg.gFil / gBeta / gJit. The derivative cutoff is fixed at 1 Hz, as in the HFB.

struct GripAxis {
  int16_t  h[3];                   // last 3 samples (counts at 5.000 V)
  uint8_t  n;                      // 0 = (re)start, 2 = running
  float    x, dx;                  // filtered position (HID counts) and speed (counts/us)
  uint32_t us;
  int16_t  out;                    // HID output - 511 (zero = centred, also at power-up)
};
static GripAxis grip[3];

#if !NB_TRACE
// First-order low-pass step: alpha = w / (1 + w), w = 2 pi f dt
static void lowPass(float* y, float in, float w) __attribute__((noinline));
static void lowPass(float* y, float in, float w) {
  *y += w / (1.0f + w) * (in - *y);
}
#endif

static int16_t median3(const int16_t* h) {
  int16_t a = h[0], b = h[1], c = h[2];
  if (a > b) { int16_t t = a; a = b; b = t; }
  return (c < a) ? a : (c > b) ? b : c;
}

// New sample of channel ch (1 Rx, 2 Ry, 3 Rz): filter it and update the HID output
// (Rz inverted as in v7.0; centred when the axis is disabled in the grip calibration).
static void gripSample(uint8_t ch) {
  GripAxis g;                                  // work on a copy: much smaller AVR code
  memcpy(&g, &grip[ch - 1], sizeof(g));
  int16_t raw = nbGripRaw[ch];
  g.h[0] = g.n ? g.h[1] : raw;
  g.h[1] = g.n ? g.h[2] : raw;
  g.h[2] = raw;
  uint16_t lo, hi;
  if (!nbGripRange(ch, &lo, &hi)) { g.n = 0; g.out = 0; }
  else {
    uint16_t m = (uint16_t)(constrain(median3(g.h), (int16_t)lo, (int16_t)hi) - lo);
    float p = (float)((uint32_t)m * (16UL * NB_AXIS_MAX) / (uint16_t)(hi - lo)) * (1.0f / 16.0f);
#if NB_TRACE
    // debug build: no room for the One Euro filter (median and hysteresis only)
    g.x = p; g.n = 2;
#else
    uint32_t now = micros();
    // dt in us (a channel is sampled every ~7 ms, never faster than 3 ADS conversions)
    float dt = (float)min(now - g.us, 50000UL);
    g.us = now;
    if (g.n < 2) { g.x = p; g.dx = 0; g.n = 2; }
    else {
      float w = 6.2831853e-6f * dt;           // 2 pi f dt for f = 1 Hz (derivative cutoff)
      lowPass(&g.dx, (p - g.x) / dt, w);
      // cutoff = GFIL + GBETA * speed; the HFB speed is in its -1..1 range per s, here in
      // HID counts per us: x 2 / 1023 x 1e6
      lowPass(&g.x, p, w * ((float)cfg.gFil * 0.1f +
                            (float)cfg.gBeta * (0.1f * 2.0f / NB_AXIS_MAX * 1e6f) * fabs(g.dx)));
    }
#endif
    int16_t h = (int16_t)(g.x + 0.5f);
    if (ch == 3) h = NB_AXIS_MAX - h;
    h -= 511;
    if (abs(h - g.out) >= cfg.gJit || h == -511 || h == NB_AXIS_MAX - 511) g.out = h;
  }
  memcpy(&grip[ch - 1], &g, sizeof(g));
}

// Grip inputs -> joystick: Rx/Ry/Rz, 27 buttons, ATB modes (buttons 1+3 toggle both).
//   ATB1: grip mini-stick -> HAT (Rx/Ry centred)    ATB2: brake axis -> button 28 (Rz centred)
static void nbHidInputs(uint32_t nowMs) {
  uint8_t ch = nbGripPoll();
  if (ch) gripSample(ch);
  int16_t rx = grip[0].out + 511, ry = grip[1].out + 511, rz = grip[2].out + 511;

  static uint32_t lastBtnMs = 0;
  if ((uint32_t)(nowMs - lastBtnMs) >= NB_BUTTON_POLL_MS) {
    lastBtnMs = nowMs;
    uint32_t b = nbReadButtons();
    uint32_t changed = b ^ nbButtons;
    nbButtons = b;
    for (uint8_t i = 0; changed; i++, changed >>= 1, b >>= 1) {
      if (changed & 1) Joystick.setButton(i, b & 1);
    }
  }

  static bool atbPrev = false;
  bool b1 = nbButtons & 1, b3 = (nbButtons >> 2) & 1;
  if (b1 && b3 && !atbPrev) {
    uint8_t on = (cfg.atb1 || cfg.atb2) ? 0 : 1;
    cfg.atb1 = on; cfg.atb2 = on;
    nbConfigSave();
    atbPrev = true;
  } else if (!b1 || !b3) {
    atbPrev = false;
  }

  if (cfg.atb1) {
    // index = up | down << 1 | right << 2 | left << 3; value = angle / 45, -1 = released
    static const int8_t hatTab[16] PROGMEM = { -1, 0, 4, -1, 2, 1, 3, -1, 6, 7, 5, -1, -1, -1, -1, -1 };
    uint8_t hidx = (ry > 750) | ((ry < 250) << 1) | ((rx > 750) << 2) | ((rx < 250) << 3);
    int8_t hv = (int8_t)pgm_read_byte(&hatTab[hidx]);
    Joystick.setHatSwitch(0, (hv < 0) ? -1 : hv * 45);
    Joystick.setRxAxis(511);
    Joystick.setRyAxis(511);
  } else {
    Joystick.setHatSwitch(0, -1);
    Joystick.setRxAxis(rx);
    Joystick.setRyAxis(ry);
  }

  static bool brakeLast = false;
  if (cfg.atb2) {
    if      (rz > 600 && !brakeLast)  { Joystick.setButton(27, 1); brakeLast = true;  }
    else if (rz <= 600 && brakeLast)  { Joystick.setButton(27, 0); brakeLast = false; }
    Joystick.setRzAxis(511);
  } else {
    if (brakeLast) { Joystick.setButton(27, 0); brakeLast = false; }
    Joystick.setRzAxis(rz);
  }
}

// ================================================================
// Safety: the spring must pull the stick towards its centre
// ================================================================
// A spring that pushes the stick AWAY from its centre runs the stick into the end stop
// (it happens with a wrong force feedback axis setting in the sim). Checked on the
// library force while a spring plays and no constant or ramp force does (those may push
// the stick anywhere on purpose): stick more than SAFE_DISP from the spring centre with
// a force above SAFE_FORCE in the SAME direction for SAFE_MS -> motors off.
#define SAFE_DISP   150     // HID counts from the spring centre
#define SAFE_FORCE  10      // library force units (of 255)
#define SAFE_MS     300

static uint16_t awayMs[2] = {0, 0};

static void safetyCheck(int16_t xHID, int16_t yHID, uint8_t dtMs) {
  TEffectState* sp = curSpring;
  bool check = nbMotorOn && sp && !(curMask & ((1 << USB_EFFECT_CONSTANT) | (1 << USB_EFFECT_RAMP)));
  for (uint8_t a = 0; a < 2; a++) {
    bool away = false;
    if (check) {
      int16_t d = springDisp(sp, a, a ? yHID : xHID);
      int16_t f = (int16_t)forces[a];
      away = (d > SAFE_DISP && f > SAFE_FORCE) || (d < -SAFE_DISP && f < -SAFE_FORCE);
    }
    awayMs[a] = away ? (uint16_t)(awayMs[a] + dtMs) : 0;
    if (awayMs[a] >= SAFE_MS) {
      nbMotorEnable(false);
      awayMs[0] = awayMs[1] = 0;
      if (safetyStops < 255) safetyStops++;
      Serial.print(F("SAFETY STOP: the ")); Serial.print(a ? 'Y' : 'X');
      Serial.println(F(" spring pushes AWAY from its centre, motors OFF. Check the FFB axes of the sim (inverted? swapped?), then START."));
      return;
    }
  }
}

// ================================================================
// Velocity: least-squares slope over 12 samples, one every 5 ms (60 ms window)
// ================================================================
#define VEL_N        12
#define VEL_TICK_MS  5
#define VD_LIMIT     64       // limit of the damping term, force units
static int16_t velBuf[2][VEL_N];
static uint8_t velHead = 0;
static int16_t velX = 0, velY = 0;          // HID counts per second
static uint32_t lastVelMs = 0;          // (the buffer starts empty: the first 60 ms after
                                         //  power-up, motors still OFF, read a false speed)

static int16_t velSlope(const int16_t* b) {
  // sum((2i - 11) * y_i), i = 0 oldest, as sum(w * (y_newer - y_older)) over 6 pairs
  int32_t sum = 0;
  uint8_t o = velHead;
  uint8_t n = velHead ? velHead - 1 : VEL_N - 1;
  for (int8_t w = VEL_N - 1; w > 0; w -= 2) {
    sum += (int16_t)(w * (b[n] - b[o]));
    if (++o == VEL_N) o = 0;
    n = n ? n - 1 : VEL_N - 1;
  }
  return (int16_t)((sum * 1000L) / (286L * VEL_TICK_MS));   // counts per second
}

static void velUpdate(int16_t xHID, int16_t yHID, uint32_t nowMs) {
  if ((uint32_t)(nowMs - lastVelMs) < VEL_TICK_MS) return;
  lastVelMs += VEL_TICK_MS;
  if ((uint32_t)(nowMs - lastVelMs) >= VEL_TICK_MS) lastVelMs = nowMs;   // after a stall
  velBuf[0][velHead] = xHID;
  velBuf[1][velHead] = yHID;
  velHead = (velHead + 1) % VEL_N;
  velX = velSlope(velBuf[0]);
  velY = velSlope(velBuf[1]);
}

// ================================================================
// Serial
// ================================================================
static void echoCmd(const char* cmd, long v) {
  Serial.print(cmd); Serial.print(F(": ")); Serial.println(v);
}

// STATUS: one KEY:value line per setting. Names packed in flash, same order as statPtrs[]:
// the first STAT_N8 point to 8-bit values, the rest to 16-bit values.
//   INV_AX/AY = axis inversion for the sim, DIR_X/Y = encoder direction (calibration),
//   MDIR_X/Y = motor direction (setup step 3)
const char statNames[] PROGMEM =
  "STEPS\0MOTOR_EN\0SAFETY_STOPS\0INV_AX\0INV_AY\0DIR_X\0DIR_Y\0MDIR_X\0MDIR_Y\0DCAP\0MINE\0"
  "TG\0SG\0DG\0IG\0FG\0PG\0CG\0SLEW\0PF\0ATB1\0ATB2\0FFBMS\0VD\0HAPTIC_RTP\0DRV\0"
  "GFIL\0GBETA\0GJIT\0DTX\0DTY\0I2CERR\0GLX\0GLY\0";
#define STAT_N8 29
const void* const statPtrs[] PROGMEM = {
  &cfg.steps, &nbMotorOn, &safetyStops, &cfg.uinvX, &cfg.uinvY, &cfg.invAX, &cfg.invAY,
  &cfg.invMX, &cfg.invMY, &cfg.dutyCap, &cfg.minEff, &cfg.tg, &cfg.sg, &cfg.dg, &cfg.ig, &cfg.fg,
  &cfg.pg, &cfg.cg, &cfg.slew, &cfg.pf, &cfg.atb1, &cfg.atb2, &cfg.ffbMs, &cfg.velDamp, &hapticRtp,
  &nbDrvPresent, &cfg.gFil, &cfg.gBeta, &cfg.gJit, &cfg.dtX, &cfg.dtY, &nbI2cErr, &nbGlitch[0], &nbGlitch[1],
};

const char crvNames[] PROGMEM = "XP\0XN\0YP\0YN\0";

// "CRV_XP:100,100,..." (STATUS and the answer to CURVE)
static void printCurve(uint8_t k) {
  Serial.print(F("CRV_"));
  Serial.print((const __FlashStringHelper*)(crvNames + k * 3));
  for (uint8_t i = 0; i < NB_CRV_PTS; i++) {
    Serial.print(i ? ',' : ':');
    Serial.print(crv.v[k][i]);
  }
  Serial.println();
}

static void printStatus() {
  Serial.println(F("\n=== FFB BASE STATUS ==="));
  Serial.println(F("Firmware: " FW_NAME " " NB_VERSION));
  PGM_P nm = statNames;
  for (uint8_t i = 0; i < sizeof(statPtrs) / sizeof(statPtrs[0]); i++) {
    const void* p = pgm_read_ptr(&statPtrs[i]);
    nbKV((const __FlashStringHelper*)nm, (i < STAT_N8) ? (long)*(const uint8_t*)p : (long)*(const uint16_t*)p);
    nm += strlen_P(nm) + 1;
  }
  // USB supply in mV used by the grip compensation (the VCC of the HFB joystick):
  // Vcc = 1100 mV * 1023 / bandgap reading, bgAcc >> 9 = reading x 128
#if !NB_TRACE
  nbKV(F("VCC_MV"), 144038400L / (long)(bgAcc >> 9));
#endif
  for (uint8_t k = 0; k < 4; k++) printCurve(k);
  Serial.println(F("=== END ==="));
}

// "XP 100 100 ... 100" -> curve k. Exactly 9 values, each clamped to 0..150.
static void curveCmd(char* arg) {
  char* p = arg;
  int8_t k = -1;
  if (p && p[0] && p[1] && p[2] == ' ') {
    for (uint8_t i = 0; i < 4; i++) {
      if (p[0] == pgm_read_byte(crvNames + i * 3) && p[1] == pgm_read_byte(crvNames + i * 3 + 1)) k = i;
    }
    p += 3;
  }
  uint8_t v[NB_CRV_PTS];
  uint8_t n = 0;
  while (k >= 0 && n <= NB_CRV_PTS) {
    while (*p == ' ') p++;
    if (*p < '0' || *p > '9') break;
    uint16_t x = 0;
    while (*p >= '0' && *p <= '9') { if (x < 1000) x = x * 10 + (*p - '0'); p++; }
    if (n < NB_CRV_PTS) v[n] = (uint8_t)min(x, (uint16_t)NB_CRV_MAX);
    n++;
  }
  if (k < 0 || n != NB_CRV_PTS || *p) {
    Serial.println(F("CURVE: XP|XN|YP|YN + 9 values 0-150"));
    return;
  }
  memcpy(crv.v[k], v, NB_CRV_PTS);
  crvSave();
  printCurve(k);
}

const char numNames[] PROGMEM =
  "TG\0SG\0DG\0IG\0FG\0FFBMS\0VD\0DTX\0DTY\0DCAP\0MINE\0PG\0CG\0SLEW\0PF\0"
#if !NB_TRACE
  "GFIL\0GBETA\0GJIT\0"              // (the debug build has no One Euro filter: no room)
#endif
  ;
#define NS_16    1   // uint16_t field (else uint8_t)
#define NS_GAINS 2   // re-apply the FFB gains
#define NS_DRIVE 4   // re-derive the drive counts
// Gains are limited to 100 (= what the sim sends): more force comes from the curves (up to
// 150 % of SG) and is always bounded by DCAP.
const NumSet numSets[] PROGMEM = {
  { &cfg.tg,      0, 100, NS_GAINS },
  { &cfg.sg,      0, 100, NS_GAINS },
  { &cfg.dg,      0, 100, NS_GAINS },
  { &cfg.ig,      0, 100, NS_GAINS },
  { &cfg.fg,      0, 100, NS_GAINS },
  { &cfg.ffbMs,   1, 100, 0 },
  { &cfg.velDamp, 0, 100, 0 },
  { &cfg.dtX,     0, 250, NS_16 | NS_DRIVE },
  { &cfg.dtY,     0, 250, NS_16 | NS_DRIVE },
  { &cfg.dutyCap, 1,  50, NS_DRIVE },
  { &cfg.minEff,  0,  50, NS_DRIVE },
  { &cfg.pg,      0, 100, NS_GAINS },
  { &cfg.cg,      0, 100, NS_GAINS },
  { &cfg.slew,    0, 255, 0 },
  { &cfg.pf,     25, 255, 0 },
#if !NB_TRACE
  { &cfg.gFil,    1, 200, 0 },    // grip min cutoff, 0.1 Hz
  { &cfg.gBeta,   0, 250, 0 },    // grip speed coefficient, 0.1
  { &cfg.gJit,    0,  20, 0 },    // grip output hysteresis, HID counts
#endif
};
// Toggles "NAME". INV_AX / INV_AY: axis inversion for the sim (cfg.uinvX / uinvY). The loop
// flips the HID axis AND the force of that axis, so the spring still pulls to the centre.
// The encoder and motor directions are measured by FFB_Setup, never set here.
const char togNames[] PROGMEM = "INV_AX\0INV_AY\0ATB1\0ATB2\0";
void* const togPtrs[] PROGMEM = { &cfg.uinvX, &cfg.uinvY, &cfg.atb1, &cfg.atb2 };

// Settings stored by an older firmware may be outside the 1.1.0 limits (gains up to 255,
// DCAP up to 60): clamp them once at power-up.
static void nbClampSettings() {
  bool changed = false;
  if (!cfg.gFil) {                    // block written before 1.1.2: grip filter defaults
    cfg.gFil = 10; cfg.gBeta = 20; cfg.gJit = 2;
    changed = true;
  }
  for (uint8_t k = 0; k < sizeof(numSets) / sizeof(numSets[0]); k++) {
    NumSet e;
    memcpy_P(&e, &numSets[k], sizeof(NumSet));
    uint16_t v = (e.flags & NS_16) ? *(uint16_t*)e.p : *(uint8_t*)e.p;
    uint16_t c = constrain(v, e.lo, e.hi);
    if (c == v) continue;
    if (e.flags & NS_16) *(uint16_t*)e.p = c; else *(uint8_t*)e.p = (uint8_t)c;
    changed = true;
  }
  if (changed) nbConfigSave();
}

// Boot warning and answer to START while FFB_Setup is not finished.
static void refuseMsg() {
  Serial.println(F("START refused: finish FFB_Setup first (type SETUP there)."));
}

static int8_t findName(PGM_P names, const char* s) {
  int8_t i = 0;
  while (pgm_read_byte(names)) {
    if (strcmp_P(s, names) == 0) return i;
    names += strlen_P(names) + 1;
    i++;
  }
  return -1;
}

static void processSerial() {
  // Several commands per pass, so high-rate RTP updates from the GUI do not pile up.
  for (uint8_t n = 0; n < 8 && Serial.available(); n++) {
    char cmd[52];            // longest: "CURVE XP 150 150 150 150 150 150 150 150 150"
    if (!nbReadCmd(cmd, sizeof(cmd))) continue;
    char* arg = nbSplitArg(cmd);
    int8_t k;

    if      (strcmp_P(cmd, PSTR("STATUS")) == 0) { printStatus(); }
#if NB_TRACE
    else if (strcmp_P(cmd, PSTR("TRACE")) == 0)  { traceOn = (arg && arg[1] == 'N'); }   // ON / OFF
#endif
    else if (strcmp_P(cmd, PSTR("STOP")) == 0)   { nbMotorEnable(false); Serial.println(F("STOP")); }
    else if (strcmp_P(cmd, PSTR("START")) == 0) {
      if ((cfg.steps & NB_ST_ALL) == NB_ST_ALL) { nbMotorEnable(true); Serial.println(F("START")); }
      else refuseMsg();
    }
    else if (strcmp_P(cmd, PSTR("DEFAULTS")) == 0) {
      nbConfigDefaultsTuning(); nbConfigSave(); nbDriveDerive(); nbApplyGains();
      crvFlat(); crvSave();
      Serial.println(F("DEFAULTS: done"));
    }
    else if (strcmp_P(cmd, PSTR("CURVE")) == 0) { curveCmd(arg); }
    else if (arg && (k = findName(numNames, cmd)) >= 0) {
      NumSet e;
      memcpy_P(&e, &numSets[k], sizeof(NumSet));
      int v = constrain(atoi(arg), (int)e.lo, (int)e.hi);
      if (e.flags & NS_16) *(uint16_t*)e.p = (uint16_t)v;
      else                 *(uint8_t*)e.p  = (uint8_t)v;
      nbConfigSave();
      if (e.flags & NS_GAINS) nbApplyGains();
      if (e.flags & NS_DRIVE) nbDriveDerive();
      echoCmd(cmd, v);
    }
    else if (!arg && (k = findName(togNames, cmd)) >= 0) {
      uint8_t* p = (uint8_t*)pgm_read_ptr(&togPtrs[k]);
      *p = !*p;
      nbConfigSave();
      echoCmd(cmd, *p);
    }
    else if (memcmp_P(cmd, PSTR("HAPTIC_RTP_O"), 12) == 0) {      // HAPTIC_RTP_ON / _OFF
      hapticRtp = (cmd[12] == 'N');
      nbDrvSetRtp(hapticRtp);
      nbKV(F("HAPTIC_RTP"), hapticRtp);              // "HAPTIC_RTP:1" / ":0", as in STATUS
    }
    else if (arg && strcmp_P(cmd, PSTR("RTP")) == 0) {
      // no echo: the GUI sends this at a high rate
      if (hapticRtp && nbDrvPresent) nbDrvWrite(NB_DRV_REG_RTPIN, (uint8_t)constrain(atoi(arg), 0, 127));
    }
  }
}

#if NB_TRACE

static void traceLine(uint32_t t, int16_t x, int16_t y, int16_t ox, int16_t oy) __attribute__((noinline));
static void traceLine(uint32_t t, int16_t x, int16_t y, int16_t ox, int16_t oy) {
  TEffectState* sp = curSpring;
  uint16_t mask = curMask;
  int16_t v[12] = { x, (int16_t)forces[0], ox, y, (int16_t)forces[1], oy,
                    sp ? sp->conditions[0].cpOffset : 0, sp ? sp->conditions[1].cpOffset : 0,
                    sp ? sp->conditionBlocksCount : 0, (int16_t)mask,
                    sp ? sp->conditions[0].positiveCoefficient : 0,
                    (int16_t)(nbGlitch[0] + nbGlitch[1]) };
  Serial.print(F("T:")); Serial.print(t);
  for (uint8_t i = 0; i < 12; i++) { Serial.print(','); Serial.print(v[i]); }
  Serial.println();
}
#endif

// ================================================================
// Setup / loop
// ================================================================
void setup() {
  nbMotorsBegin();                    // D7 LOW first: motors stay off until START
  Serial.begin(NB_SERIAL_BAUD);
  Serial.setTimeout(20);
  nbI2cBegin();
  nbConfigLoad();
  nbClampSettings();
  crvLoad();
  nbDriveDerive();
  nbEncBegin();
  nbVccBegin();
  nbGripBegin();
#if JOYSTICK_DEFAULT_AXIS_MINIMUM != 0 || JOYSTICK_DEFAULT_AXIS_MAXIMUM != NB_AXIS_MAX
  Joystick.setXAxisRange(0, NB_AXIS_MAX);      // (the library default is already 0..1023)
  Joystick.setYAxisRange(0, NB_AXIS_MAX);
  Joystick.setRxAxisRange(0, NB_AXIS_MAX);
  Joystick.setRyAxisRange(0, NB_AXIS_MAX);
  Joystick.setRzAxisRange(0, NB_AXIS_MAX);
#endif
  Joystick.begin(false);              // no auto send: one sendState() every HID_SEND_MS
  nbApplyGains();
  // (no wait for the Serial Monitor: the base must be usable right after power-up)
  Serial.println(F("=== Noisy Boi FFB - " FW_NAME " " NB_VERSION " (motors OFF until START) ==="));
  if ((cfg.steps & NB_ST_ALL) != NB_ST_ALL) refuseMsg();
}

static int32_t xFilt = 2048L << 8, yFilt = 2048L << 8;   // encoder EMA, unwrapped counts x 256

void loop() {
  processSerial();
  uint32_t nowMs = millis();

  // === 1. Encoders -> HID, fixed 2 ms tick ===
  static uint32_t lastCtrlUs = 0;
  uint32_t ctrlUs = micros();
  if ((uint32_t)(ctrlUs - lastCtrlUs) >= CTRL_TICK_US) {
    lastCtrlUs = ctrlUs;

    int16_t xu = nbUnwrap(nbReadAngle(NB_CH_X), cfg.centerX);
    int16_t yu = nbUnwrap(nbReadAngle(NB_CH_Y), cfg.centerY);
    xFilt += (((int32_t)xu << 8) - xFilt) * ALPHA_ENC_256 >> 8;
    yFilt += (((int32_t)yu << 8) - yFilt) * ALPHA_ENC_256 >> 8;
    // calibration frame -> frame of the sim (INV_AX / INV_AY)
    int16_t xHID = nbHidFromCont(0, (int16_t)(xFilt >> 8));
    int16_t yHID = nbHidFromCont(1, (int16_t)(yFilt >> 8));
    if (cfg.uinvX) xHID = NB_AXIS_MAX - xHID;
    if (cfg.uinvY) yHID = NB_AXIS_MAX - yHID;
    Joystick.setXAxis(xHID);
    Joystick.setYAxis(yHID);
    velUpdate(xHID, yHID, nowMs);

    // === 2. FFB library + motors, every FFBMS ms ===
    static uint32_t lastFfbMs = 0;
    uint8_t ffbPeriod = cfg.ffbMs ? cfg.ffbMs : 1;
    if ((uint32_t)(nowMs - lastFfbMs) >= ffbPeriod) {
      uint8_t dtMs = (uint8_t)min((uint32_t)(nowMs - lastFfbMs), (uint32_t)20);   // for SLEW
      lastFfbMs = nowMs;

      ffbParams[0].springMaxPosition = 512;
      ffbParams[0].springPosition    = xHID - 511;
      ffbParams[0].frictionMaxPositionChange = 512;
      ffbParams[0].frictionPositionChange    = xHID - 511;
      ffbParams[1].springMaxPosition = 512;
      ffbParams[1].springPosition    = yHID - 511;
      ffbParams[1].frictionMaxPositionChange = 512;
      ffbParams[1].frictionPositionChange    = yHID - 511;
      Joystick.setEffectParams(ffbParams);

      Joystick.getUSBPID();
      nbFixEffects();
      nbFindSpring();
      nbGains[0].springGain = crvSpringGain(0, xHID);   // read by getForce() through setGains()
      nbGains[1].springGain = crvSpringGain(1, yHID);
      Joystick.getForce(forces);
      safetyCheck(xHID, yHID, dtMs);

      int16_t fx = (int16_t)forces[0];
      int16_t fy = (int16_t)forces[1];

      // Velocity damping (library frame: subtracting velocity always opposes the motion),
      // only while the library outputs a force.
      if ((fx != 0 || fy != 0) && cfg.velDamp) {
        int16_t dx = (int16_t)(((int32_t)velX * cfg.velDamp) / 1000L);
        int16_t dy = (int16_t)(((int32_t)velY * cfg.velDamp) / 1000L);
        fx = constrain(fx - constrain(dx, -VD_LIMIT, VD_LIMIT), -255, 255);
        fy = constrain(fy - constrain(dy, -VD_LIMIT, VD_LIMIT), -255, 255);
      }

      // SLEW: how fast the force may change (units per ms), also when it drops to zero.
      static int16_t lastFx = 0, lastFy = 0;
      if (cfg.slew) {
        int16_t step = (int16_t)cfg.slew * dtMs;
        fx = constrain(fx, lastFx - step, lastFx + step);
        fy = constrain(fy, lastFy - step, lastFy + step);
      }
      lastFx = fx;
      lastFy = fy;

      // sim frame -> motor: the inversion for the sim and the measured motor direction
      if (cfg.invMX ^ cfg.uinvX) fx = -fx;
      if (cfg.invMY ^ cfg.uinvY) fy = -fy;

      static uint8_t onX = 0, onY = 0;          // MINE hysteresis state per axis
      int16_t outX = nbDriveCounts(fx, 0, &onX);
      int16_t outY = nbDriveCounts(fy, 1, &onY);
      if (nbMotorOn) {
        nbSetMotor(0, outX);
        nbSetMotor(1, outY);
      }

#if NB_TRACE
      static uint32_t lastTraceMs = 0;
      if (traceOn && (uint32_t)(nowMs - lastTraceMs) >= 10) {
        lastTraceMs = nowMs;
        traceLine(nowMs, xHID, yHID, outX, outY);
      }
#endif
    }
  }

  // === 3. Grip axes, buttons, ATB modes; pending curve bytes to the EEPROM ===
  nbHidInputs(nowMs);
  crvFlushStep();

  // === 4. HID report every HID_SEND_MS ===
  static uint32_t lastSendMs = 0;
  if ((uint32_t)(nowMs - lastSendMs) >= HID_SEND_MS) {
    lastSendMs = nowMs;
    Joystick.sendState();
  }
}
