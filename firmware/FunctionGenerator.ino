/*
 * ============================================================================
 *  BEE 425 Design Project - Function Generator firmware
 *  Target : Arduino Nano (ATmega328P, 16 MHz)   -- board in KiCad project
 *           "Function generator - Arduino"
 * ============================================================================
 *
 *
 *  Pin map (from the schematic nets):
 *     D2  rotary encoder A (INT0)        D9  AD9833 FSYNC
 *     D3  rotary encoder B (INT1)        D10 MCP41010 /CS
 *     D4  encoder push switch            D11 MOSI  (AD9833 SDATA + pot SI)
 *     D5  DC-offset PWM (Timer0 OC0B)    D13 SCK   (shared)
 *     D6  SW3  Frequency/Amp/Offset      A0  SW2  Waveform
 *     A4  SDA / A5 SCL  -> OLED header J6
 *
 *  User interface
 *     Knob turn ........ change the selected field (frequency / amplitude /
 *                        offset) by the current step
 *     Knob short press . cycle the step size of the selected field
 *     Knob long press .. output ON / OFF
 *     SW3 (D6) ......... select field: FREQUENCY -> AMPLITUDE -> OFFSET
 *     SW2 (A0) ......... select waveform: SINE -> TRIANGLE -> SQUARE
 *     Hold the knob while powering up to restore factory defaults.
 *
 *  Serial console (115200 baud, newline terminated) for bring-up / test plan:
 *     f<Hz>   frequency, e.g. f1000        w<0|1|2>  0=sine 1=tri 2=square
 *     a<mV>   peak amplitude, e.g. a2500   o<mV>     DC offset, e.g. o-1000
 *     e<0|1>  output off/on                ?         print status
 *
 *  Libraries: none to install. SPI, Wire and EEPROM come with the Arduino AVR
 *  core. The OLED (SSD1306, I2C 0x3C) is driven by a small page-mode driver
 *  in this file, so no 1 KB frame buffer is needed on the 2 KB ATmega328P.
 *
 *  Sources / citations (course requires them):
 *   - AD9833 register map and write sequence: Analog Devices AD9833 datasheet
 *     (control register bits B28, RESET, OPBITEN, DIV2, MODE; FREQ0 = 0x4000).
 *   - MCP41010 command format (0x11 = write wheel 0): Microchip DS11195.
 *   - SSD1306 power-up command sequence: SSD1306 datasheet / the same values
 *     used by the Adafruit_SSD1306 library (BSD license).
 *   - 5x7 font table (ASCII 32..126): the classic "glcdfont" from the
 *     Adafruit_GFX library (BSD license).
 *   - Quadrature transition-table decoding is a standard technique; the table
 *     and ISR below were written for this project.
 *   Everything else in this file is original project code.
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#if !defined(__AVR_ATmega328P__)
#error "This sketch is written for the Arduino Nano (ATmega328P): the encoder ISR reads PIND directly."
#endif

// ============================================================================
//  1. PIN MAP
// ============================================================================
const uint8_t PIN_ENC_A      = 2;     // D2  encoder A  (external 10k pull-up R1)
const uint8_t PIN_ENC_B      = 3;     // D3  encoder B  (external 10k pull-up R2)
const uint8_t PIN_ENC_SW     = 4;     // D4  encoder switch (external 10k pull-up R3)
const uint8_t PIN_OFFSET_PWM = 5;     // D5  DC-offset PWM (OC0B)
const uint8_t PIN_BTN_MODE   = 6;     // D6  SW3 (no external pull-up -> INPUT_PULLUP)
const uint8_t PIN_DDS_FSYNC  = 9;     // D9  AD9833 FSYNC
const uint8_t PIN_POT_CS     = 10;    // D10 MCP41010 /CS (also the AVR SS pin: keep OUTPUT)
const uint8_t PIN_BTN_WAVE   = A0;    // A0  SW2 (no external pull-up -> INPUT_PULLUP)

// ============================================================================
//  2. CONFIGURATION AND CALIBRATION
//     Values marked CALIBRATE should be trimmed against a scope/DMM.
// ============================================================================

// ---- Output range --------------------------------------------------------
const uint32_t FREQ_MIN_HZ  = 10;
const uint32_t FREQ_MAX_HZ  = 100000UL;
const int16_t  AMP_MAX_MV   = 5000;   // peak amplitude limit (spec: +/-5 V)
const int16_t  OFFS_MAX_MV  = 4900;   // DC offset limit (+/-); the circuit reaches about +4.97 / -4.94 V
const int16_t  OUT_LIMIT_MV = 7500;   // |offset| + amplitude must stay below the +/-9 V rails

const bool OUTPUT_ON_AT_BOOT = false; // safer: knob long-press turns the output on

// ---- AD9833 ----------------------------------------------------------------
const uint32_t DDS_MCLK_HZ = 25000000UL;  // CALIBRATE: module crystal (usually 25 MHz)

// ---- Amplitude path model (component values from the schematic) -----------
const float R_FB        = 220000.0f;  // R7  feedback resistor
const float R_IN        = 10000.0f;   // R5  op-amp input resistor
const float R_SRC       = 1200.0f;    // R4 (1k) + AD9833 output impedance (~200 R)
const float R_POT       = 10000.0f;   // MCP41010 end-to-end resistance
const float R_WIPER     = 52.0f;      // MCP41010 wiper resistance (typ., 5 V)
// Peak-to-peak voltage at the AD9833 VOUT pin:
const float SRC_VPP_SINE_TRI = 0.60f; // CALIBRATE: DAC output (datasheet ~0.6 Vpp)
const float SRC_VPP_SQUARE   = 4.80f; // CALIBRATE: MSB output is a logic level (~VDD); measure it!
// Overall gain trims: set to (displayed amplitude / measured amplitude) after
// measuring an open-circuit (Hi-Z) output at about 2 V peak.
const float GAIN_TRIM_SINE_TRI = 1.00f;
const float GAIN_TRIM_SQUARE   = 1.00f;
// Op-amp closed-loop bandwidth (OPA197: GBW 10 MHz / gain 22 ~ 450 kHz).
// Used to boost sine/triangle amplitude at high frequency. 0 disables it.
const float OPAMP_BW_HZ = 400000.0f;

// ---- DC-offset path model ---------------------------------------------------
const float VCC_V       = 5.00f;      // CALIBRATE: measure the +5 V rail
const float R_OFFS_BIAS = 20000.0f;   // R9  (+5 V -> op-amp + input)
const float R_OFFS_GND  = 10000.0f;   // R12 (op-amp + input -> GND)
const float R_OFFS_PATH = 1000.0f + 10000.0f + 100000.0f;  // R11 + R8 + R6
const int16_t OFFSET_TRIM_MV = 0;     // CALIBRATE: added to every offset so 0 V reads 0 V

// ---- User interface timing -------------------------------------------------
const uint8_t  ENC_COUNTS_PER_DETENT = 4;   // EC11 full-cycle detent = 4 transitions (use 2 for half-cycle types)
const bool     ENC_REVERSE           = false; // flip if the knob turns the wrong way
const uint16_t ENC_RESYNC_MS         = 400;   // drop a half-finished detent after this much quiet time
const uint16_t DEBOUNCE_MS   = 25;
const uint16_t LONG_PRESS_MS = 700;
const uint16_t SAVE_DELAY_MS = 3000;        // write EEPROM this long after the last change
const uint8_t  OLED_ADDR     = 0x3C;
const uint8_t  OLED_COLUMN_OFFSET = 0;     // set to 2 for SH1106-based 1.3" modules
const uint16_t DISPLAY_MIN_INTERVAL_MS = 40;

#define DEBUG_SERIAL 1

// ============================================================================
//  3. TYPES, TABLES, GLOBAL STATE
// ============================================================================
const uint8_t WAVE_SINE = 0, WAVE_TRIANGLE = 1, WAVE_SQUARE = 2, WAVE_COUNT = 3;
const uint8_t FIELD_FREQ = 0, FIELD_AMP = 1, FIELD_OFFSET = 2, FIELD_COUNT = 3;

const uint8_t EV_NONE = 0, EV_SHORT = 1, EV_LONG = 2;

// Step sizes (PROGMEM to save RAM)
const uint32_t FREQ_STEPS[] PROGMEM = { 1, 10, 100, 1000, 10000 };
const uint8_t  FREQ_STEP_COUNT = 5;
const int16_t  MV_STEPS[] PROGMEM = { 10, 100, 1000 };
const uint8_t  MV_STEP_COUNT = 3;

struct Settings {
  uint32_t freqHz;
  int16_t  ampMv;        // peak amplitude, millivolts
  int16_t  offsMv;       // DC offset, millivolts
  uint8_t  wave;
  uint8_t  freqStepIdx;
  uint8_t  ampStepIdx;
  uint8_t  offStepIdx;
};

struct Button {
  uint8_t  pin;
  bool     raw;          // last raw reading (true = pressed)
  bool     down;         // debounced state
  bool     longDone;
  uint32_t tEdge;
  uint32_t tDown;
};

Settings cfg;
bool     outputOn   = false;
uint8_t  field      = FIELD_FREQ;

Button   buttons[3] = {
  { PIN_BTN_MODE, false, false, false, 0, 0 },   // [0] SW3
  { PIN_BTN_WAVE, false, false, false, 0, 0 },   // [1] SW2
  { PIN_ENC_SW,   false, false, false, 0, 0 }    // [2] encoder push
};

bool     displayOk    = false;
bool     displayDirty = true;
uint32_t tLastDraw    = 0;

bool     savePending  = false;
uint32_t tLastChange  = 0;

bool     ddsMuted     = true;     // AD9833 held in RESET (mid-scale, AC-coupled = 0 V)
uint8_t  lastPotCode  = 0;

// ============================================================================
//  4. SMALL HELPERS
// ============================================================================
static long clampL(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int16_t maxAmpMv() {
  int16_t room = OUT_LIMIT_MV - (int16_t)abs(cfg.offsMv);
  return room < AMP_MAX_MV ? room : AMP_MAX_MV;
}
static int16_t maxOffsMv() {
  int16_t room = OUT_LIMIT_MV - cfg.ampMv;
  return room < OFFS_MAX_MV ? room : OFFS_MAX_MV;
}

static void setDefaults() {
  cfg.freqHz      = 1000;
  cfg.ampMv       = 1000;
  cfg.offsMv      = 0;
  cfg.wave        = WAVE_SINE;
  cfg.freqStepIdx = 2;     // 100 Hz
  cfg.ampStepIdx  = 1;     // 100 mV
  cfg.offStepIdx  = 1;     // 100 mV
}

// Bring any (possibly corrupt) settings back into the legal range.
static void sanitize() {
  cfg.freqHz = (uint32_t)clampL((long)cfg.freqHz, FREQ_MIN_HZ, FREQ_MAX_HZ);
  if (cfg.wave >= WAVE_COUNT) cfg.wave = WAVE_SINE;
  if (cfg.freqStepIdx >= FREQ_STEP_COUNT) cfg.freqStepIdx = 2;
  if (cfg.ampStepIdx  >= MV_STEP_COUNT)   cfg.ampStepIdx  = 1;
  if (cfg.offStepIdx  >= MV_STEP_COUNT)   cfg.offStepIdx  = 1;
  cfg.offsMv = (int16_t)clampL(cfg.offsMv, -OFFS_MAX_MV, OFFS_MAX_MV);
  cfg.ampMv  = (int16_t)clampL(cfg.ampMv, 0, AMP_MAX_MV);
  cfg.offsMv = (cfg.offsMv / 10) * 10;
  cfg.ampMv  = (cfg.ampMv / 10) * 10;
  if (cfg.ampMv + abs(cfg.offsMv) > OUT_LIMIT_MV) cfg.ampMv = OUT_LIMIT_MV - abs(cfg.offsMv);
}

static void markChanged() {
  displayDirty = true;
  savePending  = true;
  tLastChange  = millis();
}

// ============================================================================
//  5. EEPROM (settings survive power cycles)
// ============================================================================
const uint8_t EE_MAGIC = 0xB7;

static uint8_t settingsChecksum(const Settings &s) {
  const uint8_t *p = (const uint8_t *)&s;
  uint8_t c = EE_MAGIC;
  for (uint8_t i = 0; i < sizeof(Settings); i++) c = (uint8_t)((c << 1) | (c >> 7)) ^ p[i];
  return c;
}

static bool loadSettings() {
  uint8_t magic = EEPROM.read(0);
  if (magic != EE_MAGIC) return false;
  Settings s;
  EEPROM.get(1, s);
  if (EEPROM.read(1 + sizeof(Settings)) != settingsChecksum(s)) return false;
  cfg = s;
  return true;
}

static void saveSettings() {
  EEPROM.update(0, EE_MAGIC);
  EEPROM.put(1, cfg);                                   // put() only writes changed bytes
  EEPROM.update(1 + sizeof(Settings), settingsChecksum(cfg));
}

// ============================================================================
//  6. SPI DEVICES: AD9833 DDS and MCP41010 digital potentiometer
//     Both idle SCK high (CPOL=1). The AD9833 needs SPI mode 2 (data latched
//     on the falling edge); the MCP41010 supports modes 0,0 and 1,1, so it
//     is used in mode 3. Same idle level => no stray clock edge when the
//     sketch switches between the two devices.
// ============================================================================
const uint16_t AD_CTRL_B28   = 0x2000;   // two-write 28-bit frequency load
const uint16_t AD_CTRL_RESET = 0x0100;   // output to mid-scale, phase acc. cleared
const uint16_t AD_CTRL_OPBIT = 0x0020;   // route DAC MSB to VOUT (square wave)
const uint16_t AD_CTRL_DIV2  = 0x0008;   // with OPBITEN: 1 = MSB at full rate
const uint16_t AD_CTRL_MODE  = 0x0002;   // 1 = triangle (SIN ROM bypassed)
const uint16_t AD_FREQ0      = 0x4000;   // FREQ0 register address bits
const uint16_t AD_PHASE0     = 0xC000;   // PHASE0 register = 0

static void ad9833Write(uint16_t word) {
  SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE2));
  digitalWrite(PIN_DDS_FSYNC, LOW);
  SPI.transfer16(word);
  digitalWrite(PIN_DDS_FSYNC, HIGH);
  SPI.endTransaction();
}

static uint16_t ddsWaveBits(uint8_t w) {
  switch (w) {
    case WAVE_TRIANGLE: return AD_CTRL_MODE;
    case WAVE_SQUARE:   return AD_CTRL_OPBIT | AD_CTRL_DIV2;
    default:            return 0;                          // sine
  }
}

// Write the control register for the current waveform and mute state.
static void ddsApplyControl() {
  ad9833Write(AD_CTRL_B28 | ddsWaveBits(cfg.wave) | (ddsMuted ? AD_CTRL_RESET : 0));
}

static void ddsSetFrequency(uint32_t hz) {
  // FTW = f * 2^28 / fMCLK (rounded), 0.093 Hz resolution at 25 MHz
  uint32_t ftw = (uint32_t)((((uint64_t)hz << 28) + (DDS_MCLK_HZ / 2)) / DDS_MCLK_HZ);
  ad9833Write(AD_FREQ0 | (uint16_t)(ftw & 0x3FFF));          // 14 LSBs
  ad9833Write(AD_FREQ0 | (uint16_t)((ftw >> 14) & 0x3FFF));  // 14 MSBs
}

static void ddsSetMuted(bool m) {
  if (m != ddsMuted) {
    ddsMuted = m;
    ddsApplyControl();
  }
}

static void ddsInit() {
  ddsMuted = true;
  ad9833Write(AD_CTRL_B28 | AD_CTRL_RESET);
  ddsSetFrequency(cfg.freqHz);
  ad9833Write(AD_PHASE0);
  ddsApplyControl();                   // wave selected, still in reset
}

// MCP41010: 16-bit command, 0x11 = "write data to pot 0", then 8-bit wiper code.
static void potWrite(uint8_t code) {
  lastPotCode = code;
  SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_POT_CS, LOW);
  SPI.transfer(0x11);
  SPI.transfer(code);
  digitalWrite(PIN_POT_CS, HIGH);
  SPI.endTransaction();
}

// ============================================================================
//  7. ANALOG MODEL: amplitude -> pot code, offset -> PWM duty
// ============================================================================

// Overall gain (Vout_pp / Vsrc_pp) of pot + op-amp stage at wiper code n.
// Source -> R_SRC -> pot end A; wiper loaded by R5 into the virtual ground.
static float stageGain(uint8_t n) {
  float rwb = R_POT * n / 256.0f + R_WIPER;
  float rwa = R_POT * (256 - n) / 256.0f + R_WIPER;
  float rl  = (rwb * R_IN) / (rwb + R_IN);
  return (R_FB / R_IN) * rl / (R_SRC + rwa + rl);
}

static float freqBoost(uint32_t f, uint8_t wave) {
  if (wave == WAVE_SQUARE || OPAMP_BW_HZ <= 0.0f) return 1.0f;
  float r = (float)f / OPAMP_BW_HZ;
  return sqrtf(1.0f + r * r);
}

static float sourceVpp(uint8_t wave)  { return wave == WAVE_SQUARE ? SRC_VPP_SQUARE   : SRC_VPP_SINE_TRI; }
static float gainTrim(uint8_t wave)   { return wave == WAVE_SQUARE ? GAIN_TRIM_SQUARE : GAIN_TRIM_SINE_TRI; }

// Smallest-error wiper code (1..255) for the requested peak amplitude.
static uint8_t potCodeFor(int16_t ampMv, uint32_t f, uint8_t wave) {
  float vppWanted = 2.0f * ampMv / 1000.0f;
  float gainNeed  = vppWanted * freqBoost(f, wave) / (gainTrim(wave) * sourceVpp(wave));
  if (gainNeed >= stageGain(255)) return 255;
  uint8_t lo = 1, hi = 255;                       // stageGain() rises monotonically with n
  while (lo < hi) {
    uint8_t mid = (uint8_t)((lo + hi) / 2);
    if (stageGain(mid) < gainNeed) lo = mid + 1; else hi = mid;
  }
  if (lo > 1 && (gainNeed - stageGain(lo - 1)) < (stageGain(lo) - gainNeed)) lo--;
  return lo;
}

// Open-circuit peak amplitude (mV) the model predicts for a pot code.
static int16_t predictedPeakMv(uint8_t code) {
  float vpp = sourceVpp(cfg.wave) * stageGain(code) * gainTrim(cfg.wave) / freqBoost(cfg.freqHz, cfg.wave);
  return (int16_t)(vpp * 500.0f + 0.5f);
}

// PWM duty (0..255) for the requested DC offset.
//   Vout = Vcc * (K0 - K1*duty)   with  K1 = R7/(R11+R8+R6),
//   K0 = (R12/(R9+R12)) * (1 + K1)
static uint8_t offsetDuty(int16_t offsMv) {
  float k1 = R_FB / R_OFFS_PATH;
  float k0 = (R_OFFS_GND / (R_OFFS_BIAS + R_OFFS_GND)) * (1.0f + k1);
  float v  = (offsMv + OFFSET_TRIM_MV) / 1000.0f;
  float d  = (k0 - v / VCC_V) / k1;
  long  c  = lroundf(d * 255.0f);
  return (uint8_t)clampL(c, 0, 255);
}

// ============================================================================
//  8. APPLYING SETTINGS TO THE HARDWARE
// ============================================================================
static void updateFrequency() {
  ddsSetFrequency(cfg.freqHz);
  // sine/triangle amplitude depends slightly on frequency (op-amp roll-off)
  if (outputOn && cfg.ampMv > 0 && cfg.wave != WAVE_SQUARE && OPAMP_BW_HZ > 0.0f)
    potWrite(potCodeFor(cfg.ampMv, cfg.freqHz, cfg.wave));
}

static void updateAmplitude() {
  if (!outputOn || cfg.ampMv == 0) {
    ddsSetMuted(true);                 // exact 0 V AC: DDS in reset, pot at 0
    potWrite(0);
  } else {
    potWrite(potCodeFor(cfg.ampMv, cfg.freqHz, cfg.wave));   // set gain first ...
    ddsSetMuted(false);                                      // ... then un-mute
  }
}

static void updateOffset() {
  analogWrite(PIN_OFFSET_PWM, offsetDuty(outputOn ? cfg.offsMv : 0));
}

// Waveform change: the square wave is a ~5 V logic level, sine/triangle only
// 0.6 V, so drop the pot to 0 before switching to avoid an output spike.
static void updateWaveform() {
  potWrite(0);
  ddsApplyControl();
  updateAmplitude();
}

static void updateOutputState() {
  if (outputOn) { updateOffset(); updateAmplitude(); }
  else          { updateAmplitude(); updateOffset(); }
}

// ============================================================================
//  9. ROTARY ENCODER (interrupt driven, bounce-tolerant)
// ============================================================================
// Index = (previous state << 2) | new state, state = (B << 1) | A.
// Valid Gray-code steps give +/-1, bounce/illegal jumps give 0.
static const int8_t ENC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

volatile uint8_t encPrev  = 0;
volatile int8_t  encAccum = 0;
volatile int8_t  encDelta = 0;
volatile uint32_t encLastMs = 0;

static void encoderISR() {
  uint8_t cur = (PIND >> 2) & 0x03;               // D2 -> bit0 (A), D3 -> bit1 (B)
  int8_t  step = ENC_TABLE[(encPrev << 2) | cur];
  encPrev = cur;
  encLastMs = millis();
  if (ENC_REVERSE) step = -step;
  encAccum += step;
  if (encAccum >= (int8_t)ENC_COUNTS_PER_DETENT)       { encDelta++; encAccum = 0; }
  else if (encAccum <= -(int8_t)ENC_COUNTS_PER_DETENT) { encDelta--; encAccum = 0; }
}

static int8_t takeEncoderDelta() {
  noInterrupts();
  int8_t d = encDelta;
  encDelta = 0;
  // Contact bounce can leave a partial count behind; forget it after a pause
  // so it cannot swallow or add a step to the next real detent.
  if (encAccum != 0 && (millis() - encLastMs) > ENC_RESYNC_MS) encAccum = 0;
  interrupts();
  return d;
}

// ============================================================================
// 10. BUTTONS (debounced; optional long press)
// ============================================================================
static uint8_t pollButton(uint8_t idx, bool wantLong) {
  Button &b = buttons[idx];
  bool r = (digitalRead(b.pin) == LOW);            // active low
  uint32_t now = millis();
  if (r != b.raw) { b.raw = r; b.tEdge = now; }
  if (r != b.down && (now - b.tEdge) >= DEBOUNCE_MS) {
    b.down = r;
    if (r) {
      b.tDown = now;
      b.longDone = false;
      if (!wantLong) return EV_SHORT;              // act on press when no long-press needed
    } else if (wantLong && !b.longDone) {
      return EV_SHORT;                             // released before the long-press time
    }
  }
  if (wantLong && b.down && !b.longDone && (now - b.tDown) >= LONG_PRESS_MS) {
    b.longDone = true;
    return EV_LONG;
  }
  return EV_NONE;
}

// ============================================================================
// 11. USER ACTIONS
// ============================================================================
#if DEBUG_SERIAL
static int freeRam() {                    // gap between heap and stack (bytes)
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
#endif

static void printStatus() {
#if DEBUG_SERIAL
  Serial.print(F("f="));      Serial.print(cfg.freqHz);
  Serial.print(F(" Hz  "));
  Serial.print(cfg.wave == WAVE_SINE ? F("sine") : (cfg.wave == WAVE_TRIANGLE ? F("triangle") : F("square")));
  Serial.print(F("  amp="));  Serial.print(cfg.ampMv);
  Serial.print(F(" mV pk  off=")); Serial.print(cfg.offsMv);
  Serial.print(F(" mV  out=")); Serial.print(outputOn ? F("ON") : F("OFF"));
  Serial.print(F("  pot="));  Serial.print(lastPotCode);
  if (ddsMuted) Serial.print(F("  muted"));
  else { Serial.print(F("  predicted pk=")); Serial.print(predictedPeakMv(lastPotCode)); Serial.print(F(" mV")); }
  Serial.print(F("  free RAM="));
  Serial.println(freeRam());
#endif
}

static void adjustSelected(int8_t detents) {
  switch (field) {
    case FIELD_FREQ: {
      long step = (long)pgm_read_dword(&FREQ_STEPS[cfg.freqStepIdx]);
      long f = clampL((long)cfg.freqHz + detents * step, FREQ_MIN_HZ, FREQ_MAX_HZ);
      if ((uint32_t)f != cfg.freqHz) { cfg.freqHz = (uint32_t)f; updateFrequency(); markChanged(); }
      break;
    }
    case FIELD_AMP: {
      int16_t step = (int16_t)pgm_read_word(&MV_STEPS[cfg.ampStepIdx]);
      int16_t a = (int16_t)clampL((long)cfg.ampMv + (long)detents * step, 0, maxAmpMv());
      if (a != cfg.ampMv) { cfg.ampMv = a; updateAmplitude(); markChanged(); }
      break;
    }
    case FIELD_OFFSET: {
      int16_t step = (int16_t)pgm_read_word(&MV_STEPS[cfg.offStepIdx]);
      int16_t lim = maxOffsMv();
      int16_t o = (int16_t)clampL((long)cfg.offsMv + (long)detents * step, -lim, lim);
      if (o != cfg.offsMv) { cfg.offsMv = o; updateOffset(); markChanged(); }
      break;
    }
  }
}

static void cycleStep() {
  switch (field) {
    case FIELD_FREQ:   cfg.freqStepIdx = (cfg.freqStepIdx + 1) % FREQ_STEP_COUNT; break;
    case FIELD_AMP:    cfg.ampStepIdx  = (cfg.ampStepIdx  + 1) % MV_STEP_COUNT;   break;
    case FIELD_OFFSET: cfg.offStepIdx  = (cfg.offStepIdx  + 1) % MV_STEP_COUNT;   break;
  }
  markChanged();
}

static void nextWaveform() {
  cfg.wave = (cfg.wave + 1) % WAVE_COUNT;
  updateWaveform();
  markChanged();
  printStatus();
}

static void toggleOutput() {
  outputOn = !outputOn;
  updateOutputState();
  displayDirty = true;
  printStatus();
}

static void handleInputs() {
  int8_t d = takeEncoderDelta();
  if (d) { adjustSelected(d); printStatus(); }

  if (pollButton(0, false) == EV_SHORT) { field = (field + 1) % FIELD_COUNT; displayDirty = true; }
  if (pollButton(1, false) == EV_SHORT) { nextWaveform(); }

  uint8_t ev = pollButton(2, true);
  if (ev == EV_SHORT)      cycleStep();
  else if (ev == EV_LONG)  toggleOutput();
}

// ============================================================================
// 12. SERIAL CONSOLE
// ============================================================================
static void runCommand(char *s) {
  char c = s[0];
  if (c >= 'A' && c <= 'Z') c += 32;
  long v = atol(s + 1);
  switch (c) {
    case 'f': cfg.freqHz = (uint32_t)clampL(v, FREQ_MIN_HZ, FREQ_MAX_HZ); updateFrequency(); markChanged(); break;
    case 'w': if (v >= 0 && v < WAVE_COUNT) { cfg.wave = (uint8_t)v; updateWaveform(); markChanged(); } break;
    case 'a': cfg.ampMv  = (int16_t)clampL(v, 0, maxAmpMv());              updateAmplitude(); markChanged(); break;
    case 'o': cfg.offsMv = (int16_t)clampL(v, -maxOffsMv(), maxOffsMv());  updateOffset();    markChanged(); break;
    case 'e': outputOn = (v != 0); updateOutputState(); displayDirty = true; break;
    default: break;                                    // '?' just prints status
  }
  printStatus();
}

static void handleSerial() {
  static char line[16];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n) { line[n] = 0; runCommand(line); n = 0; }
    } else if (n < sizeof(line) - 1) {
      line[n++] = ch;
    }
  }
}

// ============================================================================
// 13. OLED (SSD1306 128x64, I2C) - tiny page-mode driver, no frame buffer
//     The screen is 8 pages of 128 columns (1 byte = 8 vertical pixels).
//     Each page is composed in one 128-byte line buffer and sent on its own,
//     so the whole UI needs 128 bytes of RAM instead of a 1 KB frame buffer.
//     UI rows are aligned to pages:
//       page 0 header | 2-3 frequency (2x) | 5 amplitude | 6 offset | 7 footer
// ============================================================================

// 5x7 font, ASCII 32..126, 5 columns per glyph, bit0 = top row.
// Source: Adafruit_GFX "glcdfont" (BSD license).
static const uint8_t FONT5X7[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x5F, 0x00, 0x00,
  0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14,
  0x24, 0x2A, 0x7F, 0x2A, 0x12,
  0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x56, 0x20, 0x50,
  0x00, 0x08, 0x07, 0x03, 0x00,
  0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00,
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
  0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x80, 0x70, 0x30, 0x00,
  0x08, 0x08, 0x08, 0x08, 0x08,
  0x00, 0x00, 0x60, 0x60, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02,
  0x3E, 0x51, 0x49, 0x45, 0x3E,
  0x00, 0x42, 0x7F, 0x40, 0x00,
  0x72, 0x49, 0x49, 0x49, 0x46,
  0x21, 0x41, 0x49, 0x4D, 0x33,
  0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39,
  0x3C, 0x4A, 0x49, 0x49, 0x31,
  0x41, 0x21, 0x11, 0x09, 0x07,
  0x36, 0x49, 0x49, 0x49, 0x36,
  0x46, 0x49, 0x49, 0x29, 0x1E,
  0x00, 0x00, 0x14, 0x00, 0x00,
  0x00, 0x40, 0x34, 0x00, 0x00,
  0x00, 0x08, 0x14, 0x22, 0x41,
  0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08,
  0x02, 0x01, 0x59, 0x09, 0x06,
  0x3E, 0x41, 0x5D, 0x59, 0x4E,
  0x7C, 0x12, 0x11, 0x12, 0x7C,
  0x7F, 0x49, 0x49, 0x49, 0x36,
  0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x49, 0x49, 0x49, 0x41,
  0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x41, 0x51, 0x73,
  0x7F, 0x08, 0x08, 0x08, 0x7F,
  0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01,
  0x7F, 0x08, 0x14, 0x22, 0x41,
  0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x1C, 0x02, 0x7F,
  0x7F, 0x04, 0x08, 0x10, 0x7F,
  0x3E, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x09, 0x09, 0x09, 0x06,
  0x3E, 0x41, 0x51, 0x21, 0x5E,
  0x7F, 0x09, 0x19, 0x29, 0x46,
  0x26, 0x49, 0x49, 0x49, 0x32,
  0x03, 0x01, 0x7F, 0x01, 0x03,
  0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F,
  0x3F, 0x40, 0x38, 0x40, 0x3F,
  0x63, 0x14, 0x08, 0x14, 0x63,
  0x03, 0x04, 0x78, 0x04, 0x03,
  0x61, 0x59, 0x49, 0x4D, 0x43,
  0x00, 0x7F, 0x41, 0x41, 0x41,
  0x02, 0x04, 0x08, 0x10, 0x20,
  0x00, 0x41, 0x41, 0x41, 0x7F,
  0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40,
  0x00, 0x03, 0x07, 0x08, 0x00,
  0x20, 0x54, 0x54, 0x78, 0x40,
  0x7F, 0x28, 0x44, 0x44, 0x38,
  0x38, 0x44, 0x44, 0x44, 0x28,
  0x38, 0x44, 0x44, 0x28, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18,
  0x00, 0x08, 0x7E, 0x09, 0x02,
  0x18, 0xA4, 0xA4, 0x9C, 0x78,
  0x7F, 0x08, 0x04, 0x04, 0x78,
  0x00, 0x44, 0x7D, 0x40, 0x00,
  0x20, 0x40, 0x40, 0x3D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00,
  0x00, 0x41, 0x7F, 0x40, 0x00,
  0x7C, 0x04, 0x78, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78,
  0x38, 0x44, 0x44, 0x44, 0x38,
  0xFC, 0x18, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC,
  0x7C, 0x08, 0x04, 0x04, 0x08,
  0x48, 0x54, 0x54, 0x54, 0x24,
  0x04, 0x04, 0x3F, 0x44, 0x24,
  0x3C, 0x40, 0x40, 0x20, 0x7C,
  0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C,
  0x44, 0x28, 0x10, 0x28, 0x44,
  0x4C, 0x90, 0x90, 0x90, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44,
  0x00, 0x08, 0x36, 0x41, 0x00,
  0x00, 0x00, 0x77, 0x00, 0x00,
  0x00, 0x41, 0x36, 0x08, 0x00,
  0x02, 0x01, 0x02, 0x04, 0x02
};

static const uint8_t OLED_INIT[] PROGMEM = {
  0xAE,              // display off
  0xD5, 0x80,        // clock divide / oscillator
  0xA8, 0x3F,        // multiplex 1/64
  0xD3, 0x00,        // display offset 0
  0x40,              // start line 0
  0x8D, 0x14,        // charge pump on (internal VCC)
  0xA1,              // segment remap (column 127 -> SEG0)
  0xC8,              // COM scan direction reversed (upright image)
  0xDA, 0x12,        // COM pins config for 128x64
  0x81, 0xCF,        // contrast
  0xD9, 0xF1,        // pre-charge
  0xDB, 0x40,        // VCOMH level
  0xA4,              // output follows RAM
  0xA6,              // normal (not inverted)
  0x2E,              // scrolling off
  0xAF               // display on
};

static bool oledCommands(const uint8_t *cmds, uint8_t n) {       // cmds in RAM
  Wire.beginTransmission(OLED_ADDR);
  Wire.write((uint8_t)0x00);                      // control byte: commands
  Wire.write(cmds, n);
  return Wire.endTransmission() == 0;
}

static bool oledBegin() {
  Wire.begin();
  Wire.setClock(400000UL);
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) return false;  // nobody answered at this address
  uint8_t buf[sizeof(OLED_INIT)];
  memcpy_P(buf, OLED_INIT, sizeof(OLED_INIT));
  return oledCommands(buf, sizeof(buf));
}

static void oledWritePage(uint8_t page, const uint8_t *line) {
  uint8_t c[3] = { (uint8_t)(0xB0 | page),
                   (uint8_t)(OLED_COLUMN_OFFSET & 0x0F),
                   (uint8_t)(0x10 | (OLED_COLUMN_OFFSET >> 4)) };
  oledCommands(c, 3);
  for (uint8_t i = 0; i < 128; i += 16) {         // Wire buffer is 32 bytes: send 16 at a time
    Wire.beginTransmission(OLED_ADDR);
    Wire.write((uint8_t)0x40);                    // control byte: display data
    Wire.write(line + i, 16);
    Wire.endTransmission();
  }
}

// Double every bit of a 7-row glyph column (7 bits -> 14 bits).
static uint16_t spreadBits(uint8_t b) {
  uint16_t r = 0;
  for (uint8_t i = 0; i < 7; i++) if (b & (1 << i)) r |= (uint16_t)3 << (2 * i);
  return r;
}

// Draw text into the page buffer. scale 1: one page tall. scale 2: two pages
// tall - call once with half=0 (upper page) and once with half=1 (lower page).
// Glyphs are shifted down one pixel so inverted bars get a top margin.
static uint8_t drawText(uint8_t *line, uint8_t x, const char *s, uint8_t scale, uint8_t half) {
  for (; *s; s++) {
    char ch = *s;
    if (ch < 32 || ch > 126) ch = '?';
    const uint8_t *g = FONT5X7 + (uint8_t)(ch - 32) * 5;
    for (uint8_t col = 0; col < 6; col++) {       // 5 glyph columns + 1 spacing column
      uint8_t bits = (col < 5) ? pgm_read_byte(g + col) : 0;
      uint8_t out;
      if (scale == 1) out = (uint8_t)(bits << 1);
      else            out = (uint8_t)((spreadBits(bits) << 1) >> (half ? 8 : 0));
      for (uint8_t r = 0; r < scale; r++) { if (x < 128) line[x] = out; x++; }
    }
  }
  return x;
}

static uint8_t textWidth(const char *s, uint8_t scale) { return (uint8_t)(strlen(s) * 6 * scale); }

// Draw a text constant stored in flash.
static void putP(uint8_t *line, uint8_t x, const char *pstr) {
  char t[24];
  strncpy_P(t, pstr, sizeof(t) - 1);
  t[sizeof(t) - 1] = 0;
  drawText(line, x, t, 1, 0);
}

static void fmtFreq(char *buf, uint8_t n, uint32_t f) {
  if (f < 1000UL) snprintf_P(buf, n, PSTR("%lu Hz"), f);
  else            snprintf_P(buf, n, PSTR("%lu.%03lukHz"), f / 1000UL, f % 1000UL);
}

static void fmtMv(char *buf, uint8_t n, int16_t mv, bool withSign) {
  uint16_t a = (uint16_t)abs(mv);
  if (withSign) snprintf_P(buf, n, PSTR("%c%u.%02u"), mv < 0 ? '-' : '+', a / 1000, (a % 1000) / 10);
  else          snprintf_P(buf, n, PSTR("%u.%02u"), a / 1000, (a % 1000) / 10);
}

// Build the pixel data of one page of the main screen.
static void composePage(uint8_t page, uint8_t *line) {
  char a[16], b[16];
  bool invert = false;
  memset(line, 0, 128);

  switch (page) {
    case 0: {                                       // waveform name, output state
      switch (cfg.wave) {
        case WAVE_SINE:     putP(line, 0, PSTR("SINE"));     break;
        case WAVE_TRIANGLE: putP(line, 0, PSTR("TRIANGLE")); break;
        default:            putP(line, 0, PSTR("SQUARE"));   break;
      }
      strcpy_P(a, outputOn ? PSTR("OUT ON") : PSTR("OUT OFF"));
      drawText(line, 128 - textWidth(a, 1), a, 1, 0);
      break;
    }
    case 2: case 3:                                 // frequency, double size
      fmtFreq(a, sizeof(a), cfg.freqHz);
      drawText(line, 4, a, 2, page - 2);
      invert = (field == FIELD_FREQ);
      break;
    case 5:                                         // amplitude (peak and peak-to-peak)
      fmtMv(a, sizeof(a), cfg.ampMv, false);
      fmtMv(b, sizeof(b), (int16_t)(2 * cfg.ampMv), false);
      putP(line, 2, PSTR("Amp"));
      drawText(line, 26, a, 1, 0);
      putP(line, 26 + textWidth(a, 1), PSTR("Vp"));
      drawText(line, 68, b, 1, 0);
      putP(line, 68 + textWidth(b, 1), PSTR("Vpp"));
      invert = (field == FIELD_AMP);
      break;
    case 6:                                         // DC offset
      fmtMv(a, sizeof(a), cfg.offsMv, true);
      putP(line, 2, PSTR("Offset"));
      drawText(line, 44, a, 1, 0);
      putP(line, 44 + textWidth(a, 1), PSTR(" V"));
      invert = (field == FIELD_OFFSET);
      break;
    case 7:                                         // step size of the selected field
      putP(line, 0, PSTR("Step"));
      if (field == FIELD_FREQ) {
        uint32_t st = pgm_read_dword(&FREQ_STEPS[cfg.freqStepIdx]);
        if (st >= 1000UL) snprintf_P(a, sizeof(a), PSTR("%lu kHz"), st / 1000UL);
        else              snprintf_P(a, sizeof(a), PSTR("%lu Hz"), st);
      } else {
        int16_t st = (int16_t)pgm_read_word(&MV_STEPS[field == FIELD_AMP ? cfg.ampStepIdx : cfg.offStepIdx]);
        if (st >= 1000) snprintf_P(a, sizeof(a), PSTR("%d V"), st / 1000);
        else            snprintf_P(a, sizeof(a), PSTR("%d mV"), st);
      }
      drawText(line, 30, a, 1, 0);
      break;
    default:
      break;                                        // pages 1 and 4 stay blank
  }

  if (invert) for (uint8_t i = 0; i < 128; i++) line[i] ^= 0xFF;
}

static void drawUi() {
  if (!displayOk) return;
  uint8_t line[128];
  for (uint8_t p = 0; p < 8; p++) {
    composePage(p, line);
    oledWritePage(p, line);
  }
}

static void showSplash() {
  if (!displayOk) return;
  uint8_t line[128];
  for (uint8_t p = 0; p < 8; p++) {
    memset(line, 0, sizeof(line));
    switch (p) {
      case 0: putP(line, 0, PSTR("FUNCTION GENERATOR"));       break;
      case 1: putP(line, 0, PSTR("AD9833 + Arduino Nano"));    break;
      case 3: putP(line, 0, PSTR("SW3: freq/amp/offset"));     break;
      case 4: putP(line, 0, PSTR("SW2: waveform"));            break;
      case 5: putP(line, 0, PSTR("Knob: turn / step"));        break;
      case 6: putP(line, 0, PSTR("Hold knob: output"));        break;
      default: break;
    }
    oledWritePage(p, line);
  }
  delay(1800);
}

// ============================================================================
// 14. SETUP / LOOP
// ============================================================================
void setup() {
  // Chip selects idle high before anything else touches the SPI bus.
  pinMode(PIN_DDS_FSYNC, OUTPUT);  digitalWrite(PIN_DDS_FSYNC, HIGH);
  pinMode(PIN_POT_CS,    OUTPUT);  digitalWrite(PIN_POT_CS,    HIGH);
  pinMode(PIN_OFFSET_PWM, OUTPUT);

  pinMode(PIN_ENC_A,    INPUT_PULLUP);   // external 10k pull-ups exist; internal ones are harmless
  pinMode(PIN_ENC_B,    INPUT_PULLUP);
  pinMode(PIN_ENC_SW,   INPUT_PULLUP);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);   // SW3 and SW2 have NO external pull-up
  pinMode(PIN_BTN_WAVE, INPUT_PULLUP);

#if DEBUG_SERIAL
  Serial.begin(115200);
#endif

  SPI.begin();
  potWrite(0);                                   // silence the amplitude path
  analogWrite(PIN_OFFSET_PWM, offsetDuty(0));    // 0 V DC offset
  delay(50);                                     // let the AD9833 module power up

  bool factoryReset = (digitalRead(PIN_ENC_SW) == LOW);
  if (factoryReset || !loadSettings()) {
    setDefaults();
    saveSettings();
  }
  sanitize();

  ddsInit();

  displayOk = oledBegin();
#if DEBUG_SERIAL
  if (!displayOk) Serial.println(F("OLED not found at 0x3C - check J6 pin order (SDA/SCL) and address"));
#endif
  showSplash();

  encPrev = (PIND >> 2) & 0x03;
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  outputOn = OUTPUT_ON_AT_BOOT;
  updateWaveform();
  updateOutputState();

#if DEBUG_SERIAL
  Serial.println(F("Function generator ready. Commands: f<Hz> w<0-2> a<mV> o<mV> e<0|1> ?"));
#endif
  printStatus();
  displayDirty = true;
}

void loop() {
  handleInputs();
#if DEBUG_SERIAL
  handleSerial();
#endif

  if (savePending && (millis() - tLastChange) >= SAVE_DELAY_MS) {
    saveSettings();
    savePending = false;
  }

  if (displayDirty && (millis() - tLastDraw) >= DISPLAY_MIN_INTERVAL_MS) {
    displayDirty = false;
    tLastDraw = millis();
    drawUi();                       // ~30 ms of I2C; the encoder ISR keeps counting meanwhile
  }
}
