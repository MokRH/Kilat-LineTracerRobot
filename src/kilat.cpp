/*
 * kilat.cpp
 * =============================================================================
 * Single-source implementation for the KILAT line-following robot library.
 *
 * Consolidates what was previously split across KILAT_Pins.cpp, KILAT_Mux.cpp,
 * KILAT_Motor.cpp, KILAT_LED.cpp, KILAT_Button.cpp, KILAT_IR.cpp,
 * KILAT_PID.cpp, KILAT_OLED.cpp, KILAT_Junction.cpp, KILAT_Bluetooth.cpp,
 * KILAT_Camera.cpp, KILAT_Husky.cpp, KILAT_Servo.cpp and KILAT.cpp.
 *
 * See kilat.h for the public API and configuration.
 *
 * -----------------------------------------------------------------------------
 *  Bug fixes & efficiency improvements preserved from the modular version:
 *
 *  Bugs fixed:
 *    1. `dir == "FWD"` was pointer comparison, not string comparison. Now
 *       dispatches on dir[0] (O(1), always correct).
 *    2. Same pointer-comparison bug for LED colours and junction/action types.
 *    3. JunctionH5.h L101 had `}` instead of `)` - syntax error making the
 *       T-junction branch unreachable. Fixed; observable behaviour preserved.
 *    4. `KILAT_IRupdate()` had no return type (ill-formed C++). Now uint16_t.
 *    5. Comel `readMux(char *item)` compared pointer to int 0. Now takes
 *       uint8_t; IRnumber is uint8_t[].
 *    6. `KILAT_PID` accepted KD but never used it (used KP*50 instead). KD
 *       is now the real derivative gain.
 *    7. `KILAT_PID` division-by-zero on first call (prev_time==0). Guarded.
 *    8. `KILAT_getHusky` had stray `;` after if (empty body) and uninitialized
 *       `result`. Rewritten; result initialised to 0.
 *    9. Unreachable `break;` after `return` in BT/Camera helpers. Removed.
 *   10. `sizeof(IRnumber)/2` only worked because sizeof(char*)==2 on AVR.
 *       Replaced with KILAT_IR_COUNT.
 *   11. `IR_avg` recomputed inside calibration inner loop. Moved outside.
 *   12. Stray `delay(1)` inside calibration inner loop. Removed.
 *
 *  Efficiency wins:
 *    A. `readMux()` - O(1) PROGMEM lookup table (was 12-branch if-else).
 *    B. `KILAT_IRupdate()` - thresholds cached in SRAM (was EEPROM.read per
 *       sensor per loop iteration; ~36 ms/s of CPU at 1 kHz).
 *    C. Three Junction files unified into one (saves ~250 lines).
 *    D. Three `oledDisplay_showIR()` copies merged into one.
 *    E. OLED bitmaps moved to PROGMEM (saves ~80 bytes SRAM).
 *    F. First-char dispatch instead of string compare everywhere.
 *    G. `KILAT_PID` uses float dt (integer division truncated derivative to 0
 *       at typical loop rates).
 *    H. `KILAT_getButton` caches the up/down reading (saves one analogRead).
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 *  Guards — see the "IMPLEMENTATION INCLUSION" block at the bottom of
 *  kilat.h for the full explanation.
 *
 *  This file is included in two ways:
 *
 *    A. #included by kilat.h (from the .ino's translation unit, where the
 *       robot-type #define IS visible). The body below is compiled and
 *       provides all the library symbols in the .ino's TU.
 *
 *    B. Compiled standalone by the Arduino IDE / `avr-g++` as a regular
 *       .cpp. With no robot-type define visible, the #if below is false and
 *       the entire file is empty — producing an empty object file with no
 *       symbols and no #error.
 *
 *  KILAT_IMPLEMENTATION_INCLUDED prevents the body from being compiled
 *  twice in the same TU (which would happen if kilat.cpp is compiled
 *  standalone WITH -D<RobotType> — kilat.cpp includes kilat.h, which
 *  includes kilat.cpp again).
 * --------------------------------------------------------------------------- */
#if defined(HammerHead_5) || defined(HammerHead_11) || defined(Comel2) || defined(Comel3)
#ifndef KILAT_IMPLEMENTATION_INCLUDED
#define KILAT_IMPLEMENTATION_INCLUDED

#include "kilat.h"


/* ========================================================================== *
 *  1.  PIN DEFINITIONS
 * ========================================================================== */
const uint8_t KILAT_PIN_IR_ON = A3;
const uint8_t KILAT_PIN_BUZZ  = 4;

const uint8_t KILAT_Digi_write[3] = { 0, 1, 2 };          /* green, yellow, red */
const uint8_t KILAT_Motor[4]      = { 3, 11, 5, 6 };      /* pwmA+, pwmA-, pwmB+, pwmB- */

#if defined(HammerHead_5) || defined(HammerHead_11)
  const uint8_t KILAT_Mux_sel[3]  = { A1, 13, A0 };       /* s0, s1, s2 */
  const uint8_t KILAT_Mux_read[2] = { A6, A7 };
#elif defined(Comel2) || defined(Comel3)
  const uint8_t KILAT_Mux_sel[3]  = { A2, A0, A1 };
  const uint8_t KILAT_Mux_read[2] = { A7, A7 };           /* only index 0 is used */
#endif

#if defined(HammerHead_11)
  const uint8_t KILAT_IRnumber[KILAT_IR_COUNT] = { 10, 9, 8, 7, 6,
                                                     5,
                                                    4, 3, 2, 1, 0 };
#elif defined(HammerHead_5)
  const uint8_t KILAT_IRnumber[KILAT_IR_COUNT] = { 7, 6, 5, 4, 3 };
#elif defined(Comel2) || defined(Comel3)
  const uint8_t KILAT_IRnumber[KILAT_IR_COUNT] = { 4, 3, 2, 1, 0 };
#endif


/* ========================================================================== *
 *  2.  MUX  (PROGMEM lookup table)
 * ========================================================================== *
 *  Index = logical channel. Value = 6-bit select byte.
 *    bits 0..2 : Mux_sel[0..2]
 *    bits 4..5 : which Mux_read pin (0x10 -> read[1], 0x20 -> read[0])
 *  0xFF marks an unused slot.
 * ========================================================================== */
#if defined(HammerHead_5) || defined(HammerHead_11)
static const uint8_t kMuxTable[KILAT_MUX_TABLE_SIZE] PROGMEM = {
  /* 0  */ 0x10, 0x13, 0x14, 0x16, 0x17, 0x15, 0x25, 0x27,   /* IR 0..7   */
  /* 8  */ 0x26, 0x24, 0x11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   /* IR 8..10  */
  /* 16 */ 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x21, 0x22, 0xFF,   /* 20,21,22  */
  /* 24 */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#elif defined(Comel2) || defined(Comel3)
static const uint8_t kMuxTable[KILAT_MUX_TABLE_SIZE] PROGMEM = {
  /* 0  */ 0x04, 0x06, 0x07, 0x05, 0x02, 0xFF, 0xFF, 0xFF,   /* IR 0..4   */
  /* 8  */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  /* 16 */ 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x01, 0xFF,   /* 20,21,22  */
  /* 24 */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#endif

uint8_t readMux(uint8_t item) {
  if (item >= KILAT_MUX_TABLE_SIZE) return 0;

  uint8_t select_val = pgm_read_byte(&kMuxTable[item]);
  if (select_val == 0xFF) return 0;

  digitalWrite(KILAT_Mux_sel[0], bitRead(select_val, 0));
  digitalWrite(KILAT_Mux_sel[1], bitRead(select_val, 1));
  digitalWrite(KILAT_Mux_sel[2], bitRead(select_val, 2));

  uint8_t readIdx = (select_val & 0x20) ? 0u : 1u;
  return (uint8_t)map(analogRead(KILAT_Mux_read[readIdx]), 0, 1023, 0, 255);
}


/* ========================================================================== *
 *  3.  MOTOR
 * ========================================================================== */
uint8_t turnType = 1;   /* default: turn at centre */

void motor_A(int16_t spdA) {
  spdA = constrain(spdA, -255, 255);
  if (spdA >= 0) {
    analogWrite(KILAT_Motor[0], (uint8_t)spdA);
    digitalWrite(KILAT_Motor[1], LOW);
  } else {
    analogWrite(KILAT_Motor[1], (uint8_t)(-spdA));
    digitalWrite(KILAT_Motor[0], LOW);
  }
}

void motor_B(int16_t spdB) {
  spdB = constrain(spdB, -255, 255);
  if (spdB >= 0) {
    analogWrite(KILAT_Motor[2], (uint8_t)spdB);
    digitalWrite(KILAT_Motor[3], LOW);
  } else {
    analogWrite(KILAT_Motor[3], (uint8_t)(-spdB));
    digitalWrite(KILAT_Motor[2], LOW);
  }
}

void KILAT_movement(const char *dir, uint8_t spdL, uint16_t spdR) {
  spdL = constrain(spdL, 0, 255);
  spdR = constrain(spdR, 0, 255);

  switch (dir[0]) {
    case 'F':   /* FWD */
      motor_A(spdL);  motor_B(spdR);
      break;
    case 'R':
      if (dir[1] == 'E') {            /* REV */
        motor_A(-(int16_t)spdL);  motor_B(-(int16_t)spdR);
      } else {                        /* RIGHT */
        motor_A(spdL);  motor_B(-(int16_t)spdR);
      }
      break;
    case 'L':   /* LEFT */
      motor_A(-(int16_t)spdL);  motor_B(spdR);
      break;
    case 'S':   /* STOP */
    default:
      motor_A(0);  motor_B(0);
      break;
  }
  turnType = 1;
}


/* ========================================================================== *
 *  4.  LED
 * ========================================================================== */
void KILAT_LED(const char *led, bool ON_OFF) {
  uint8_t idx;
  switch (led[0]) {
    case 'r': idx = 2; break;   /* red    */
    case 'y': idx = 1; break;   /* yellow */
    case 'g': idx = 0; break;   /* green  */
    default:  return;
  }
  digitalWrite(KILAT_Digi_write[idx], ON_OFF ? HIGH : LOW);
}


/* ========================================================================== *
 *  5.  BUTTON
 * ========================================================================== */
uint8_t KILAT_getButton(void) {
  /* Cache the up/down channel: the original called readMux(MUX_CH_UP_DOWN)
   * twice - two analogReads (~200 us) of waste per call. */
  uint8_t upDown = readMux(MUX_CH_UP_DOWN);
  uint8_t buttonState = 0;

  if (readMux(MUX_CH_START_BTN) < 100)    buttonState |= 0b00000100;  /* START */
  if (upDown < 100)                       buttonState |= 0b00000001;  /* DOWN  */
  if (upDown > 100 && upDown < 200)       buttonState |= 0b00000010;  /* UP    */

  delay(20);   /* cheap debounce */
  return buttonState;
}


/* ========================================================================== *
 *  6.  IR  (calibration + read-out with SRAM-cached thresholds)
 * ========================================================================== */
uint8_t g_IR_threshold[KILAT_EEPROM_BYTES];

void KILAT_IRinit(void) {
  for (uint8_t i = 0; i < KILAT_EEPROM_BYTES; i++)
    g_IR_threshold[i] = EEPROM.read(i);
}

void KILAT_IRcalibrate(void) {
  /* Only run when UP is held and the battery is healthy. */
  if (KILAT_getButton() != 0b00000010) return;
  if (readMux(MUX_CH_BATT_MON) <= 100) return;

  uint8_t IR_min[KILAT_IR_COUNT];
  uint8_t IR_max[KILAT_IR_COUNT];
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    IR_min[i] = 255;
    IR_max[i] = 0;
  }

  /* Blink all 3 LEDs to tell the user calibration is about to start. */
  while (KILAT_getButton() == 0b00000010) {
    KILAT_LED("green", 1); KILAT_LED("yellow", 0); KILAT_LED("red", 0);
    delay(200);
    KILAT_LED("green", 0); KILAT_LED("yellow", 1); KILAT_LED("red", 0);
    delay(200);
    KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 1);
    delay(200);
  }
  KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 0);

  oledDisplay_char(1, 0, "READING");
  oledDisplay_char(2, 1, "OFFSET");
  delay(1000);

  /* Sweep forward for half the window, reverse for the other half. */
  uint8_t dispCount = 13, dispShift = 0;
  for (uint8_t j = 0; j < 200; j++) {
    if (j < 100) KILAT_movement("FWD", 80, 80);
    else         KILAT_movement("REV", 80, 80);

    for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
      uint8_t v = readMux(KILAT_IRnumber[i]);
      if (v < IR_min[i]) IR_min[i] = v;
      if (v > IR_max[i]) IR_max[i] = v;
    }

    /* Progress bar: draw one '-' every 14 samples. */
    if (++dispCount == 14) {
      oledDisplay_char(dispShift, 3, "-");
      dispShift++;
      dispCount = 0;
    }
  }

  KILAT_movement("STOP", 0, 0);

  /* Compute and persist thresholds. The 3.0x / 1.04x multipliers come from
   * the original firmware and reflect different IR hardware on each robot. */
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    uint8_t avg = (uint8_t)(((uint16_t)IR_min[i] + IR_max[i]) / 2);
#if defined(Comel2)
    avg = (uint8_t)(avg * 1.04f);
#elif defined(Comel3) || defined(HammerHead_5) || defined(HammerHead_11)
    avg = (uint8_t)(IR_min[i] * 3.0f);
#endif
    EEPROM.write(i, avg);
    g_IR_threshold[i] = avg;   /* keep cache in sync */
  }

#ifndef oled_OFF
  oledDisplay_char(0, 0, "        "); oledDisplay_char(2, 0, "OFFSET");
  oledDisplay_char(0, 1, "        "); oledDisplay_char(3, 1, "SAVED");

  char getChar[9] = "";
  String("val:" + String(g_IR_threshold[0]) + " ").toCharArray(getChar, sizeof(getChar));
  oledDisplay_char(0, 3, getChar);
  delay(1000);
  oledDisplay_clear();
#endif
}

uint16_t KILAT_IRupdate(bool line_color) {
  uint16_t IR_position = 0;
  if (line_color == false) {
    /* Black line: sensor "on line" when reading > threshold. */
    for (uint8_t i = 0; i < KILAT_IR_COUNT; i++)
      if (readMux(KILAT_IRnumber[i]) > g_IR_threshold[i])
        bitSet(IR_position, i);
  } else {
    /* White line: the opposite. */
    for (uint8_t i = 0; i < KILAT_IR_COUNT; i++)
      if (readMux(KILAT_IRnumber[i]) < g_IR_threshold[i])
        bitSet(IR_position, i);
  }
  return IR_position;
}


/* ========================================================================== *
 *  7.  PID
 * ========================================================================== *
 *  Original buggy formula:
 *    control = (KP * current_error) + (KP * 50 * (d_error / dt))
 *  - KD accepted but ignored (used KP*50 instead).
 *  - Integer division by dt collapsed sub-ms dt to 0.
 *  - First call: 0/0 = NaN.
 *
 *  Fixed:
 *    control = KP * current_error + KD * (d_error / dt)   // float dt, guarded
 * ========================================================================== */
static int32_t pid_prev_error = 0;
static int32_t pid_current_time = 0, pid_prev_time = 0;

void KILAT_PID(uint8_t spd, int16_t current_error, float KP, float KD) {
  pid_current_time = millis();

  int32_t dt = pid_current_time - pid_prev_time;
  float derivative = (dt > 0)
      ? ((float)(current_error - pid_prev_error) / (float)dt)
      : 0.0f;

  float control = (KP * (float)current_error) + (KD * derivative);

  pid_prev_error = current_error;
  pid_prev_time  = pid_current_time;

  int16_t left  = (int16_t)spd + (int16_t)control;
  int16_t right = (int16_t)spd - (int16_t)control;
  motor_A(constrain(left,  -(int16_t)spd, (int16_t)spd));
  motor_B(constrain(right, -(int16_t)spd, (int16_t)spd));
}


/* ========================================================================== *
 *  8.  OLED
 * ========================================================================== */
#ifndef oled_OFF
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

/* ---- Bitmap glyphs (PROGMEM) -------------------------------------------- */
static const uint8_t kBattOutline[16] PROGMEM = {
  0xfc, 0x04, 0x07, 0x01, 0x01, 0x07, 0x04, 0xfc,
  0xff, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xff
};
static const uint8_t kLowBatt[16]    PROGMEM = {
  0x00, 0x9f, 0x50, 0x50, 0x50, 0x80, 0x00, 0x00,
  0x00, 0x73, 0x84, 0x64, 0x84, 0x73, 0x00, 0x00
};
static const uint8_t kBarBase[24]    PROGMEM = {
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00
};
#endif

void oledDisplay_setup() {
#ifndef oled_OFF
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_lucasarts_scumm_subtitle_r_2x2_f);
#if defined(HammerHead_5) || defined(HammerHead_11)
  u8x8.setFlipMode(1);
#endif
#if defined(Comel2) || defined(Comel3)
  u8x8.setFlipMode(0);
#endif
#endif
}

void oledDisplay_char(uint8_t column, uint8_t line, const char *string) {
#ifndef oled_OFF
  u8x8.drawString(column, line * 2, (char *)string);
#endif
}

void oledDisplay_clear(void) {
#ifndef oled_OFF
  u8x8.clear();
#endif
}

void oledDisplay_showName() {
#ifndef oled_OFF
#if defined(HammerHead_5) || defined(HammerHead_11)
  oledDisplay_char(0, 0, " Hammer ");
  oledDisplay_char(0, 1, "  Head  ");
#elif defined(Comel2)
  oledDisplay_char(3, 0, "Kilat");
  oledDisplay_char(2, 1, "Comel2");
#elif defined(Comel3)
  oledDisplay_char(3, 0, "Kilat");
  oledDisplay_char(2, 1, "Comel3");
#endif
#endif
}

void oledDisplay_showBatt() {
#ifndef oled_OFF
  uint8_t battPercent = (constrain(readMux(MUX_CH_BATT_MON), 128, 150) - 128) * 100 / (150 - 128);
  battPercent = constrain(battPercent, 0, 100);
  uint8_t level = map(battPercent, 0, 100, 0, 14);

  static uint8_t level_old = 15;
  if (level == level_old) return;
  level_old = level;

  uint8_t drawBatt[16];
  memcpy_P(drawBatt, kBattOutline, sizeof(drawBatt));

  if (level > 2) {
    uint8_t fill = level - 1;
    for (uint8_t j = 0; j < 8; j++) {
      for (uint8_t i = 0; i < fill && i < 14; i++) {
        if (i < 7)      bitSet(drawBatt[8 + j], 7 - i);
        else            bitSet(drawBatt[0 + j], 14 - i);
      }
    }
  } else {
    memcpy_P(drawBatt, kLowBatt, sizeof(drawBatt));
  }

  u8x8.drawTile(0, 6, 1, drawBatt);
  u8x8.drawTile(0, 7, 1, drawBatt + 8);
#endif
}

void oledDisplay_showBar(uint8_t pos_y, uint8_t pos_x, uint8_t percent) {
#ifndef oled_OFF
  uint8_t drawBar[24];
  memcpy_P(drawBar, kBarBase, sizeof(drawBar));

  percent = constrain(percent, 0, 100);
  uint8_t level = map(percent, 0, 100, 0, 24);

  for (uint8_t j = 0; j < 7; j++) {
    for (uint8_t i = 0; i < level; i++) {
      if      (i <  7) bitSet(drawBar[16 + j], 7 - i);
      else if (i < 14) bitSet(drawBar[ 8 + j], 14 - i);
      else             bitSet(drawBar[ 0 + j], 21 - i);
    }
  }

  u8x8.drawTile(pos_y, pos_x - 2, 1, drawBar);
  u8x8.drawTile(pos_y, pos_x - 1, 1, drawBar + 8);
  u8x8.drawTile(pos_y, pos_x,     1, drawBar + 16);
#endif
}

void oledDisplay_showIR() {
#ifndef oled_OFF
  const uint8_t Ub = 255;
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    uint8_t getValue = constrain(readMux(KILAT_IRnumber[i]), g_IR_threshold[i], Ub);
    oledDisplay_showBar((KILAT_IR_CENTER + 7) - i, 7,
                        map(getValue, g_IR_threshold[i], Ub, 0, 100));

    bool onLine = (getValue > g_IR_threshold[i]);
    if      (i == KILAT_IR_COUNT - 1)  KILAT_LED("green",  onLine);   /* far left  */
    else if (i == KILAT_IR_CENTER)     KILAT_LED("yellow", onLine);   /* centre    */
    else if (i == 0)                   KILAT_LED("red",    onLine);   /* far right */
  }
#endif
}

static uint32_t oled_startTime = 0;

void oledDisplay_time(uint8_t StartStop) {
  char getChar[12] = "";

  if (StartStop == 0) {
    oledDisplay_char(0, 0, "        ");
    oledDisplay_char(3, 0, "Start");
    oledDisplay_char(0, 1, "  Time  ");
    oledDisplay_char(0, 3, "0.00 sec");
    oled_startTime = millis();
    return;
  }

  /* Finish: build the time string once, then blink forever. */
  uint32_t totalTime = millis() - oled_startTime;
  uint16_t sec = 0;
  while (totalTime >= 1000) { totalTime -= 1000; sec++; }

  if (totalTime >= 100)
    String(String(sec) + "." + String(totalTime / 10) + "sec").toCharArray(getChar, sizeof(getChar));
  else
    String(String(sec) + ".0" + String(totalTime / 10) + "sec").toCharArray(getChar, sizeof(getChar));

  oledDisplay_char(0, 0, " Finish ");
  oledDisplay_char(0, 3, getChar);

  KILAT_movement("STOP", 0, 0);
  while (1) {
    for (uint8_t i = 0; i < 2; i++) {
      KILAT_LED("green", 0); KILAT_LED("red", 0); KILAT_LED("yellow", 0);
      delay(100);
      KILAT_LED("green", 1); KILAT_LED("red", 1); KILAT_LED("yellow", 0);
      delay(50);
    }
    KILAT_LED("green", 0); KILAT_LED("red", 0); KILAT_LED("yellow", 1);
    delay(300);
  }
}


/* ========================================================================== *
 *  9.  JUNCTION  (unified - was 3 duplicate files)
 * ========================================================================== */

/* ---- Per-robot junction-internal constants ----------------------------- */
#if defined(HammerHead_11)
  #define ACT_SPD_LO               25u
  #define TURN_LEFT_LO_BIT         7u
  #define TURN_LEFT_HI_BIT         8u
  #define TURN_RIGHT_LO_BIT        2u
  #define TURN_RIGHT_HI_BIT        3u
  #define SKIP_SETS_TURNTYPE_FIRST 1
#elif defined(HammerHead_5)
  #define ACT_SPD_LO               25u
  #define TURN_LEFT_LO_BIT         3u
  #define TURN_LEFT_HI_BIT         4u
  #define TURN_RIGHT_LO_BIT        0u
  #define TURN_RIGHT_HI_BIT        1u
  #define SKIP_SETS_TURNTYPE_FIRST 1
#elif defined(Comel2) || defined(Comel3)
  #define ACT_SPD_LO               50u
  #define TURN_LEFT_LO_BIT         3u
  #define TURN_LEFT_HI_BIT         4u
  #define TURN_RIGHT_LO_BIT        0u
  #define TURN_RIGHT_HI_BIT        1u
  #define SKIP_SETS_TURNTYPE_FIRST 0
#endif

/* Y-junction patterns. H5 only checked 2 of the 3 patterns; preserved. */
#if defined(HammerHead_5)
  #define IS_Y_JUNCTION(v)  (((v) & 0b10010) == 0b10010 || ((v) & 0b01001) == 0b01001)
#else
  #define IS_Y_JUNCTION(v)  (((v) & 0b10010) == 0b10010 || \
                             ((v) & 0b10001) == 0b10001 || \
                             ((v) & 0b01001) == 0b01001)
#endif

/* T-junction patterns. H5 had a syntax bug disabling the 2nd pattern;
 * preserving the observable behaviour. */
#if defined(HammerHead_5)
  #define IS_T_JUNCTION(v)  (((v) & 0b10101) == 0b10101)
#else
  #define IS_T_JUNCTION(v)  (((v) & 0b10101) == 0b10101 || ((v) & 0b01110) == 0b01110)
#endif

/* ---- Module state ------------------------------------------------------- */
uint8_t last_IR = 0b01;          /* 0b10 = last seen left, 0b01 = right */
static uint32_t timer_spdUP = 0;

/* ---- Helper: aggregate 11-bit IR into 5-bit pattern (H11 only) --------- */
#if KILAT_HAS_LINE_AGG
static inline uint8_t aggregate_pattern(uint16_t ir) {
  uint8_t p = 0;
  bitWrite(p, 4, bitRead(ir, 10) | bitRead(ir,  9));                     /* left_2  */
  bitWrite(p, 3, bitRead(ir,  8) | bitRead(ir,  7));                     /* left_1  */
  bitWrite(p, 2, bitRead(ir,  6) | bitRead(ir,  5) | bitRead(ir, 4));    /* mid     */
  bitWrite(p, 1, bitRead(ir,  3) | bitRead(ir,  2));                     /* right_1 */
  bitWrite(p, 0, bitRead(ir,  1) | bitRead(ir,  0));                     /* right_2 */
  return p;
}
#endif

/* ---- Helper: PID error from raw IR word -------------------------------- */
static inline int16_t compute_error(uint16_t ir) {
  int16_t error = 0;
  uint8_t total = 0;
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    if (bitRead(ir, i)) {
      error += (int16_t)(i - KILAT_IR_CENTER) * -100;
      total++;
    }
  }
  return (total > 0) ? (error / (int16_t)total) : 0;
}

/* ---- Helper: update last_IR from outermost sensors --------------------- */
static inline void update_last_IR(uint16_t ir) {
#if KILAT_HAS_LINE_AGG
  uint8_t p = aggregate_pattern(ir);
  uint8_t leftBit  = bitRead(p, 4);
  uint8_t rightBit = bitRead(p, 0);
#else
  uint8_t leftBit  = bitRead(ir, KILAT_IR_COUNT - 1);
  uint8_t rightBit = bitRead(ir, 0);
#endif
  if (leftBit && !rightBit)       last_IR = 0b10;
  else if (!leftBit && rightBit)  last_IR = 0b01;
}

/* ---- action() ---------------------------------------------------------- */
void action(const char *action_, uint8_t action_spd, uint16_t action_duration, bool line_color) {
  bool turnLeft  = (action_[0] == 'L');
  bool turnRight = (action_[0] == 'R');
  if (!turnLeft && !turnRight) {
    KILAT_movement("STOP", 0, 0);
    return;
  }

  if (turnType == 1) {
    if (action_spd == 0) action_spd = KILAT_ACT_SPD_T1;
    else                 action_spd = constrain(action_spd, ACT_SPD_LO, 150);
    if (turnRight) KILAT_movement("RIGHT", action_spd, action_spd);
    else           KILAT_movement("LEFT",  action_spd, action_spd);
  }
  else if (turnType == 2) {
    if (action_spd == 0) action_spd = KILAT_ACT_SPD_T2;
    else                 action_spd = constrain(action_spd, ACT_SPD_LO, 150);
    if (turnRight) KILAT_movement("RIGHT", action_spd, map(action_spd, 25, 150, 0, 31));
    else           KILAT_movement("LEFT",  map(action_spd, 25, 150, 0, 31), action_spd);
  }
#if KILAT_HAS_TURNTYPE3
  else if (turnType == 3) {
    if (action_spd == 0) action_spd = KILAT_ACT_SPD_T3;
    else                 action_spd = constrain(action_spd, ACT_SPD_LO, 150);
    if (turnRight) KILAT_movement("RIGHT", action_spd, map(action_spd, 25, 150, 0, 63));
    else           KILAT_movement("LEFT",  map(action_spd, 25, 150, 0, 63), action_spd);
  }
#endif

  delay(map(action_spd, ACT_SPD_LO, 150, 150, 50));

  if (action_duration == 0) {
    while (1) {
      uint16_t get_IR = KILAT_IRupdate(line_color) & KILAT_IR_MASK;
      if (turnLeft &&
          (bitRead(get_IR, TURN_LEFT_LO_BIT) || bitRead(get_IR, TURN_LEFT_HI_BIT))) {
        last_IR = 0b10;
        break;
      }
      if (turnRight &&
          (bitRead(get_IR, TURN_RIGHT_LO_BIT) || bitRead(get_IR, TURN_RIGHT_HI_BIT))) {
        last_IR = 0b01;
        break;
      }
    }
  } else {
    delay(action_duration);
  }

  KILAT_movement("STOP", 0, 0);
  delay(25);
}

/* ---- junction() -------------------------------------------------------- */
void junction(const char *junc_type, uint16_t spd, float KP, float KD,
              int16_t skipDuration, bool gap, bool line_color) {
  uint32_t enterTime  = millis();
  uint8_t  spd_target = constrain(spd, KILAT_SPD_TARGET_MIN, 255);
  spd = constrain(spd, KILAT_SPD_LO, KILAT_SPD_HI);

  char jType = junc_type[0];   /* 'Y', 'T', 'L', 'R', 'S' */

  while (1) {
    uint16_t get_IR = KILAT_IRupdate(line_color) & KILAT_IR_MASK;
    KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 0);

    if (get_IR == 0) {
      /* ---- Robot lost the line ---- */
      if (gap == 0) {
        uint8_t traceBack_spd = map(spd, 50, 255, KILAT_TRACE_LO, KILAT_TRACE_HI);
        if (last_IR == 0b10) KILAT_movement("LEFT",  traceBack_spd, traceBack_spd);
        else                 KILAT_movement("RIGHT", traceBack_spd, traceBack_spd);
#if defined(HammerHead_11)
        if (spd > 150) spd = 150;
#endif
      } else {
        KILAT_movement("FWD", spd, spd);
      }
      continue;
    }

    /* ---- At least one sensor on the line ---- */
    if (spd < spd_target) {
      if (millis() - timer_spdUP >= KILAT_SPD_UP_MS) {
        timer_spdUP = millis();
        spd += KILAT_SPD_UP_STEP;
      }
    } else {
      spd = spd_target;
    }

#if KILAT_HAS_LINE_AGG
    uint8_t pattern = aggregate_pattern(get_IR);
#else
    uint16_t pattern = get_IR;
#endif

    if      (jType == 'Y' && IS_Y_JUNCTION(pattern))         { turnType = 2;               break; }
    else if (jType == 'T' && IS_T_JUNCTION(pattern))         { turnType = KILAT_TURN_T;    break; }
    else if (jType == 'L' && (pattern & 0b10101) == 0b10100) { turnType = KILAT_TURN_SIDE; break; }
    else if (jType == 'R' && (pattern & 0b10101) == 0b00101) { turnType = KILAT_TURN_SIDE; break; }
    else if (jType == 'S' && (millis() - enterTime) >= (uint32_t)skipDuration) {
#if SKIP_SETS_TURNTYPE_FIRST
        turnType = 1;
        if (skipDuration != -1) break;
#else
        if (skipDuration != -1) { turnType = 1; break; }
#endif
    }
    else {
      int16_t error = compute_error(get_IR);
      KILAT_PID(spd, error, KP, KD);
    }

    update_last_IR(get_IR);
  }
}


/* ========================================================================== *
 *  10.  OPTIONAL: BLUETOOTH
 * ========================================================================== */
#ifdef BT_ON
SoftwareSerial btSerial(7, 8);   /* arRx, arTx */

void KILAT_btSetup() {
  btSerial.begin(115200);
}

void KILAT_btRename(const char *get_btChar) {
  btSerial.write("AT+NAME");
  btSerial.write(get_btChar);
  btSerial.write("\r\n");
  delay(100);
}

char KILAT_getBT() {
  while (btSerial.available() > 0)
    btSerial.read();
  while (1) {
    if (btSerial.available() > 0)
      return (char)btSerial.read();
  }
}
#endif


/* ========================================================================== *
 *  11.  OPTIONAL: CAMERA
 * ========================================================================== */
#ifdef camera_ON
SoftwareSerial cameraSerial(12, 10);   /* auxRx, auxTx */

void KILAT_cameraSetup() {
  cameraSerial.begin(115200);
}

char KILAT_getCamera(char defaultImage, uint16_t maxDuration) {
  turnType = 1;
  uint32_t enterTime = millis();

  while (cameraSerial.available() > 0)
    cameraSerial.read();

  while (1) {
    if (cameraSerial.available() > 0)
      return (char)cameraSerial.read();
    if ((millis() - enterTime) > maxDuration)
      return defaultImage;
  }
}
#endif


/* ========================================================================== *
 *  12.  OPTIONAL: HUSKYLENS
 * ========================================================================== */
#ifdef Husky_ON
HUSKYLENS huskylens;

void KILAT_HuskySetup() {
  Wire.begin();
  while (!huskylens.begin(Wire)) {
    /* Spin until the HUSKYLENS answers. */
  }
}

uint8_t KILAT_getHusky() {
  if (!huskylens.request())     return 0;
  if (!huskylens.isLearned())   return 0;
  if (!huskylens.available())   return 0;

  uint8_t result = 0;
  while (huskylens.available()) {
    result = huskylens.read().ID;
  }
  return result;
}
#endif


/* ========================================================================== *
 *  13.  OPTIONAL: SERVOS
 * ========================================================================== */
#ifdef Servo1_ON
Servo servo1;
void KILAT_gripper_1(uint8_t angle, uint16_t gripperDelay) {
  servo1.attach(9);
  servo1.write(angle);
  delay(gripperDelay);
  turnType = 1;
}
#endif

#ifdef Servo2_ON
Servo servo2;
void KILAT_gripper_2(uint8_t angle, uint16_t gripperDelay) {
  servo2.attach(10);
  servo2.write(angle);
  delay(gripperDelay);
  turnType = 1;
}
#endif


/* ========================================================================== *
 *  14.  SETUP + WAIT-TO-START
 * ========================================================================== */

/* Helper: configure an array of pins as OUTPUT (or INPUT) in one go. */
template <size_t N>
static inline void configurePins(const uint8_t (&pins)[N], uint8_t mode, bool initLow) {
  for (size_t i = 0; i < N; i++) {
    pinMode(pins[i], mode);
    if (mode == OUTPUT && initLow) digitalWrite(pins[i], LOW);
  }
}

void KILAT_waitToStart(void) {
  oledDisplay_showName();

  while (1) {
    oledDisplay_showBatt();
    oledDisplay_showIR();

    uint8_t getButton = KILAT_getButton();
    if (getButton == 0b010) {                  /* UP    -> tiny forward nudge */
      KILAT_movement("FWD", 100, 100);
      while (KILAT_getButton() == 0b010) { }
      KILAT_movement("STOP", 0, 0);
    } else if (getButton == 0b001) {           /* DOWN  -> tiny reverse nudge */
      KILAT_movement("REV", 100, 100);
      while (KILAT_getButton() == 0b001) { }
      KILAT_movement("STOP", 0, 0);
    } else if (getButton == 0b100) {           /* START -> exit loop         */
      break;
    }
  }

  oledDisplay_char(0, 2, "        ");
  oledDisplay_char(0, 3, "        ");
  oledDisplay_char(3, 3, "READY");
  while (KILAT_getButton() != 0b000) { }
}

void KILAT_setup(void) {
  /* ---- Pin configuration ---- */
  configurePins(KILAT_Mux_read,   INPUT,  false);
  configurePins(KILAT_Mux_sel,    OUTPUT, true);
  configurePins(KILAT_Motor,      OUTPUT, true);
  configurePins(KILAT_Digi_write, OUTPUT, true);

  pinMode(KILAT_PIN_BUZZ,  OUTPUT); digitalWrite(KILAT_PIN_BUZZ,  LOW);
  pinMode(KILAT_PIN_IR_ON, OUTPUT); digitalWrite(KILAT_PIN_IR_ON, KILAT_IR_ON_DEFAULT);

  /* ---- PWM timer prescalers (980 Hz on D3/D11, 976 Hz on D5/D6) ---- */
  TCCR2B = 0b00000100;   /* x64 */
  TCCR2A = 0b00000011;   /* fast PWM */
  TCCR0B = 0b00000011;   /* x64 */
  TCCR0A = 0b00000011;   /* fast PWM */

  /* ---- OLED + cached IR thresholds ---- */
  oledDisplay_setup();
  KILAT_IRinit();

  /* ---- Optional peripherals ---- */
#ifdef BT_ON
  KILAT_btSetup();
#endif
#ifdef camera_ON
  KILAT_cameraSetup();
#endif
#ifdef Husky_ON
  KILAT_HuskySetup();
#endif

  /* ---- Calibration + wait for START ---- */
  KILAT_IRcalibrate();
  KILAT_waitToStart();

  KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 0);
  oledDisplay_time(0);
}

#endif /* KILAT_IMPLEMENTATION_INCLUDED */
#endif /* defined(HammerHead_5) || defined(HammerHead_11) || defined(Comel2) || defined(Comel3) */
