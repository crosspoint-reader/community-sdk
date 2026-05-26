#include "EInkDisplay.h"

#include <BoardConfig.h>

#include <cstring>
#include <fstream>
#include <vector>

// SSD1677 command definitions
// Initialization and reset
#define CMD_SOFT_RESET 0x12             // Soft reset
#define CMD_BOOSTER_SOFT_START 0x0C     // Booster soft-start control
#define CMD_DRIVER_OUTPUT_CONTROL 0x01  // Driver output control
#define CMD_BORDER_WAVEFORM 0x3C        // Border waveform control
#define CMD_TEMP_SENSOR_CONTROL 0x18    // Temperature sensor control

// RAM and buffer management
#define CMD_DATA_ENTRY_MODE 0x11     // Data entry mode
#define CMD_SET_RAM_X_RANGE 0x44     // Set RAM X address range
#define CMD_SET_RAM_Y_RANGE 0x45     // Set RAM Y address range
#define CMD_SET_RAM_X_COUNTER 0x4E   // Set RAM X address counter
#define CMD_SET_RAM_Y_COUNTER 0x4F   // Set RAM Y address counter
#define CMD_WRITE_RAM_BW 0x24        // Write to BW RAM (current frame)
#define CMD_WRITE_RAM_RED 0x26       // Write to RED RAM (used for fast refresh)
#define CMD_AUTO_WRITE_BW_RAM 0x46   // Auto write BW RAM
#define CMD_AUTO_WRITE_RED_RAM 0x47  // Auto write RED RAM

// Display update and refresh
#define CMD_DISPLAY_UPDATE_CTRL1 0x21  // Display update control 1
#define CMD_DISPLAY_UPDATE_CTRL2 0x22  // Display update control 2
#define CMD_MASTER_ACTIVATION 0x20     // Master activation
#define CTRL1_NORMAL 0x00              // Normal mode - compare RED vs BW for partial
#define CTRL1_BYPASS_RED 0x40          // Bypass RED RAM (treat as 0) - for full refresh

// LUT and voltage settings
#define CMD_WRITE_LUT 0x32       // Write LUT
#define CMD_GATE_VOLTAGE 0x03    // Gate voltage
#define CMD_SOURCE_VOLTAGE 0x04  // Source voltage
#define CMD_WRITE_VCOM 0x2C      // Write VCOM
#define CMD_WRITE_TEMP 0x1A      // Write temperature

// Power management
#define CMD_DEEP_SLEEP 0x10  // Deep sleep

#define CMD_UC8253_PANEL_SETTING       0x00
#define CMD_UC8253_POWER_OFF           0x02
#define CMD_UC8253_POWER_ON            0x04
#define CMD_UC8253_DEEP_SLEEP          0x07
#define CMD_UC8253_DTM1                0x10
#define CMD_UC8253_DISPLAY_REFRESH     0x12
#define CMD_UC8253_DTM2                0x13
#define CMD_UC8253_VCOM_DATA_INTERVAL  0x50
#define CMD_UC8253_POWER_SETTING       0x01
#define CMD_UC8253_BOOSTER_SOFT_START  0x06
#define CMD_UC8253_PLL_CONTROL         0x30
#define CMD_UC8253_RESOLUTION_SETTING  0x61
#define CMD_UC8253_VCOM_DC_SETTING     0x82

#ifndef MURPHY_LOAD_OEM_LUT_EACH_REFRESH
#define MURPHY_LOAD_OEM_LUT_EACH_REFRESH 1
#endif

static void logMurphyPinLevels(const char* label) {
  if (!Serial) return;
  Serial.printf("[%lu]   Murphy pins %s: MOSI3=%d SCK4=%d CS5=%d DC6=%d RST7=%d BUSY8=%d FL48=%d\n",
                millis(), label, digitalRead(3), digitalRead(4), digitalRead(5), digitalRead(6),
                digitalRead(7), digitalRead(8), digitalRead(48));
}

static uint32_t countMurphyBlackPixels(const uint8_t* data, uint32_t size) {
  if (!data) return 0;
  uint32_t blackPixels = 0;
  for (uint32_t i = 0; i < size; i++) {
    blackPixels += __builtin_popcount(static_cast<uint8_t>(~data[i]));
  }
  return blackPixels;
}

static constexpr uint8_t MURPHY_LUT_20_DEFAULT[] = {
    0x01, 0x08, 0x08, 0x08, 0x08, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01,
    0x01, 0x08, 0x08, 0x08, 0x08, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_21_DEFAULT[] = {
    0x01, 0x48, 0x48, 0x48, 0x48, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01,
    0x01, 0x88, 0x88, 0x88, 0x88, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_22_DEFAULT[] = {
    0x01, 0x48, 0x48, 0x48, 0x48, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01,
    0x01, 0x88, 0x88, 0x88, 0x88, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_23_DEFAULT[] = {
    0x01, 0x88, 0x88, 0x88, 0x88, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01,
    0x01, 0x48, 0x48, 0x48, 0x48, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_24_DEFAULT[] = {
    0x01, 0x88, 0x88, 0x88, 0x88, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01,
    0x01, 0x48, 0x48, 0x48, 0x48, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Alternate (fast/short) OEM LUT set. Reverse-engineered from Murphy_M3 firmware
// at flash addresses 0x3c236fb6..0x3c23706c. Shorter on-times than the default
// set (frame counts collapsed into the first four phases). Loaded by the OEM
// when the alternate init branch (power_setting MURPHY_OEM_POWER_SETTING_ALT)
// is used. LUT_20_ALT and LUT_23_ALT_B are 56 bytes; the rest are 42 bytes.
static constexpr uint8_t MURPHY_LUT_20_ALT[] = {
    0x01, 0x0F, 0x0F, 0x0F, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_21_ALT[] = {
    0x01, 0x4F, 0x8F, 0x0F, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_22_ALT[] = {
    0x01, 0x4F, 0x8F, 0x4F, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_23_ALT[] = {
    0x01, 0x0F, 0x8F, 0x0F, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static constexpr uint8_t MURPHY_LUT_24_ALT[] = {
    0x01, 0x0F, 0x8F, 0x4F, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Alternate OEM power_setting payload (0x3c236ca3). The simple branch — used by
// initMurphyM3Controller() today — uses {0x03,0x10,0x3F,0x3B,0x0D}.
static constexpr uint8_t MURPHY_OEM_POWER_SETTING_ALT[] = {0x03, 0x10, 0x3F, 0x3F, 0x03};

// Partial-refresh trigger pair seen in the alt branch after data write. Kept as
// constants for future bring-up; not used by the current refresh path.
static constexpr uint8_t MURPHY_CMD_PARTIAL_WINDOW = 0x17;
static constexpr uint8_t MURPHY_CMD_PARTIAL_TRIGGER_ARG = 0xA5;

// Custom LUT for fast refresh
const unsigned char lut_grayscale[] PROGMEM = {
    // 00 black/white
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 01 light gray
    0x54, 0x54, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 10 gray
    0xAA, 0xA0, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 11 dark gray
    0xA2, 0x22, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // L4 (VCOM)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // TP/RP groups (global timing)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G0: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G1: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G2: A=0 B=0 C=0 D=0 RP=0 (4 frames)
    0x00, 0x00, 0x00, 0x00, 0x00,  // G3: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G4: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G5: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G6: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G7: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G8: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G9: A=0 B=0 C=0 D=0 RP=0

    // Frame rate
    0x8F, 0x8F, 0x8F, 0x8F, 0x8F,

    // Voltages (VGH, VSH1, VSH2, VSL, VCOM)
    0x17, 0x41, 0xA8, 0x32, 0x30,

    // Reserved
    0x00, 0x00};

const unsigned char lut_grayscale_revert[] PROGMEM = {
    // 00 black/white
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 10 gray
    0x54, 0x54, 0x54, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 01 light gray
    0xA8, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 11 dark gray
    0xFC, 0xFC, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // L4 (VCOM)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // TP/RP groups (global timing)
    0x01, 0x01, 0x01, 0x01, 0x01,  // G0: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x01,  // G1: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G2: A=0 B=0 C=0 D=0 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G3: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G4: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G5: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G6: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G7: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G8: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G9: A=0 B=0 C=0 D=0 RP=0

    // Frame rate
    0x8F, 0x8F, 0x8F, 0x8F, 0x8F,

    // Voltages (VGH, VSH1, VSH2, VSL, VCOM)
    0x17, 0x41, 0xA8, 0x32, 0x30,

    // Reserved
    0x00, 0x00};

// X3 reverse-exact full refresh LUTs (42 bytes each)
const uint8_t lut_x3_vcom_full[] PROGMEM = {
    0x00, 0x06, 0x02, 0x06, 0x06, 0x01, 0x00, 0x05, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_ww_full[] PROGMEM = {
    0x20, 0x06, 0x02, 0x06, 0x06, 0x01, 0x00, 0x05, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bw_full[] PROGMEM = {
    0xAA, 0x06, 0x02, 0x06, 0x06, 0x01, 0x80, 0x05, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_wb_full[] PROGMEM = {
    0x55, 0x06, 0x02, 0x06, 0x06, 0x01, 0x40, 0x05, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bb_full[] PROGMEM = {
    0x10, 0x06, 0x02, 0x06, 0x06, 0x01, 0x00, 0x05, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// X3 dedicated grayscale LUTs — tuned drive strengths for 4-level gray
// All entries share the same single-phase timing so the controller scans
// every row with consistent gate timing. Source voltages differ per transition:
//   VCOM: GND (stable common electrode reference)
//   BB:   GND (active hold — prevents floating source crosstalk)
//   WW:   brief VDL pulse (dark gray)
//   BW:   moderate VDL pulse (light gray)
//   WB:   GND (active hold — unused transition)
const uint8_t lut_x3_vcom_gray[] PROGMEM = {
    0x00, 0x03, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_ww_gray[] PROGMEM = {
    // Dark gray: VS=0x20 → GND,VDL(2),GND,GND — brief pulse (sub-phase B)
    0x20, 0x03, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bw_gray[] PROGMEM = {
    // Light gray: VS=0x80 → VDL(3),GND,GND,GND — subtle pulse (sub-phase A, TP0=3)
    0x80, 0x03, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_wb_gray[] PROGMEM = {
    // Active GND hold: VS=0x00 → all GND, matching timing
    0x00, 0x03, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bb_gray[] PROGMEM = {
    // Active GND hold: VS=0x00 → all GND, matching timing
    0x00, 0x03, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// X3 stock image-write LUTs
const uint8_t lut_x3_vcom_img[] PROGMEM = {
    0x00, 0x08, 0x0B, 0x02, 0x03, 0x01, 0x00, 0x0C, 0x02, 0x07, 0x02, 0x01, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_ww_img[] PROGMEM = {
    0xA8, 0x08, 0x0B, 0x02, 0x03, 0x01, 0x44, 0x0C, 0x02, 0x07, 0x02, 0x01, 0x04, 0x01,
    0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bw_img[] PROGMEM = {
    0x80, 0x08, 0x0B, 0x02, 0x03, 0x01, 0x62, 0x0C, 0x02, 0x07, 0x02, 0x01, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_wb_img[] PROGMEM = {
    0x88, 0x08, 0x0B, 0x02, 0x03, 0x01, 0x60, 0x0C, 0x02, 0x07, 0x02, 0x01, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bb_img[] PROGMEM = {
    0x00, 0x08, 0x0B, 0x02, 0x03, 0x01, 0x4A, 0x0C, 0x02, 0x07, 0x02, 0x01, 0x88, 0x01,
    0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// X3 AA LUTs: fast partial-style set tuned to preserve X3 polarity behavior.
const uint8_t lut_x3_vcom_fast[] PROGMEM = {
    0x00, 0x18, 0x18, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_ww_fast[] PROGMEM = {
    0x60, 0x18, 0x18, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bw_fast[] PROGMEM = {
    0x20, 0x18, 0x18, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_wb_fast[] PROGMEM = {
    0x10, 0x18, 0x18, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t lut_x3_bb_fast[] PROGMEM = {
    0x90, 0x18, 0x18, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void EInkDisplay::setDisplayDimensions(uint16_t width, uint16_t height) {
  displayWidth = width;
  displayHeight = height;
  displayWidthBytes = width / 8;
  bufferSize = displayWidthBytes * height;
  _x3Mode = false;
  _murphyM3Mode = false;
}

void EInkDisplay::setDisplayX3() {
  setDisplayDimensions(X3_DISPLAY_WIDTH, X3_DISPLAY_HEIGHT);
  _x3Mode = true;
}

void EInkDisplay::setDisplayMurphyM3() {
  // CrossPoint's renderer treats the display dimensions as the controller's
  // command orientation. The Murphy controller is 240x416, but the device is
  // held as 240x416 portrait glass with controller RAM rotated relative to the
  // UI. Expose a 416x240 framebuffer here so the existing renderer portrait
  // transform presents a 240x416 logical screen, then rotate only when writing
  // the Murphy plane to the UC8253.
  setDisplayDimensions(MURPHY_M3_FRAMEBUFFER_WIDTH, MURPHY_M3_FRAMEBUFFER_HEIGHT);
  _murphyM3Mode = true;
  memset(murphyM3PreviousFrame, 0xFF, sizeof(murphyM3PreviousFrame));
}

void EInkDisplay::skipInitialResync() {
  if (!_x3Mode) return;
  _x3InitialFullSyncsRemaining = 0;
  _x3RedRamSynced = true;
}

void EInkDisplay::requestResync(uint8_t settlePasses) {
  _x3ForceFullSyncNext = _x3Mode;
  _x3ForcedConditionPassesNext = _x3Mode ? settlePasses : 0;
}

EInkDisplay::EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy)
    : _sclk(sclk),
      _mosi(mosi),
      _cs(cs),
      _dc(dc),
      _rst(rst),
      _busy(busy),
      frameBuffer(nullptr),
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
      frameBufferActive(nullptr),
#endif
      customLutActive(false) {
  if (Serial) Serial.printf("[%lu] EInkDisplay: Constructor called\n", millis());
  if (Serial) Serial.printf("[%lu]   SCLK=%d, MOSI=%d, CS=%d, DC=%d, RST=%d, BUSY=%d\n", millis(), sclk, mosi, cs, dc, rst, busy);
}

void EInkDisplay::begin() {
  if (Serial) Serial.printf("[%lu] EInkDisplay: begin() called\n", millis());

  frameBuffer = frameBuffer0;
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  frameBufferActive = frameBuffer1;
#endif

  // Initialize to white
  memset(frameBuffer0, 0xFF, bufferSize);
  _x3RedRamSynced = false;
  _x3InitialFullSyncsRemaining = _x3Mode ? 2 : 0;
  _x3ForceFullSyncNext = false;
  _x3ForcedConditionPassesNext = 0;
  _x3GrayState = {};
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  if (Serial) Serial.printf("[%lu]   Static frame buffer (%lu bytes)\n", millis(), bufferSize);
#else
  memset(frameBuffer1, 0xFF, bufferSize);
  if (Serial) Serial.printf("[%lu]   Static frame buffers (2 x %lu bytes)\n", millis(), bufferSize);
#endif

  if (Serial) Serial.printf("[%lu]   Initializing e-ink display driver...\n", millis());

  if (BoardConfig::ACTIVE.display.powerEnable >= 0) {
    pinMode(BoardConfig::ACTIVE.display.powerEnable, OUTPUT);
    digitalWrite(BoardConfig::ACTIVE.display.powerEnable, HIGH);
    delay(20);
  }

  // Initialize SPI with custom pins. Murphy uses the vendor bit-banged write path
  // during bring-up, so do not attach the Arduino SPI peripheral to EPD pins.
  if (!_murphyM3Mode) {
    SPI.begin(_sclk, -1, _mosi, _cs);
  }
  const uint32_t spiHz = _murphyM3Mode ? 4000000 : (_x3Mode ? 10000000 : 40000000);
  spiSettings = SPISettings(spiHz, MSBFIRST, SPI_MODE0);
  if (Serial) {
    Serial.printf("[%lu]   %s at %lu Hz, Mode 0\n", millis(),
                  _murphyM3Mode ? "Bit-banged display bus configured" : "SPI initialized", spiHz);
  }

  // Setup GPIO pins
  if (_murphyM3Mode) {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    digitalWrite(_sclk, HIGH);
    digitalWrite(_mosi, LOW);
    pinMode(48, INPUT);
    logMurphyPinLevels("after power enables");
  }
  pinMode(_cs, OUTPUT);
  pinMode(_dc, OUTPUT);
  pinMode(_rst, OUTPUT);
  pinMode(_busy, INPUT);

  digitalWrite(_cs, HIGH);
  digitalWrite(_dc, HIGH);

  if (Serial) Serial.printf("[%lu]   GPIO pins configured\n", millis());

  // Reset display
  resetDisplay();

  // Initialize display controller
  initDisplayController();

  if (Serial) Serial.printf("[%lu]   E-ink display driver initialized\n", millis());
}

// ============================================================================
// Low-level display control methods
// ============================================================================

void EInkDisplay::resetDisplay() {
  if (Serial) Serial.printf("[%lu]   Resetting display...\n", millis());
  if (_murphyM3Mode) {
    digitalWrite(_rst, HIGH);
    delay(20);
    logMurphyPinLevels("before reset");
    digitalWrite(_rst, LOW);
    delay(20);
    logMurphyPinLevels("reset low");
    digitalWrite(_rst, HIGH);
    delay(200);
    logMurphyPinLevels("reset high");
    if (Serial) Serial.printf("[%lu]   Display reset complete, BUSY=%d\n", millis(), digitalRead(_busy));
    return;
  }

  digitalWrite(_rst, HIGH);
  delay(20);
  digitalWrite(_rst, LOW);
  delay(2);
  digitalWrite(_rst, HIGH);
  delay(20);
  if (Serial) Serial.printf("[%lu]   Display reset complete\n", millis());
  if (_x3Mode || _murphyM3Mode) {
    delay(50);
    return;
  }
}

void EInkDisplay::waitForRefresh(const char* comment) {
  unsigned long start = millis();
  if (!_x3Mode && !_murphyM3Mode) {
    while (digitalRead(_busy) == HIGH) {
      delay(1);
      if (millis() - start > 30000) break;
    }
  } else if (_murphyM3Mode) {
    constexpr unsigned long murphyBusyTimeoutMs = 1500;
    const int initialBusy = digitalRead(_busy);
    while (digitalRead(_busy) == LOW) {
      delay(1);
      if (millis() - start > murphyBusyTimeoutMs) break;
    }
    if (comment && Serial) {
      Serial.printf("[%lu]   Refresh done: %s (%lu ms, busy %d->%d%s)\n", millis(), comment, millis() - start,
                    initialBusy, digitalRead(_busy), millis() - start > murphyBusyTimeoutMs ? ", timeout" : "");
    }
    return;
  } else {
    bool sawLow = false;
    while (digitalRead(_busy) == HIGH) {
      delay(1);
      if (millis() - start > 1000) break;
    }
    if (digitalRead(_busy) == LOW) {
      sawLow = true;
      while (digitalRead(_busy) == LOW) {
        delay(1);
        if (millis() - start > 30000) break;
      }
    }
    if (!sawLow) return;
  }
  if (comment && Serial) Serial.printf("[%lu]   Refresh done: %s (%lu ms)\n", millis(), comment, millis() - start);
}

void EInkDisplay::sendCommand(uint8_t command) {
  if (_murphyM3Mode) {
    digitalWrite(_dc, LOW);
    digitalWrite(_cs, LOW);
    for (uint8_t bit = 0; bit < 8; bit++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_mosi, (command & 0x80) ? HIGH : LOW);
      digitalWrite(_sclk, HIGH);
      command <<= 1;
    }
    digitalWrite(_cs, HIGH);
    digitalWrite(_dc, HIGH);
    return;
  }

  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);  // Command mode
  digitalWrite(_cs, LOW);  // Select chip
  SPI.transfer(command);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendData(uint8_t data) {
  if (_murphyM3Mode) {
    digitalWrite(_dc, HIGH);
    digitalWrite(_cs, LOW);
    for (uint8_t bit = 0; bit < 8; bit++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_mosi, (data & 0x80) ? HIGH : LOW);
      digitalWrite(_sclk, HIGH);
      data <<= 1;
    }
    digitalWrite(_cs, HIGH);
    digitalWrite(_dc, HIGH);
    return;
  }

  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);  // Data mode
  digitalWrite(_cs, LOW);   // Select chip
  SPI.transfer(data);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendData(const uint8_t* data, uint16_t length) {
  if (_murphyM3Mode) {
    if (!data || length == 0) return;
    digitalWrite(_dc, HIGH);
    digitalWrite(_cs, LOW);
    for (uint16_t idx = 0; idx < length; idx++) {
      uint8_t value = data[idx];
      for (uint8_t bit = 0; bit < 8; bit++) {
        digitalWrite(_sclk, LOW);
        digitalWrite(_mosi, (value & 0x80) ? HIGH : LOW);
        digitalWrite(_sclk, HIGH);
        value <<= 1;
      }
    }
    digitalWrite(_cs, HIGH);
    digitalWrite(_dc, HIGH);
    return;
  }

  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);       // Data mode
  digitalWrite(_cs, LOW);        // Select chip
  SPI.writeBytes(data, length);  // Transfer all bytes
  digitalWrite(_cs, HIGH);       // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::waitWhileBusy(const char* comment) {
  unsigned long start = millis();
  if (!_x3Mode && !_murphyM3Mode) {
    while (digitalRead(_busy) == HIGH) {
      delay(1);
      if (millis() - start > 30000) break;
    }
  } else if (_murphyM3Mode) {
    constexpr unsigned long murphyBusyTimeoutMs = 1500;
    const int initialBusy = digitalRead(_busy);
    while (digitalRead(_busy) == LOW) {
      delay(1);
      if (millis() - start > murphyBusyTimeoutMs) break;
    }
    if (comment && Serial) {
      Serial.printf("[%lu]   Wait complete: %s (%lu ms, busy %d->%d%s)\n", millis(), comment, millis() - start,
                    initialBusy, digitalRead(_busy), millis() - start > murphyBusyTimeoutMs ? ", timeout" : "");
    }
    return;
  } else {
    bool sawLow = false;
    while (digitalRead(_busy) == HIGH) {
      delay(1);
      if (millis() - start > 1000) break;
    }
    if (digitalRead(_busy) == LOW) {
      sawLow = true;
      while (digitalRead(_busy) == LOW) {
        delay(1);
        if (millis() - start > 30000) break;
      }
    }
    if (!sawLow) return;
  }
  if (comment) {
    if (Serial) Serial.printf("[%lu]   Wait complete: %s (%lu ms)\n", millis(), comment, millis() - start);
  }
}

void EInkDisplay::writePlaneMurphyM3(uint8_t command, const uint8_t* data) {
  sendCommand(command);
  if (!data) return;

  uint8_t row[MURPHY_M3_CONTROLLER_WIDTH_BYTES];
  for (uint16_t controllerY = 0; controllerY < MURPHY_M3_CONTROLLER_HEIGHT; controllerY++) {
    memset(row, 0, sizeof(row));
    for (uint16_t controllerX = 0; controllerX < MURPHY_M3_CONTROLLER_WIDTH; controllerX++) {
      const uint16_t srcX = controllerY;
      const uint16_t srcY = static_cast<uint16_t>(MURPHY_M3_FRAMEBUFFER_HEIGHT - 1 - controllerX);
      const uint32_t srcByte = static_cast<uint32_t>(srcY) * displayWidthBytes + (srcX >> 3);
      const uint8_t srcMask = static_cast<uint8_t>(0x80 >> (srcX & 0x07));
      if (data[srcByte] & srcMask) {
        row[controllerX >> 3] |= static_cast<uint8_t>(0x80 >> (controllerX & 0x07));
      }
    }
    sendData(row, sizeof(row));
  }
}

void EInkDisplay::fillPlaneMurphyM3(uint8_t command, uint8_t fillByte) {
  uint8_t rowBuf[MURPHY_M3_CONTROLLER_WIDTH_BYTES];
  memset(rowBuf, fillByte, sizeof(rowBuf));
  sendCommand(command);
  if (_murphyM3Mode) {
    for (uint16_t y = 0; y < MURPHY_M3_CONTROLLER_HEIGHT; y++) {
      sendData(rowBuf, sizeof(rowBuf));
    }
    return;
  }

  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  for (uint16_t y = 0; y < displayHeight; y++) {
    SPI.writeBytes(rowBuf, displayWidthBytes);
  }
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

void EInkDisplay::triggerRefreshMurphyM3(bool turnOffScreen) {
  if (!isScreenOn) {
    sendCommand(CMD_UC8253_POWER_ON);
    waitForRefresh(" UC8253_PON");
    isScreenOn = true;
  }

  sendCommand(CMD_UC8253_DISPLAY_REFRESH);
  waitForRefresh(" UC8253_DRF");

  if (turnOffScreen) {
    sendCommand(CMD_UC8253_POWER_OFF);
    waitForRefresh(" UC8253_POF");
    isScreenOn = false;
  }
}

void EInkDisplay::loadMurphyM3DefaultLut() {
  sendCommand(0x20);
  sendData(MURPHY_LUT_20_DEFAULT, sizeof(MURPHY_LUT_20_DEFAULT));
  sendCommand(0x21);
  sendData(MURPHY_LUT_21_DEFAULT, sizeof(MURPHY_LUT_21_DEFAULT));
  sendCommand(0x22);
  sendData(MURPHY_LUT_22_DEFAULT, sizeof(MURPHY_LUT_22_DEFAULT));
  sendCommand(0x23);
  sendData(MURPHY_LUT_23_DEFAULT, sizeof(MURPHY_LUT_23_DEFAULT));
  sendCommand(0x24);
  sendData(MURPHY_LUT_24_DEFAULT, sizeof(MURPHY_LUT_24_DEFAULT));
}

void EInkDisplay::loadMurphyM3FastLut() {
  sendCommand(0x20);
  sendData(MURPHY_LUT_20_ALT, sizeof(MURPHY_LUT_20_ALT));
  sendCommand(0x21);
  sendData(MURPHY_LUT_21_ALT, sizeof(MURPHY_LUT_21_ALT));
  sendCommand(0x22);
  sendData(MURPHY_LUT_22_ALT, sizeof(MURPHY_LUT_22_ALT));
  sendCommand(0x23);
  sendData(MURPHY_LUT_23_ALT, sizeof(MURPHY_LUT_23_ALT));
  sendCommand(0x24);
  sendData(MURPHY_LUT_24_ALT, sizeof(MURPHY_LUT_24_ALT));
}

void EInkDisplay::initMurphyM3Controller() {
  if (Serial) Serial.printf("[%lu]   Initializing Murphy UC8253 controller with OEM sequence...\n", millis());

  const uint8_t powerSetting[] = {0x03, 0x10, 0x3F, 0x3B, 0x0D};
  sendCommand(CMD_UC8253_POWER_SETTING);
  sendData(powerSetting, sizeof(powerSetting));

  const uint8_t booster[] = {0xD7, 0xD7, 0x1F};
  sendCommand(CMD_UC8253_BOOSTER_SOFT_START);
  sendData(booster, sizeof(booster));

  sendCommand(CMD_UC8253_POWER_ON);
  waitWhileBusy(" UC8253_INIT_PON");
  isScreenOn = true;

  sendCommand(CMD_UC8253_PANEL_SETTING);
  sendData(0xFF);

  sendCommand(CMD_UC8253_PLL_CONTROL);
  sendData(0x09);

  const uint8_t resolution[] = {0xF0, 0x01, 0xA0};
  sendCommand(CMD_UC8253_RESOLUTION_SETTING);
  sendData(resolution, sizeof(resolution));

  sendCommand(CMD_UC8253_VCOM_DC_SETTING);
  sendData(0x0F);

  sendCommand(CMD_UC8253_VCOM_DATA_INTERVAL);
  sendData(0x97);

  waitWhileBusy(" UC8253_INIT");

  fillPlaneMurphyM3(CMD_UC8253_DTM1, 0xFF);
  fillPlaneMurphyM3(CMD_UC8253_DTM2, 0xFF);
}

void EInkDisplay::initDisplayController() {
  if (_murphyM3Mode) {
    initMurphyM3Controller();
    return;
  }

#ifndef X3_USE_X4_INIT
  if (_x3Mode) {
    sendCommand(0x00);
    sendData(0x3F);
    sendData(0x08);
    sendCommand(0x61);
    sendData(0x03);
    sendData(0x18);
    sendData(0x02);
    sendData(0x58);
    sendCommand(0x65);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);
    sendData(0x00);
    sendCommand(0x03);
    sendData(0x1D);
    sendCommand(0x01);
    sendData(0x07);
    sendData(0x17);
    sendData(0x3F);
    sendData(0x3F);
    sendData(0x17);
    sendCommand(0x82);
    sendData(0x1D);
    sendCommand(0x06);
    sendData(0x25);
    sendData(0x25);
    sendData(0x3C);
    sendData(0x37);
    sendCommand(0x30);
    sendData(0x09);
    sendCommand(0xE1);
    sendData(0x02);
    sendCommand(0x20);
    sendData(lut_x3_vcom_full, 42);
    sendCommand(0x21);
    sendData(lut_x3_ww_full, 42);
    sendCommand(0x22);
    sendData(lut_x3_bw_full, 42);
    sendCommand(0x23);
    sendData(lut_x3_wb_full, 42);
    sendCommand(0x24);
    sendData(lut_x3_bb_full, 42);
    isScreenOn = false;
    return;
  }
#endif

  if (Serial) Serial.printf("[%lu]   Initializing SSD1677 controller...\n", millis());

  const uint8_t TEMP_SENSOR_INTERNAL = 0x80;

  // Soft reset
  sendCommand(CMD_SOFT_RESET);
  waitWhileBusy(" CMD_SOFT_RESET");

  // Temperature sensor control (internal)
  sendCommand(CMD_TEMP_SENSOR_CONTROL);
  sendData(TEMP_SENSOR_INTERNAL);

  // Booster soft-start control (GDEQ0426T82 specific values)
  sendCommand(CMD_BOOSTER_SOFT_START);
  sendData(0xAE);
  sendData(0xC7);
  sendData(0xC3);
  sendData(0xC0);
  sendData(0x40);

  // Driver output control: set display height and scan direction
  sendCommand(CMD_DRIVER_OUTPUT_CONTROL);
  sendData((displayHeight - 1) % 256);
  sendData((displayHeight - 1) / 256);
  sendData(0x02);                // SM=1 (interlaced), TB=0

  // Border waveform control
  sendCommand(CMD_BORDER_WAVEFORM);
  sendData(0x01);

  // Set up full screen RAM area
  setRamArea(0, 0, displayWidth, displayHeight);

  if (Serial) Serial.printf("[%lu]   Clearing RAM buffers...\n", millis());
  sendCommand(CMD_AUTO_WRITE_BW_RAM);  // Auto write BW RAM
  sendData(0xF7);
  waitWhileBusy(" CMD_AUTO_WRITE_BW_RAM");

  sendCommand(CMD_AUTO_WRITE_RED_RAM);  // Auto write RED RAM
  sendData(0xF7);                       // Fill with white pattern
  waitWhileBusy(" CMD_AUTO_WRITE_RED_RAM");

  if (Serial) Serial.printf("[%lu]   SSD1677 controller initialized\n", millis());
}

void EInkDisplay::setRamArea(const uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  constexpr uint8_t DATA_ENTRY_X_INC_Y_DEC = 0x01;

  // Reverse Y coordinate (gates are reversed on this display)
  y = displayHeight - y - h;

  // Set data entry mode (X increment, Y decrement for reversed gates)
  sendCommand(CMD_DATA_ENTRY_MODE);
  sendData(DATA_ENTRY_X_INC_Y_DEC);

  // Set RAM X address range (start, end) - X is in PIXELS
  sendCommand(CMD_SET_RAM_X_RANGE);
  sendData(x % 256);            // start low byte
  sendData(x / 256);            // start high byte
  sendData((x + w - 1) % 256);  // end low byte
  sendData((x + w - 1) / 256);  // end high byte

  // Set RAM Y address range (start, end) - Y is in PIXELS
  sendCommand(CMD_SET_RAM_Y_RANGE);
  sendData((y + h - 1) % 256);  // start low byte
  sendData((y + h - 1) / 256);  // start high byte
  sendData(y % 256);            // end low byte
  sendData(y / 256);            // end high byte

  // Set RAM X address counter - X is in PIXELS
  sendCommand(CMD_SET_RAM_X_COUNTER);
  sendData(x % 256);  // low byte
  sendData(x / 256);  // high byte

  // Set RAM Y address counter - Y is in PIXELS
  sendCommand(CMD_SET_RAM_Y_COUNTER);
  sendData((y + h - 1) % 256);  // low byte
  sendData((y + h - 1) / 256);  // high byte
}

void EInkDisplay::clearScreen(const uint8_t color) const {
  memset(frameBuffer, color, bufferSize);
}

void EInkDisplay::drawImage(const uint8_t* imageData, const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h,
                            const bool fromProgmem) const {
  if (!frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  // Calculate bytes per line for the image
  const uint16_t imageWidthBytes = w / 8;

  // Copy image data to frame buffer
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t destY = y + row;
    if (destY >= displayHeight)
      break;

    const uint16_t destOffset = destY * displayWidthBytes + (x / 8);
    const uint16_t srcOffset = row * imageWidthBytes;

    for (uint16_t col = 0; col < imageWidthBytes; col++) {
      if ((x / 8 + col) >= displayWidthBytes)
        break;

      if (fromProgmem) {
        frameBuffer[destOffset + col] = pgm_read_byte(&imageData[srcOffset + col]);
      } else {
        frameBuffer[destOffset + col] = imageData[srcOffset + col];
      }
    }
  }

  if (Serial) Serial.printf("[%lu]   Image drawn to frame buffer\n", millis());
}

// Draws only black pixels from the image, leaves white pixels clear (unchanged in framebuffer)
void EInkDisplay::drawImageTransparent(const uint8_t* imageData, const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h,
                                     const bool fromProgmem) const {
  if (!frameBuffer) {
    Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  // Calculate bytes per line for the image
  const uint16_t imageWidthBytes = w / 8;

  // Copy only black pixels to frame buffer
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t destY = y + row;
    if (destY >= displayHeight)
      break;

    const uint16_t destOffset = destY * displayWidthBytes + (x / 8);
    const uint16_t srcOffset = row * imageWidthBytes;

    for (uint16_t col = 0; col < imageWidthBytes; col++) {
      if ((x / 8 + col) >= displayWidthBytes)
        break;

      uint8_t srcByte = fromProgmem ? pgm_read_byte(&imageData[srcOffset + col]) : imageData[srcOffset + col];
      frameBuffer[destOffset + col] &= srcByte;
    }
  }

  if (Serial) Serial.printf("[%lu]   Transparent image drawn to frame buffer\n", millis());
}

void EInkDisplay::writeRamBuffer(uint8_t ramBuffer, const uint8_t* data, uint32_t size) {
  const char* bufferName = (ramBuffer == CMD_WRITE_RAM_BW) ? "BW" : "RED";
  const unsigned long startTime = millis();
  if (Serial) Serial.printf("[%lu]   Writing frame buffer to %s RAM (%lu bytes)...\n", startTime, bufferName, size);

  sendCommand(ramBuffer);
  sendData(data, size);

  const unsigned long duration = millis() - startTime;
  if (Serial) Serial.printf("[%lu]   %s RAM write complete (%lu ms)\n", millis(), bufferName, duration);
}

void EInkDisplay::setFramebuffer(const uint8_t* bwBuffer) const {
  memcpy(frameBuffer, bwBuffer, bufferSize);
}

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
void EInkDisplay::swapBuffers() {
  uint8_t* temp = frameBuffer;
  frameBuffer = frameBufferActive;
  frameBufferActive = temp;
}
#endif

void EInkDisplay::grayscaleRevert() {
  if (_murphyM3Mode) {
    inGrayscaleMode = false;
    displayBuffer(FULL_REFRESH);
    return;
  }

  if (!inGrayscaleMode) {
    return;
  }

  inGrayscaleMode = false;

  // Load the revert LUT
  setCustomLUT(true, lut_grayscale_revert);
  refreshDisplay(FAST_REFRESH);
  setCustomLUT(false);
}

void EInkDisplay::copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer) {
  if (!lsbBuffer) {
    _x3GrayState.lsbValid = false;
    return;
  }

  if (_murphyM3Mode) {
    setFramebuffer(lsbBuffer);
    _x3GrayState.lsbValid = true;
    return;
  }

  if (_x3Mode) {
    // X3 single-pass AA: write LSB plane to old-data RAM.
    uint8_t row[128];
    auto sendMirroredPlane = [&](const uint8_t* plane) {
      for (uint16_t y = 0; y < displayHeight; y++) {
        const uint16_t srcY = static_cast<uint16_t>(displayHeight - 1 - y);
        const uint8_t* src = plane + static_cast<uint32_t>(srcY) * displayWidthBytes;
        for (uint16_t x = 0; x < displayWidthBytes; x++) {
          row[x] = src[x];
        }
        sendData(row, displayWidthBytes);
      }
    };

    sendCommand(0x10);
    sendMirroredPlane(lsbBuffer);
    _x3GrayState.lsbValid = true;
    return;
  }
  setRamArea(0, 0, displayWidth, displayHeight);
  writeRamBuffer(CMD_WRITE_RAM_BW, lsbBuffer, bufferSize);
}

void EInkDisplay::copyGrayscaleMsbBuffers(const uint8_t* msbBuffer) {
  if (!msbBuffer) {
    return;
  }

  if (_murphyM3Mode) {
    setFramebuffer(msbBuffer);
    return;
  }

  if (_x3Mode) {
    if (!_x3GrayState.lsbValid) {
      return;
    }

    uint8_t row[128];
    auto sendMirroredPlane = [&](const uint8_t* plane) {
      for (uint16_t y = 0; y < displayHeight; y++) {
        const uint16_t srcY = static_cast<uint16_t>(displayHeight - 1 - y);
        const uint8_t* src = plane + static_cast<uint32_t>(srcY) * displayWidthBytes;
        for (uint16_t x = 0; x < displayWidthBytes; x++) {
          row[x] = src[x];
        }
        sendData(row, displayWidthBytes);
      }
    };

    sendCommand(0x13);
    sendMirroredPlane(msbBuffer);
    return;
  }
  setRamArea(0, 0, displayWidth, displayHeight);
  writeRamBuffer(CMD_WRITE_RAM_RED, msbBuffer, bufferSize);
}

void EInkDisplay::copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  if (_murphyM3Mode) {
    setFramebuffer(msbBuffer ? msbBuffer : lsbBuffer);
    return;
  }

  if (_x3Mode) {
    copyGrayscaleLsbBuffers(lsbBuffer);
    copyGrayscaleMsbBuffers(msbBuffer);
    return;
  }
  setRamArea(0, 0, displayWidth, displayHeight);
  writeRamBuffer(CMD_WRITE_RAM_BW, lsbBuffer, bufferSize);
  writeRamBuffer(CMD_WRITE_RAM_RED, msbBuffer, bufferSize);
}

void EInkDisplay::writeGrayscalePlaneStrip(GrayPlane plane, const uint8_t* rows, uint16_t yStart, uint16_t numRows) {
  if (!rows || yStart >= displayHeight || numRows == 0) {
    return;
  }
  if (yStart + numRows > displayHeight) {
    numRows = displayHeight - yStart;
  }

  if (_murphyM3Mode) {
    for (uint16_t row = 0; row < numRows; row++) {
      memcpy(frameBuffer + static_cast<uint32_t>(yStart + row) * displayWidthBytes,
             rows + static_cast<uint32_t>(row) * displayWidthBytes, displayWidthBytes);
    }
    return;
  }

  if (_x3Mode) {
    const uint8_t cmd = (plane == GRAY_PLANE_LSB) ? 0x10 : 0x13;
    sendCommand(cmd);
    for (uint16_t row = 0; row < numRows; row++) {
      const uint16_t srcY = static_cast<uint16_t>(numRows - 1 - row);
      sendData(rows + static_cast<uint32_t>(srcY) * displayWidthBytes, displayWidthBytes);
    }
    _x3GrayState.lsbValid = _x3GrayState.lsbValid || plane == GRAY_PLANE_LSB;
    return;
  }

  const uint8_t command = (plane == GRAY_PLANE_LSB) ? CMD_WRITE_RAM_BW : CMD_WRITE_RAM_RED;
  setRamArea(0, yStart, displayWidth, numRows);
  writeRamBuffer(command, rows, static_cast<uint32_t>(displayWidthBytes) * numRows);
}

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
/**
 * In single buffer mode, this should be called with the previously written BW buffer
 * to reconstruct the RED buffer for proper differential fast refreshes following a
 * grayscale display.
 */
void EInkDisplay::cleanupGrayscaleBuffers(const uint8_t* bwBuffer) {
  if (_x3Mode) {
    if (!bwBuffer) {
      return;
    }

    uint8_t row[128];
    auto sendMirroredPlane = [&](const uint8_t* plane, bool invertBits) {
      for (uint16_t y = 0; y < displayHeight; y++) {
        const uint16_t srcY = static_cast<uint16_t>(displayHeight - 1 - y);
        const uint8_t* src = plane + static_cast<uint32_t>(srcY) * displayWidthBytes;
        for (uint16_t x = 0; x < displayWidthBytes; x++) {
          row[x] = invertBits ? static_cast<uint8_t>(~src[x]) : src[x];
        }
        sendData(row, displayWidthBytes);
      }
    };

    // Rebase both X3 planes from restored BW buffer so next differential update
    // compares from a coherent known state.
    sendCommand(0x13);
    sendMirroredPlane(bwBuffer, false);
    sendCommand(0x10);
    sendMirroredPlane(bwBuffer, false);

    _x3RedRamSynced = true;
    _x3ForceFullSyncNext = false;
    _x3ForcedConditionPassesNext = 0;
    return;
  }

  setRamArea(0, 0, displayWidth, displayHeight);
  writeRamBuffer(CMD_WRITE_RAM_RED, bwBuffer, bufferSize);
}
#endif

void EInkDisplay::displayBuffer(RefreshMode mode, const bool turnOffScreen) {
  if (!_x3Mode && !_murphyM3Mode && !isScreenOn && !turnOffScreen)
  {
    // Force half refresh if screen is off (non-X3 only)
    mode = HALF_REFRESH;
  }

  // If currently in grayscale mode, revert first to black/white
  if (inGrayscaleMode) {
    inGrayscaleMode = false;
    grayscaleRevert();
  }

  if (_murphyM3Mode) {
    if (Serial) {
      const uint32_t blackPixels = countMurphyBlackPixels(frameBuffer, bufferSize);
      const char* modeName = (mode == FULL_REFRESH) ? "full" : (mode == HALF_REFRESH) ? "half" : "fast";
      Serial.printf("[%lu]   UC8253_MURPHY_REFRESH requested=%s black_pixels=%lu/%lu single_pass=1 lut_each=%d\n",
                    millis(), modeName, blackPixels, bufferSize * 8UL, MURPHY_LOAD_OEM_LUT_EACH_REFRESH);
    }
#if MURPHY_LOAD_OEM_LUT_EACH_REFRESH
    // NOTE: the alt LUTs (loadMurphyM3FastLut) are OEM's partial-refresh
    // waveform — they only drive a single short phase and require the
    // 0x17/0xA5 partial-window trigger pair to actually flip pixels. Using
    // them as a full-refresh waveform causes incomplete updates where the
    // panel appears to flash but the new frame doesn't latch, and a second
    // refresh shows what should have been the previous frame. Always load
    // the default LUT until the partial-refresh trigger path is wired.
    loadMurphyM3DefaultLut();
#endif
    // OEM (FUN_42038cac) sends the current frame to both DTM1 and DTM2. The
    // default LUTs are destination-only: with (old=new) for every pixel only
    // LUTWW/LUTBB fire, and those waveforms fully drive a pixel to its target
    // value. The BW/WB LUTs are short partial-refresh kicks meant for the
    // 0x17/0xA5 path; using them on changed pixels (as a prev/new diff would)
    // leaves pixels half-flipped, so the panel flashes but doesn't latch and
    // the next refresh "completes" the prior frame. Mirror the OEM here.
    writePlaneMurphyM3(CMD_UC8253_DTM1, frameBuffer);
    writePlaneMurphyM3(CMD_UC8253_DTM2, frameBuffer);
    triggerRefreshMurphyM3(turnOffScreen);
    memcpy(murphyM3PreviousFrame, frameBuffer, MURPHY_M3_BUFFER_SIZE);
    return;
  }

  if (_x3Mode) {
    // X3 update policy: RED RAM (0x10) on the controller stores the previous
    // frame for differential updates, eliminating the 52 KB _x3PrevFrame
    // software buffer.  CMD04 re-powers the charge pump when needed.
    // On X3, treat HALF refresh as fast differential mode.
    // Reader uses HALF as a cadence hint, but forcing full here makes turns too slow.
    const bool fastMode = (mode != FULL_REFRESH);
    uint8_t row[128];
    auto sendCommandDataX3 = [&](uint8_t cmd, const uint8_t* data, uint16_t len) {
      SPI.beginTransaction(spiSettings);
      digitalWrite(_cs, LOW);
      digitalWrite(_dc, LOW);
      SPI.transfer(cmd);
      if (len > 0 && data != nullptr) {
        digitalWrite(_dc, HIGH);
        SPI.writeBytes(data, len);
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
    };
    auto sendCommandDataByteX3 = [&](uint8_t cmd, uint8_t d0, uint8_t d1) {
      const uint8_t d[2] = {d0, d1};
      sendCommandDataX3(cmd, d, 2);
    };
    auto sendMirroredPlane = [&](const uint8_t* plane, bool invertBits) {
      for (uint16_t y = 0; y < displayHeight; y++) {
        const uint16_t srcY = static_cast<uint16_t>(displayHeight - 1 - y);
        const uint8_t* src = plane + static_cast<uint32_t>(srcY) * displayWidthBytes;
        for (uint16_t x = 0; x < displayWidthBytes; x++) {
          row[x] = invertBits ? static_cast<uint8_t>(~src[x]) : src[x];
        }
        sendData(row, displayWidthBytes);
      }
    };

    const bool forcedFullSync = _x3ForceFullSyncNext;
    const bool doFullSync = !fastMode || !_x3RedRamSynced ||
                            _x3InitialFullSyncsRemaining > 0 || forcedFullSync;

    if (Serial) {
      Serial.printf("[%lu]   X3_OEM_%s\n", millis(), doFullSync ? "FULL" : "FAST");
    }
    _x3GrayState.lastBaseWasPartial = !doFullSync;

    if (doFullSync) {
      // Full sync: img LUTs, inverted data to both RAMs
      sendCommandDataX3(0x20, lut_x3_vcom_img, 42);
      sendCommandDataX3(0x21, lut_x3_ww_img, 42);
      sendCommandDataX3(0x22, lut_x3_bw_img, 42);
      sendCommandDataX3(0x23, lut_x3_wb_img, 42);
      sendCommandDataX3(0x24, lut_x3_bb_img, 42);

      sendCommand(0x13);
      sendMirroredPlane(frameBuffer, true);
      sendCommand(0x10);
      sendMirroredPlane(frameBuffer, true);

      sendCommandDataByteX3(0x50, 0xA9, 0x07);
    } else {
      // Fast differential: full LUTs, RED RAM (0x10) retains previous frame
      sendCommandDataX3(0x20, lut_x3_vcom_full, 42);
      sendCommandDataX3(0x21, lut_x3_ww_full, 42);
      sendCommandDataX3(0x22, lut_x3_bw_full, 42);
      sendCommandDataX3(0x23, lut_x3_wb_full, 42);
      sendCommandDataX3(0x24, lut_x3_bb_full, 42);

      // Write only new data to 0x13; controller diffs against 0x10
      sendCommand(0x13);
      sendMirroredPlane(frameBuffer, false);

      sendCommandDataByteX3(0x50, 0x29, 0x07);
    }

    if (!isScreenOn || doFullSync) {
      sendCommand(0x04);
      waitForRefresh(" X3_CMD04");
      isScreenOn = true;
    }

    if (Serial) Serial.printf("[%lu]   X3_OEM_TRIGGER=0x12\n", millis());
    sendCommand(0x12);
    waitForRefresh(" X3_CMD12");

    // Power off analog rails immediately after refresh if requested,
    // before RAM bookkeeping (which only needs SPI, not the charge pump).
    // This mirrors X4 behavior where power-off is part of the refresh cycle.
    if (turnOffScreen) {
      sendCommand(0x02);
      waitForRefresh(" X3_CMD02_POWEROFF");
      isScreenOn = false;
    }

    if (!fastMode) delay(200);

    // One-time light settle after the first major full-sync improves early
    // page-turn quality on X3 without paying the old 6-pass cost.
    uint8_t postConditionPasses = 0;
    if (doFullSync) {
      if (forcedFullSync) postConditionPasses = _x3ForcedConditionPassesNext;
      else if (_x3InitialFullSyncsRemaining == 1) postConditionPasses = 1;
    }

    if (postConditionPasses > 0) {
      const uint16_t xStart = 0;
      const uint16_t xEnd = static_cast<uint16_t>(displayWidth - 1);
      const uint16_t yStart = 0;
      const uint16_t yEnd = static_cast<uint16_t>(displayHeight - 1);
      const uint8_t w[9] = {
          static_cast<uint8_t>(xStart >> 8), static_cast<uint8_t>(xStart & 0xFF), static_cast<uint8_t>(xEnd >> 8),
          static_cast<uint8_t>(xEnd & 0xFF), static_cast<uint8_t>(yStart >> 8), static_cast<uint8_t>(yStart & 0xFF),
          static_cast<uint8_t>(yEnd >> 8), static_cast<uint8_t>(yEnd & 0xFF), 0x01};

      sendCommandDataX3(0x20, lut_x3_vcom_full, 42);
      sendCommandDataX3(0x21, lut_x3_ww_full, 42);
      sendCommandDataX3(0x22, lut_x3_bw_full, 42);
      sendCommandDataX3(0x23, lut_x3_wb_full, 42);
      sendCommandDataX3(0x24, lut_x3_bb_full, 42);
      sendCommandDataByteX3(0x50, 0x29, 0x07);

      for (uint8_t i = 0; i < postConditionPasses; i++) {
        if (Serial) Serial.printf("[%lu]   X3_OEM_COND %u/%u\n", millis(), static_cast<unsigned>(i + 1), static_cast<unsigned>(postConditionPasses));
        sendCommand(0x91);
        sendCommandDataX3(0x90, w, 9);
        sendCommand(0x13);
        sendMirroredPlane(frameBuffer, false);
        sendCommand(0x92);
        if (!isScreenOn) {
          sendCommand(0x04);
          waitForRefresh(" X3_CMD04");
          isScreenOn = true;
        }
        if (Serial) Serial.printf("[%lu]   X3_OEM_TRIGGER=0x12(cond)\n", millis());
        sendCommand(0x12);
        waitForRefresh(" X3_CMD12(cond)");
      }
    }

    // Sync RED RAM (0x10) with non-inverted current frame for next fast diff.
    // This is a controller memory write — doesn't need the charge pump.
    sendCommand(0x10);
    sendMirroredPlane(frameBuffer, false);
    _x3RedRamSynced = true;

    if (doFullSync && _x3InitialFullSyncsRemaining > 0) {
      _x3InitialFullSyncsRemaining--;
    }
    _x3ForceFullSyncNext = false;
    _x3ForcedConditionPassesNext = 0;
    return;
  }

  // Set up full screen RAM area
  setRamArea(0, 0, displayWidth, displayHeight);

  if (mode != FAST_REFRESH) {
    // For full refresh, write to both buffers before refresh
    writeRamBuffer(CMD_WRITE_RAM_BW, frameBuffer, bufferSize);
    writeRamBuffer(CMD_WRITE_RAM_RED, frameBuffer, bufferSize);
  } else {
    // For fast refresh, write to BW buffer only
    writeRamBuffer(CMD_WRITE_RAM_BW, frameBuffer, bufferSize);
    // In single buffer mode, the RED RAM should already contain the previous frame
    // In dual buffer mode, we write back frameBufferActive which is the last frame
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    writeRamBuffer(CMD_WRITE_RAM_RED, frameBufferActive, bufferSize);
#endif
  }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  swapBuffers();
#endif

  // Refresh the display
  refreshDisplay(mode, turnOffScreen);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // In single buffer mode always sync RED RAM after refresh to prepare for next fast refresh
  // This ensures RED contains the currently displayed frame for differential comparison
  setRamArea(0, 0, displayWidth, displayHeight);
  writeRamBuffer(CMD_WRITE_RAM_RED, frameBuffer, bufferSize);
#endif
}

// EXPERIMENTAL: Windowed update support
// Displays only a rectangular region of the frame buffer, preserving the rest of the screen.
// Requirements: x and w must be byte-aligned (multiples of 8 pixels)
void EInkDisplay::displayWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const bool turnOffScreen) {
  if (Serial) Serial.printf("[%lu]   Displaying window at (%d,%d) size (%dx%d)\n", millis(), x, y, w, h);

  // Validate bounds
  if (x + w > displayWidth || y + h > displayHeight) {
    if (Serial) Serial.printf("[%lu]   ERROR: Window bounds exceed display dimensions!\n", millis());
    return;
  }

  // Validate byte alignment
  if (x % 8 != 0 || w % 8 != 0) {
    if (Serial) Serial.printf("[%lu]   ERROR: Window x and width must be byte-aligned (multiples of 8)!\n", millis());
    return;
  }

  if (!frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  if (_murphyM3Mode) {
    displayBuffer(FAST_REFRESH, turnOffScreen);
    return;
  }

  // displayWindow is not supported while the rest of the screen has grayscale content, revert it
  if (inGrayscaleMode) {
    inGrayscaleMode = false;
    grayscaleRevert();
  }

  // Calculate window buffer size
  const uint16_t windowWidthBytes = w / 8;
  const uint32_t windowBufferSize = windowWidthBytes * h;

  if (Serial) Serial.printf("[%lu]   Window buffer size: %lu bytes (%d x %d pixels)\n", millis(), windowBufferSize, w, h);

  // Allocate temporary buffer on stack
  std::vector<uint8_t> windowBuffer(windowBufferSize);

  // Extract window region from frame buffer
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t srcY = y + row;
    const uint16_t srcOffset = srcY * displayWidthBytes + (x / 8);
    const uint16_t dstOffset = row * windowWidthBytes;
    memcpy(&windowBuffer[dstOffset], &frameBuffer[srcOffset], windowWidthBytes);
  }

  // Configure RAM area for window
  setRamArea(x, y, w, h);

  // Write to BW RAM (current frame)
  writeRamBuffer(CMD_WRITE_RAM_BW, windowBuffer.data(), windowBufferSize);

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // Dual buffer: Extract window from frameBufferActive (previous frame)
  std::vector<uint8_t> previousWindowBuffer(windowBufferSize);
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t srcY = y + row;
    const uint16_t srcOffset = srcY * displayWidthBytes + (x / 8);
    const uint16_t dstOffset = row * windowWidthBytes;
    memcpy(&previousWindowBuffer[dstOffset], &frameBufferActive[srcOffset], windowWidthBytes);
  }
  writeRamBuffer(CMD_WRITE_RAM_RED, previousWindowBuffer.data(), windowBufferSize);
#endif

  // Perform fast refresh
  refreshDisplay(FAST_REFRESH, turnOffScreen);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // Post-refresh: Sync RED RAM with current window (for next fast refresh)
  setRamArea(x, y, w, h);
  writeRamBuffer(CMD_WRITE_RAM_RED, windowBuffer.data(), windowBufferSize);
#endif

  if (Serial) Serial.printf("[%lu]   Window display complete\n", millis());
}

void EInkDisplay::displayGrayBuffer(const bool turnOffScreen, const unsigned char* lut, const bool factoryMode) {
  (void)lut;
  (void)factoryMode;
  if (_murphyM3Mode) {
    displayBuffer(FULL_REFRESH, turnOffScreen);
    return;
  }

  if (_x3Mode) {
    // X3 AA pipeline: LSB->0x10 + MSB->0x13, trigger 0x12 with X3 LUT bank.
    drawGrayscale = false;
    inGrayscaleMode = false;

    if (!_x3GrayState.lsbValid) {
      return;
    }

    auto sendCommandDataX3 = [&](uint8_t cmd, const uint8_t* data, uint16_t len) {
      SPI.beginTransaction(spiSettings);
      digitalWrite(_cs, LOW);
      digitalWrite(_dc, LOW);
      SPI.transfer(cmd);
      if (len > 0 && data != nullptr) {
        digitalWrite(_dc, HIGH);
        SPI.writeBytes(data, len);
      }
      digitalWrite(_cs, HIGH);
      SPI.endTransaction();
    };
    auto sendCommandDataByteX3 = [&](uint8_t cmd, uint8_t d0, uint8_t d1) {
      const uint8_t d[2] = {d0, d1};
      sendCommandDataX3(cmd, d, 2);
    };
    uint8_t row[128];
    auto sendMirroredPlane = [&](const uint8_t* plane, bool invertBits) {
      for (uint16_t y = 0; y < displayHeight; y++) {
        const uint16_t srcY = static_cast<uint16_t>(displayHeight - 1 - y);
        const uint8_t* src = plane + static_cast<uint32_t>(srcY) * displayWidthBytes;
        for (uint16_t x = 0; x < displayWidthBytes; x++) {
          row[x] = invertBits ? static_cast<uint8_t>(~src[x]) : src[x];
        }
        sendData(row, displayWidthBytes);
      }
    };

    const uint8_t* vcom = lut_x3_vcom_gray;
    const uint8_t* ww = lut_x3_ww_gray;
    const uint8_t* bw = lut_x3_bw_gray;
    const uint8_t* wb = lut_x3_wb_gray;
    const uint8_t* bb = lut_x3_bb_gray;
    uint8_t dataInterval0 = 0x29;
    uint8_t dataInterval1 = 0x07;
    if (Serial) Serial.printf("[%lu]   X3_GRAY_MODE=gray_tuned\n", millis());
    sendCommandDataX3(0x20, vcom, 42);
    sendCommandDataX3(0x21, ww, 42);
    sendCommandDataX3(0x22, bw, 42);
    sendCommandDataX3(0x23, wb, 42);
    sendCommandDataX3(0x24, bb, 42);
    sendCommandDataByteX3(0x50, dataInterval0, dataInterval1);

    if (!isScreenOn) {
      sendCommand(0x04);
      waitForRefresh(" X3_CMD04(gray)");
      isScreenOn = true;
    }

    sendCommand(0x12);
    waitForRefresh(" X3_CMD12(gray)");

    if (turnOffScreen) {
      sendCommand(0x02);
      waitForRefresh(" X3_CMD02_POWEROFF(gray)");
      isScreenOn = false;
    }

    // RAM baseline is re-established from restored BW buffer by
    // cleanupGrayscaleBuffers() after this function returns.
    _x3RedRamSynced = false;
    _x3ForceFullSyncNext = false;
    _x3ForcedConditionPassesNext = 0;

    _x3GrayState.lsbValid = false;
    return;
  }

  drawGrayscale = false;
  inGrayscaleMode = true;

  // activate the custom LUT for grayscale rendering and refresh
  setCustomLUT(true, lut_grayscale);
  refreshDisplay(FAST_REFRESH, turnOffScreen);
  setCustomLUT(false);
}

void EInkDisplay::refreshDisplay(const RefreshMode mode, const bool turnOffScreen) {
  if (_murphyM3Mode) {
    triggerRefreshMurphyM3(turnOffScreen);
    return;
  }

  if (_x3Mode) {
    displayBuffer(mode, turnOffScreen);
    return;
  }

  // Configure Display Update Control 1
  sendCommand(CMD_DISPLAY_UPDATE_CTRL1);
  sendData((mode == FAST_REFRESH) ? CTRL1_NORMAL : CTRL1_BYPASS_RED);  // Configure buffer comparison mode

  // best guess at display mode bits:
  // bit | hex | name                    | effect
  // ----+-----+--------------------------+-------------------------------------------
  // 7   | 80  | CLOCK_ON                | Start internal oscillator
  // 6   | 40  | ANALOG_ON               | Enable analog power rails (VGH/VGL drivers)
  // 5   | 20  | TEMP_LOAD               | Load temperature (internal or I2C)
  // 4   | 10  | LUT_LOAD                | Load waveform LUT
  // 3   | 08  | MODE_SELECT             | Mode 1/2
  // 2   | 04  | DISPLAY_START           | Run display
  // 1   | 02  | ANALOG_OFF_PHASE        | Shutdown step 1 (undocumented)
  // 0   | 01  | CLOCK_OFF               | Disable internal oscillator

  // Select appropriate display mode based on refresh type
  uint8_t displayMode = 0x00;

  // Enable counter and analog if not already on
  if (!isScreenOn) {
    isScreenOn = true;
    displayMode |= 0xC0;  // Set CLOCK_ON and ANALOG_ON bits
  }

  // Turn off screen if requested
  if (turnOffScreen) {
    isScreenOn = false;
    displayMode |= 0x03;  // Set ANALOG_OFF_PHASE and CLOCK_OFF bits
  }

  if (mode == FULL_REFRESH) {
    displayMode |= 0x34;
  } else if (mode == HALF_REFRESH) {
    // Write high temp to the register for a faster refresh
    sendCommand(CMD_WRITE_TEMP);
    sendData(0x5A);
    displayMode |= 0xD4;
  } else {  // FAST_REFRESH
    displayMode |= customLutActive ? 0x0C : 0x1C;
  }

  // Power on and refresh display
  const char* refreshType = (mode == FULL_REFRESH) ? "full" : (mode == HALF_REFRESH) ? "half" : "fast";
  if (Serial) Serial.printf("[%lu]   Powering on display 0x%02X (%s refresh)...\n", millis(), displayMode, refreshType);
  sendCommand(CMD_DISPLAY_UPDATE_CTRL2);
  sendData(displayMode);

  sendCommand(CMD_MASTER_ACTIVATION);

  // Wait for display to finish updating
  if (Serial) Serial.printf("[%lu]   Waiting for display refresh...\n", millis());
  waitWhileBusy(refreshType);
}

void EInkDisplay::setCustomLUT(const bool enabled, const unsigned char* lutData) {
  if (_murphyM3Mode) {
    customLutActive = false;
    return;
  }

  if (enabled) {
    if (Serial) Serial.printf("[%lu]   Loading custom LUT...\n", millis());

    // Load custom LUT (first 105 bytes: VS + TP/RP + frame rate)
    sendCommand(CMD_WRITE_LUT);
    for (uint16_t i = 0; i < 105; i++) {
      sendData(pgm_read_byte(&lutData[i]));
    }

    // Set voltage values from bytes 105-109
    sendCommand(CMD_GATE_VOLTAGE);  // VGH
    sendData(pgm_read_byte(&lutData[105]));

    sendCommand(CMD_SOURCE_VOLTAGE);         // VSH1, VSH2, VSL
    sendData(pgm_read_byte(&lutData[106]));  // VSH1
    sendData(pgm_read_byte(&lutData[107]));  // VSH2
    sendData(pgm_read_byte(&lutData[108]));  // VSL

    sendCommand(CMD_WRITE_VCOM);  // VCOM
    sendData(pgm_read_byte(&lutData[109]));

    customLutActive = true;
    if (Serial) Serial.printf("[%lu]   Custom LUT loaded\n", millis());
  } else {
    customLutActive = false;
    if (Serial) Serial.printf("[%lu]   Custom LUT disabled\n", millis());
  }
}

void EInkDisplay::deepSleep() {
  if (Serial) Serial.printf("[%lu]   Preparing display for deep sleep...\n", millis());

  if (_murphyM3Mode) {
    sendCommand(CMD_UC8253_POWER_OFF);
    waitWhileBusy(" UC8253_POF");
    sendCommand(CMD_UC8253_DEEP_SLEEP);
    sendData(0xA5);
    isScreenOn = false;
    if (BoardConfig::ACTIVE.display.powerEnable >= 0) {
      digitalWrite(BoardConfig::ACTIVE.display.powerEnable, LOW);
    }
    return;
  }

  // First, power down the display properly
  // This shuts down the analog power rails and clock
  if (isScreenOn) {
    sendCommand(CMD_DISPLAY_UPDATE_CTRL1);
    sendData(CTRL1_BYPASS_RED);  // Normal mode

    sendCommand(CMD_DISPLAY_UPDATE_CTRL2);
    sendData(0x03);  // Set ANALOG_OFF_PHASE (bit 1) and CLOCK_OFF (bit 0)

    sendCommand(CMD_MASTER_ACTIVATION);

    // Wait for the power-down sequence to complete
    waitWhileBusy(" display power-down");

    isScreenOn = false;
  }

  // Now enter deep sleep mode
  if (Serial) Serial.printf("[%lu]   Entering deep sleep mode...\n", millis());
  sendCommand(CMD_DEEP_SLEEP);
  sendData(0x01);  // Enter deep sleep
}

void EInkDisplay::saveFrameBufferAsPBM(const char* filename) {
#ifndef ARDUINO
  const uint8_t* buffer = getFrameBuffer();

  std::ofstream file(filename, std::ios::binary);
  if (!file) {
    if (Serial) Serial.printf("Failed to open %s for writing\n", filename);
    return;
  }

  // Rotate the image 90 degrees counterclockwise when saving
  // Original buffer: 800x480 (landscape)
  // Output image: 480x800 (portrait)
  const int DISPLAY_WIDTH_LOCAL = DISPLAY_WIDTH;    // 800
  const int DISPLAY_HEIGHT_LOCAL = DISPLAY_HEIGHT;  // 480
  const int DISPLAY_WIDTH_BYTES_LOCAL = DISPLAY_WIDTH_LOCAL / 8;

  file << "P4\n";  // Binary PBM
  file << DISPLAY_HEIGHT_LOCAL << " " << DISPLAY_WIDTH_LOCAL << "\n";

  // Create rotated buffer
  std::vector<uint8_t> rotatedBuffer((DISPLAY_HEIGHT_LOCAL / 8) * DISPLAY_WIDTH_LOCAL, 0);

  for (int outY = 0; outY < DISPLAY_WIDTH_LOCAL; outY++) {
    for (int outX = 0; outX < DISPLAY_HEIGHT_LOCAL; outX++) {
      int inX = outY;
      int inY = DISPLAY_HEIGHT_LOCAL - 1 - outX;

      int inByteIndex = inY * DISPLAY_WIDTH_BYTES_LOCAL + (inX / 8);
      int inBitPosition = 7 - (inX % 8);
      bool isWhite = (buffer[inByteIndex] >> inBitPosition) & 1;

      int outByteIndex = outY * (DISPLAY_HEIGHT_LOCAL / 8) + (outX / 8);
      int outBitPosition = 7 - (outX % 8);
      if (!isWhite) {  // Invert: e-ink white=1 -> PBM black=1
        rotatedBuffer[outByteIndex] |= (1 << outBitPosition);
      }
    }
  }

  file.write(reinterpret_cast<const char*>(rotatedBuffer.data()), rotatedBuffer.size());
  file.close();
  if (Serial) Serial.printf("Saved framebuffer to %s\n", filename);
#else
  (void)filename;
  if (Serial) Serial.println("saveFrameBufferAsPBM is not supported on Arduino builds.");
#endif
}
