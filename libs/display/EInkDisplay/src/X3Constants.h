#pragma once

#include <cstdint>

// UC81xx-class command opcodes (X3 controller). Opcodes overlap with
// SSD1677 (X4) but mean different things; the CMD_X3_ prefix marks
// the X3-only code paths that use them.
//
// Internal SDK header. Consumed by X3Panel.cpp; not exported.

// Initialization
constexpr uint8_t CMD_X3_PANEL_SETTING = 0x00;       // PSR
constexpr uint8_t CMD_X3_POWER_SETTING = 0x01;       // PWR
constexpr uint8_t CMD_X3_POWER_OFF = 0x02;           // POF
constexpr uint8_t CMD_X3_POWER_OFF_SEQ = 0x03;       // PFS
constexpr uint8_t CMD_X3_POWER_ON = 0x04;            // PON
constexpr uint8_t CMD_X3_BOOSTER_SOFT_START = 0x06;  // BTST

// RAM data transfer
constexpr uint8_t CMD_X3_DTM1 = 0x10;       // Display Start Transmission 1 ("old" RAM plane)
constexpr uint8_t CMD_X3_DATA_STOP = 0x11;  // DSP — commit the preceding DTMx data stream
constexpr uint8_t CMD_X3_DTM2 = 0x13;       // Display Start Transmission 2 ("new" RAM plane)

// Refresh control
constexpr uint8_t CMD_X3_DISPLAY_REFRESH = 0x12;  // DRF — trigger refresh, implicitly closes DTM2

// LUT register bank
constexpr uint8_t CMD_X3_LUT_VCOM = 0x20;  // LUTC
constexpr uint8_t CMD_X3_LUT_WW = 0x21;    // LUTWW
constexpr uint8_t CMD_X3_LUT_BW = 0x22;    // LUTBW
constexpr uint8_t CMD_X3_LUT_WB = 0x23;    // LUTWB
constexpr uint8_t CMD_X3_LUT_BB = 0x24;    // LUTBB

// Configuration
constexpr uint8_t CMD_X3_PLL_CONTROL = 0x30;         // PLL
constexpr uint8_t CMD_X3_VCOM_DATA_INTERVAL = 0x50;  // CDI — VCOM and data interval setting (mode select)
constexpr uint8_t CMD_X3_RESOLUTION = 0x61;          // TRES
constexpr uint8_t CMD_X3_GATE_SOURCE_START = 0x65;   // GSST
constexpr uint8_t CMD_X3_VCOM_DC = 0x82;             // VDCS
constexpr uint8_t CMD_X3_LV_SELECTION = 0xE1;        // Source LV / FT_GS selection

// Partial update window
constexpr uint8_t CMD_X3_PARTIAL_WINDOW = 0x90;  // PTL — set partial window coords
constexpr uint8_t CMD_X3_PARTIAL_IN = 0x91;      // PTIN — enter partial mode
constexpr uint8_t CMD_X3_PARTIAL_OUT = 0x92;     // PTOUT — exit partial mode
