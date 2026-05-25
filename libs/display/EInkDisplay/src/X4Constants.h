#pragma once

#include <cstdint>

// SSD1677 command opcodes (X4 controller). Some opcodes overlap with
// the UC81xx (X3) controller but mean different things; the bare
// CMD_ prefix marks the X4-only code paths that use them.
//
// Internal SDK header. Consumed by X4Panel.cpp and by the X4-specific
// helpers still living on EInkDisplay (setRamArea, writeRamBuffer).
// Not exported.

// Initialization and reset
constexpr uint8_t CMD_SOFT_RESET = 0x12;             // Soft reset
constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x0C;     // Booster soft-start control
constexpr uint8_t CMD_DRIVER_OUTPUT_CONTROL = 0x01;  // Driver output control
constexpr uint8_t CMD_BORDER_WAVEFORM = 0x3C;        // Border waveform control
constexpr uint8_t CMD_TEMP_SENSOR_CONTROL = 0x18;    // Temperature sensor control

// RAM and buffer management
constexpr uint8_t CMD_DATA_ENTRY_MODE = 0x11;     // Data entry mode
constexpr uint8_t CMD_SET_RAM_X_RANGE = 0x44;     // Set RAM X address range
constexpr uint8_t CMD_SET_RAM_Y_RANGE = 0x45;     // Set RAM Y address range
constexpr uint8_t CMD_SET_RAM_X_COUNTER = 0x4E;   // Set RAM X address counter
constexpr uint8_t CMD_SET_RAM_Y_COUNTER = 0x4F;   // Set RAM Y address counter
constexpr uint8_t CMD_WRITE_RAM_BW = 0x24;        // Write to BW RAM (current frame)
constexpr uint8_t CMD_WRITE_RAM_RED = 0x26;       // Write to RED RAM (used for fast refresh)
constexpr uint8_t CMD_AUTO_WRITE_BW_RAM = 0x46;   // Auto write BW RAM
constexpr uint8_t CMD_AUTO_WRITE_RED_RAM = 0x47;  // Auto write RED RAM

// Display update and refresh
constexpr uint8_t CMD_DISPLAY_UPDATE_CTRL1 = 0x21;  // Display update control 1
constexpr uint8_t CMD_DISPLAY_UPDATE_CTRL2 = 0x22;  // Display update control 2
constexpr uint8_t CMD_MASTER_ACTIVATION = 0x20;     // Master activation
constexpr uint8_t CTRL1_NORMAL = 0x00;              // Normal mode - compare RED vs BW for partial
constexpr uint8_t CTRL1_BYPASS_RED = 0x40;          // Bypass RED RAM (treat as 0) - for full refresh

// LUT and voltage settings
constexpr uint8_t CMD_WRITE_LUT = 0x32;       // Write LUT
constexpr uint8_t CMD_GATE_VOLTAGE = 0x03;    // Gate voltage
constexpr uint8_t CMD_SOURCE_VOLTAGE = 0x04;  // Source voltage
constexpr uint8_t CMD_WRITE_VCOM = 0x2C;      // Write VCOM
constexpr uint8_t CMD_WRITE_TEMP = 0x1A;      // Write temperature

// Power management
constexpr uint8_t CMD_DEEP_SLEEP = 0x10;  // Deep sleep
