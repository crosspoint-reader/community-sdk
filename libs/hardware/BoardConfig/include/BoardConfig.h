#pragma once

#include <Arduino.h>

namespace BoardConfig {

enum class Board : uint8_t { XteinkX4, MurphyM3 };
enum class DisplayController : uint8_t { SSD1677, UC8253 };
enum class InputStyle : uint8_t { XteinkAdcLadder, DigitalFiveKey };
enum class TouchController : uint8_t { None, MurphyChsc6x, Gt911 };

struct DisplayPins {
  int8_t sclk;
  int8_t mosi;
  int8_t cs;
  int8_t dc;
  int8_t rst;
  int8_t busy;
  int8_t powerEnable;
};

struct SdPins {
  int8_t sclk;
  int8_t miso;
  int8_t mosi;
  int8_t cs;
  int8_t powerEnable;
  bool separateSpi;
};

struct InputPins {
  int8_t back;
  int8_t confirm;
  int8_t left;
  int8_t right;
  int8_t up;
  int8_t down;
  int8_t power;
};

struct FrontlightConfig {
  int8_t pin;
  uint32_t pwmFrequencyHz;
  uint8_t pwmResolutionBits;
  bool activeHigh;
};

struct TouchConfig {
  int8_t irq;
  int8_t reset;
  int8_t sda;
  int8_t scl;
  uint8_t i2cAddress;
  TouchController controller;
  uint16_t rawXMin;
  uint16_t rawXMax;
  uint16_t rawYMin;
  uint16_t rawYMax;
  bool irqActiveLow;
  bool synthesizeConfirmButton;
};

struct BoardProfile {
  Board board;
  const char* name;
  DisplayController displayController;
  InputStyle inputStyle;
  uint16_t displayWidth;
  uint16_t displayHeight;
  DisplayPins display;
  SdPins sd;
  InputPins input;
  FrontlightConfig frontlight;
  TouchConfig touch;
  int8_t batteryAdc;
  int8_t usbDetect;
  bool hasTouch;
  bool hasAudioJack;
  bool externalRtcKnown;
};

constexpr int8_t PIN_UNASSIGNED = -1;

constexpr BoardProfile XTEINK_X4 = {Board::XteinkX4,
                                    "xteink_x4",
                                    DisplayController::SSD1677,
                                    InputStyle::XteinkAdcLadder,
                                    800,
                                    480,
                                    {8, 10, 21, 4, 5, 6, PIN_UNASSIGNED},
                                    {PIN_UNASSIGNED, 7, PIN_UNASSIGNED, 12, PIN_UNASSIGNED, false},
                                    {0, 1, 2, 3, 4, 5, 3},
                                    {PIN_UNASSIGNED, 0, 0, true},
                                    {PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED, 0,
                                     TouchController::None, 0, 0, 0, 0, true, false},
                                    0,
                                    20,
                                    false,
                                    false,
                                    false};

constexpr BoardProfile MURPHY_M3 = {Board::MurphyM3,
                                    "murphy_m3",
                                    DisplayController::UC8253,
                                    InputStyle::DigitalFiveKey,
                                    416,
                                    240,
                                    {4, 3, 5, 6, 7, 8, PIN_UNASSIGNED},
                                    // Temporary legacy SPI fields. OEM SD uses 4-bit SD_MMC:
                                    // CLK=16 CMD=17 D0=15 D1=14 D2=21 D3=18.
                                    {39, 13, 40, 10, PIN_UNASSIGNED, true},
                                    // Confirmed Murphy M3 buttons are active-low:
                                    // top=GPIO1/up, middle=GPIO2/down, bottom=GPIO0/confirm.
                                    // GPIO0 is also treated as power so a long bottom-button hold sleeps.
                                    {PIN_UNASSIGNED, 0, PIN_UNASSIGNED, PIN_UNASSIGNED, 1, 2, 0},
                                    {48, 25000, 10, true},
                                    // CHSC6x-style touch: IRQ=GPIO44 active-low, I2C SDA=13/SCL=12, addr=0x2e.
                                    // Raw axes are calibrated into display logical coordinates by InputManager.
                                    {44, 45, 13, 12, 0x2e, TouchController::MurphyChsc6x, 24, 224, 24, 392, true,
                                     false},
                                    PIN_UNASSIGNED,
                                    PIN_UNASSIGNED,
                                    true,
                                    true,
                                    false};

#if defined(CROSSPOINT_BOARD_MURPHY_M3) || defined(BOARD_MURPHY_M3) || defined(BOARD_CROWPANEL_37_S3)
constexpr BoardProfile ACTIVE = MURPHY_M3;
#else
constexpr BoardProfile ACTIVE = XTEINK_X4;
#endif

constexpr bool isMurphyM3() { return ACTIVE.board == Board::MurphyM3; }
constexpr bool hasPwmFrontlight() { return ACTIVE.frontlight.pin >= 0; }

}  // namespace BoardConfig
