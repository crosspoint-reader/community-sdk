#pragma once

#include <Arduino.h>

namespace BoardConfig {

enum class Board : uint8_t { XteinkX4, M5StackPaperColor };
enum class InputStyle : uint8_t { XteinkAdcLadder, DigitalButtons, DigitalConfirmBackHold };

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

struct BoardProfile {
  Board board;
  const char* name;
  InputStyle inputStyle;
  uint16_t displayWidth;
  uint16_t displayHeight;
  DisplayPins display;
  SdPins sd;
  InputPins input;
  int8_t batteryAdc;
  int8_t usbDetect;
};

constexpr int8_t PIN_UNASSIGNED = -1;

constexpr BoardProfile XTEINK_X4 = {Board::XteinkX4,
                                    "xteink_x4",
                                    InputStyle::XteinkAdcLadder,
                                    800,
                                    480,
                                    {8, 10, 21, 4, 5, 6, PIN_UNASSIGNED},
                                    {PIN_UNASSIGNED, 7, PIN_UNASSIGNED, 12, PIN_UNASSIGNED, false},
                                    {0, 1, 2, 3, 4, 5, 3},
                                    0,
                                    20};

constexpr BoardProfile M5STACK_PAPER_COLOR = {Board::M5StackPaperColor,
                                              "m5stack_papercolor",
                                              InputStyle::DigitalConfirmBackHold,
                                              400,
                                              600,
                                              {15, 13, 44, 43, 12, 11, PIN_UNASSIGNED},
                                              {15, 14, 13, 47, PIN_UNASSIGNED, false},
                                              {1, 1, PIN_UNASSIGNED, PIN_UNASSIGNED, 10, 9, 1},
                                              PIN_UNASSIGNED,
                                              PIN_UNASSIGNED};

#if defined(CROSSPOINT_BOARD_M5STACK_PAPERCOLOR) || defined(BOARD_M5STACK_PAPERCOLOR)
constexpr BoardProfile ACTIVE = M5STACK_PAPER_COLOR;
#else
constexpr BoardProfile ACTIVE = XTEINK_X4;
#endif

constexpr bool isM5StackPaperColor() { return ACTIVE.board == Board::M5StackPaperColor; }

}  // namespace BoardConfig
