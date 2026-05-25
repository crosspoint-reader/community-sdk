#pragma once

#include <cstdint>

#include "Panel.h"

class EInkDisplay;  // forward decl — X3Panel methods take EInkDisplay& by friendship

// UC81xx-class, 792x528. Implements the full Panel virtual surface for
// the X3 controller, plus a set of X3-specific SPI helpers
// (sendCommandDataX3, sendPlaneX3, fillPlaneX3, loadLutBankX3,
// triggerRefreshX3) used by the X3 implementations to fuse short
// command-data writes, bulk-write planes with Y-flip, load LUT banks,
// and drive the refresh trigger with the X3 power-cycle sequence.
//
// X3-specific runtime state (the resync state machine + grayscale
// validity flag) also lives here so X4 builds carry none of it.
class X3Panel : public Panel {
 public:
  uint16_t displayWidth() const override { return 792; }
  uint16_t displayHeight() const override { return 528; }
  uint32_t spiClockHz() const override { return 16000000; }
  uint8_t initialFullSyncsAfterBegin() const override { return 2; }
  void postResetDelay() const override;
  void onBegin() override;

  void requestResync(uint8_t settlePasses) override;
  void skipInitialResync() override;
  void pollBusy(EInkDisplay& d, const char* comment, const char* completeWord) const override;
  void init(EInkDisplay& d) const override;
  void displayBuffer(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) override;
  void refreshDisplay(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) override;
  void displayGrayBuffer(EInkDisplay& d, bool turnOffScreen, const unsigned char* lut, bool factoryMode) override;
  void grayscaleRevert(EInkDisplay& d) override;
  void displayWindow(EInkDisplay& d, uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool turnOffScreen) override;
  void copyGrayscaleLsbBuffers(EInkDisplay& d, const uint8_t* lsbBuffer) override;
  void copyGrayscaleMsbBuffers(EInkDisplay& d, const uint8_t* msbBuffer) override;
  void copyGrayscaleBuffers(EInkDisplay& d, const uint8_t* lsbBuffer, const uint8_t* msbBuffer) override;
  void writeGrayscalePlaneStrip(EInkDisplay& d, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                uint16_t numRows) override;
  void cleanupGrayscaleBuffers(EInkDisplay& d, const uint8_t* bwBuffer) override;
  void deepSleep(EInkDisplay& d) override;

  // X3-specific runtime state. Owned here (not on EInkDisplay) so
  // X4 builds carry none of it and the resync state machine lives
  // with the code that mutates it.
  bool _x3RedRamSynced = false;
  struct X3GrayState {
    bool lastBaseWasPartial = false;
    bool lsbValid = false;
  };
  X3GrayState _x3GrayState;
  uint8_t _x3InitialFullSyncsRemaining = 0;
  bool _x3ForceFullSyncNext = false;
  uint8_t _x3ForcedConditionPassesNext = 0;

  // SPI command + optional data, fused into one CS-low transaction.
  // Used for LUT register / mode-select / partial-window writes where
  // the payload is small. Bulk plane writes use sendPlaneX3 instead.
  void sendCommandDataX3(EInkDisplay& d, uint8_t cmd, const uint8_t* data, uint16_t len) const;
  void sendCommandDataByteX3(EInkDisplay& d, uint8_t cmd, uint8_t d0) const;
  void sendCommandDataByteX3(EInkDisplay& d, uint8_t cmd, uint8_t d0, uint8_t d1) const;

  // Bulk-write a pixel plane to one of the DTM RAM commands. Y-flips
  // rows in-place (X3 controller scans gates upward), optionally
  // inverts bits before sending, then restores the buffer.
  void sendPlaneX3(EInkDisplay& d, uint8_t ramCmd, uint8_t* buf, bool invert) const;

  // Fill an entire RAM plane with a single byte (e.g. 0xFF for white).
  // Streams a small stack row buffer repeatedly so the framebuffer
  // isn't touched.
  void fillPlaneX3(EInkDisplay& d, uint8_t ramCmd, uint8_t fillByte) const;

  // Load all 5 LUT registers (VCOM/WW/BW/WB/BB) in one call. Each
  // pointer must reference a 42-byte LUT bank in PROGMEM/DRAM.
  void loadLutBankX3(EInkDisplay& d, const uint8_t* vcom, const uint8_t* ww, const uint8_t* bw, const uint8_t* wb,
                     const uint8_t* bb) const;
  void loadLutBankX3WithCdi(EInkDisplay& d, uint8_t cdi0, const uint8_t* vcom, const uint8_t* ww, const uint8_t* bw,
                            const uint8_t* wb, const uint8_t* bb) const;
  void loadLutBankX3WithCdi(EInkDisplay& d, uint8_t cdi0, uint8_t cdi1, const uint8_t* vcom, const uint8_t* ww,
                            const uint8_t* bw, const uint8_t* wb, const uint8_t* bb) const;

  // Power-on if needed, trigger refresh, optionally power-off. The
  // `tag` string appears verbatim in busy-wait log lines.
  void triggerRefreshX3(EInkDisplay& d, bool turnOffScreen, const char* tag) const;
};
