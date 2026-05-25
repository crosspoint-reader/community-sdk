#include <Arduino.h>

#include "EInkDisplay.h"
#include "Panel.h"
#include "X4Constants.h"
#include "X4Luts.h"

// X4 (SSD1677) BUSY polarity: active HIGH. BUSY held HIGH while the
// controller is working, drops LOW when the operation completes.
// Single-phase poll with a 30s safety timeout.
// SSD1677 init: soft reset, internal temp sensor, GDEQ0426T82-specific
// booster soft-start voltages, driver-output dimensions, border waveform,
// then RAM-area windowing and a one-shot AUTO_WRITE for each plane to
// clear stale content. Without the AUTO_WRITE clears the first
// differential refresh diffs against pre-reset RAM and the prior screen
// bleeds through.
void X4Panel::init(EInkDisplay& d) const {
  if (Serial) Serial.printf("[%lu]   Initializing SSD1677 controller...\n", millis());

  constexpr uint8_t TEMP_SENSOR_INTERNAL = 0x80;

  d.sendCommand(CMD_SOFT_RESET);
  d.waitWhileBusy(" CMD_SOFT_RESET");

  d.sendCommand(CMD_TEMP_SENSOR_CONTROL);
  d.sendData(TEMP_SENSOR_INTERNAL);

  // Booster soft-start (GDEQ0426T82-specific values).
  d.sendCommand(CMD_BOOSTER_SOFT_START);
  d.sendData(0xAE);
  d.sendData(0xC7);
  d.sendData(0xC3);
  d.sendData(0xC0);
  d.sendData(0x40);

  // Driver output: display height + scan direction (SM=1 interlaced, TB=0).
  d.sendCommand(CMD_DRIVER_OUTPUT_CONTROL);
  d.sendData((displayHeight() - 1) % 256);
  d.sendData((displayHeight() - 1) / 256);
  d.sendData(0x02);

  d.sendCommand(CMD_BORDER_WAVEFORM);
  d.sendData(0x01);

  d.setRamArea(0, 0, displayWidth(), displayHeight());

  if (Serial) Serial.printf("[%lu]   Clearing RAM buffers...\n", millis());
  d.sendCommand(CMD_AUTO_WRITE_BW_RAM);
  d.sendData(0xF7);
  d.waitWhileBusy(" CMD_AUTO_WRITE_BW_RAM");

  d.sendCommand(CMD_AUTO_WRITE_RED_RAM);
  d.sendData(0xF7);
  d.waitWhileBusy(" CMD_AUTO_WRITE_RED_RAM");

  if (Serial) Serial.printf("[%lu]   SSD1677 controller initialized\n", millis());
}

// X4 displayBuffer: set RAM area, write framebuffer to BW (and RED for
// non-FAST modes), then trigger the requested refresh via
// EInkDisplay::refreshDisplay. SINGLE_BUFFER_MODE syncs RED post-refresh
// to prepare for the next fast differential; dual-buffer-mode preserves
// the previous frame in frameBufferActive for the same purpose.
void X4Panel::displayBuffer(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) {
  d.setRamArea(0, 0, displayWidth(), displayHeight());

  if (mode != RefreshMode::FAST_REFRESH) {
    // Full / half: write the new frame to both BW and RED.
    d.writeRamBuffer(CMD_WRITE_RAM_BW, d.frameBuffer, bufferSize());
    d.writeRamBuffer(CMD_WRITE_RAM_RED, d.frameBuffer, bufferSize());
  } else {
    // Fast: BW gets the new frame; RED retains the prior frame for
    // differential comparison. Single-buffer mode: RED is already
    // synced from the previous refresh's post-step below. Dual-buffer
    // mode: write back from frameBufferActive (the previous frame).
    d.writeRamBuffer(CMD_WRITE_RAM_BW, d.frameBuffer, bufferSize());
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    d.writeRamBuffer(CMD_WRITE_RAM_RED, d.frameBufferActive, bufferSize());
#endif
  }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  d.swapBuffers();
#endif

  d.refreshDisplay(mode, turnOffScreen);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // Sync RED RAM with the just-displayed frame so the next fast
  // differential has the right "previous frame" to diff against.
  d.setRamArea(0, 0, displayWidth(), displayHeight());
  d.writeRamBuffer(CMD_WRITE_RAM_RED, d.frameBuffer, bufferSize());
#endif
}

// SSD1677 refreshDisplay: pick the mode bits (NORMAL vs BYPASS_RED for
// FAST vs other), set CTRL2 to drive the requested transition, then
// master-activation + waitWhileBusy on the named refresh type.
void X4Panel::refreshDisplay(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) {
  d.sendCommand(CMD_DISPLAY_UPDATE_CTRL1);
  d.sendData((mode == RefreshMode::FAST_REFRESH) ? CTRL1_NORMAL : CTRL1_BYPASS_RED);

  uint8_t displayMode = 0x00;
  if (!d.isScreenOn) {
    d.isScreenOn = true;
    displayMode |= 0xC0;  // CLOCK_ON + ANALOG_ON
  }
  if (turnOffScreen) {
    d.isScreenOn = false;
    displayMode |= 0x03;  // ANALOG_OFF_PHASE + CLOCK_OFF
  }
  if (mode == RefreshMode::FULL_REFRESH) {
    displayMode |= 0x34;
  } else if (mode == RefreshMode::HALF_REFRESH) {
    d.sendCommand(CMD_WRITE_TEMP);
    d.sendData(0x5A);
    displayMode |= 0xD4;
  } else {  // FAST_REFRESH
    displayMode |= d.customLutActive ? 0x0C : 0x1C;
  }

  const char* refreshType = (mode == RefreshMode::FULL_REFRESH)   ? "full"
                            : (mode == RefreshMode::HALF_REFRESH) ? "half"
                                                                  : "fast";
  if (Serial) Serial.printf("[%lu]   Powering on display 0x%02X (%s refresh)...\n", millis(), displayMode, refreshType);
  d.sendCommand(CMD_DISPLAY_UPDATE_CTRL2);
  d.sendData(displayMode);
  d.sendCommand(CMD_MASTER_ACTIVATION);
  if (Serial) Serial.printf("[%lu]   Waiting for display refresh...\n", millis());
  d.waitWhileBusy(refreshType);
}

void X4Panel::displayWindow(EInkDisplay& d, uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool turnOffScreen) {
  if (Serial) Serial.printf("[%lu]   Displaying window at (%d,%d) size (%dx%d)\n", millis(), x, y, w, h);

  if (x + w > displayWidth() || y + h > displayHeight()) {
    if (Serial) Serial.printf("[%lu]   ERROR: Window bounds exceed display dimensions!\n", millis());
    return;
  }
  if (x % 8 != 0 || w % 8 != 0) {
    if (Serial) Serial.printf("[%lu]   ERROR: Window x and width must be byte-aligned (multiples of 8)!\n", millis());
    return;
  }
  if (!d.frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }
  if (d.inGrayscaleMode) grayscaleRevert(d);

  const uint16_t windowWidthBytes = w / 8;
  const uint32_t windowBufferSize = windowWidthBytes * h;
  if (Serial)
    Serial.printf("[%lu]   Window buffer size: %lu bytes (%d x %d pixels)\n", millis(), windowBufferSize, w, h);

  std::vector<uint8_t> windowBuffer(windowBufferSize);
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t srcOffset = (y + row) * displayWidthBytes() + (x / 8);
    memcpy(&windowBuffer[row * windowWidthBytes], &d.frameBuffer[srcOffset], windowWidthBytes);
  }
  d.setRamArea(x, y, w, h);
  d.writeRamBuffer(CMD_WRITE_RAM_BW, windowBuffer.data(), windowBufferSize);

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  std::vector<uint8_t> previousWindowBuffer(windowBufferSize);
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t srcOffset = (y + row) * displayWidthBytes() + (x / 8);
    memcpy(&previousWindowBuffer[row * windowWidthBytes], &d.frameBufferActive[srcOffset], windowWidthBytes);
  }
  d.writeRamBuffer(CMD_WRITE_RAM_RED, previousWindowBuffer.data(), windowBufferSize);
#endif

  refreshDisplay(d, RefreshMode::FAST_REFRESH, turnOffScreen);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  d.setRamArea(x, y, w, h);
  d.writeRamBuffer(CMD_WRITE_RAM_RED, windowBuffer.data(), windowBufferSize);
#endif
  if (Serial) Serial.printf("[%lu]   Window display complete\n", millis());
}

extern const unsigned char lut_grayscale[];
extern const unsigned char lut_grayscale_revert[];
extern const unsigned char lut_factory_quality[];

void X4Panel::displayGrayBuffer(EInkDisplay& d, bool turnOffScreen, const unsigned char* lut, bool factoryMode) {
  d.drawGrayscale = false;
  // Differential mode keeps inGrayscaleMode set; reader AA clears it
  // via cleanupGrayscaleBuffers before the next BW page turn.
  d.inGrayscaleMode = !factoryMode;

  const unsigned char* selectedLut = lut;
  if (selectedLut == nullptr) selectedLut = factoryMode ? lut_factory_quality : lut_grayscale;
  setCustomLUT(d, true, selectedLut);

  if (factoryMode) {
    // Absolute mode: explicit full power cycle. CTRL1 normal because a
    // prior HALF leaves it at BYPASS_RED which would break 4-level grayscale.
    d.sendCommand(CMD_DISPLAY_UPDATE_CTRL1);
    d.sendData(CTRL1_NORMAL);
    d.sendCommand(CMD_DISPLAY_UPDATE_CTRL2);
    d.sendData(0xC7);  // CLOCK+ANALOG+DISPLAY+ANALOG_OFF+CLOCK_OFF self-contained cycle
    d.sendCommand(CMD_MASTER_ACTIVATION);
    d.waitWhileBusy("factory_gray");
    d.isScreenOn = false;
  } else {
    refreshDisplay(d, RefreshMode::FAST_REFRESH, turnOffScreen);
  }
  setCustomLUT(d, false);
}

void X4Panel::copyGrayscaleLsbBuffers(EInkDisplay& d, const uint8_t* lsbBuffer) {
  if (!lsbBuffer) return;
  d.setRamArea(0, 0, displayWidth(), displayHeight());
  d.writeRamBuffer(CMD_WRITE_RAM_BW, lsbBuffer, bufferSize());
}

void X4Panel::copyGrayscaleMsbBuffers(EInkDisplay& d, const uint8_t* msbBuffer) {
  if (!msbBuffer) return;
  d.setRamArea(0, 0, displayWidth(), displayHeight());
  d.writeRamBuffer(CMD_WRITE_RAM_RED, msbBuffer, bufferSize());
}

void X4Panel::copyGrayscaleBuffers(EInkDisplay& d, const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  d.setRamArea(0, 0, displayWidth(), displayHeight());
  d.writeRamBuffer(CMD_WRITE_RAM_BW, lsbBuffer, bufferSize());
  d.writeRamBuffer(CMD_WRITE_RAM_RED, msbBuffer, bufferSize());
}

void X4Panel::writeGrayscalePlaneStrip(EInkDisplay& d, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                       uint16_t numRows) {
  if (!rows || numRows == 0) return;
  const uint8_t ramCmd = (plane == GrayPlane::GRAY_PLANE_LSB) ? CMD_WRITE_RAM_BW : CMD_WRITE_RAM_RED;
  d.setRamArea(0, yStart, displayWidth(), numRows);
  d.sendCommand(ramCmd);
  d.sendData(rows, static_cast<uint16_t>(static_cast<uint32_t>(numRows) * displayWidthBytes()));
}

void X4Panel::cleanupGrayscaleBuffers(EInkDisplay& d, const uint8_t* bwBuffer) {
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  if (!bwBuffer) return;
  d.setRamArea(0, 0, displayWidth(), displayHeight());
  d.writeRamBuffer(CMD_WRITE_RAM_RED, bwBuffer, bufferSize());
  d.inGrayscaleMode = false;
#else
  (void)d;
  (void)bwBuffer;
#endif
}

void X4Panel::setCustomLUT(EInkDisplay& d, bool enabled, const unsigned char* lutData) {
  if (enabled) {
    if (Serial) Serial.printf("[%lu]   Loading custom LUT...\n", millis());
    d.sendCommand(CMD_WRITE_LUT);
    for (uint16_t i = 0; i < 105; i++) d.sendData(pgm_read_byte(&lutData[i]));
    d.sendCommand(CMD_GATE_VOLTAGE);
    d.sendData(pgm_read_byte(&lutData[105]));
    d.sendCommand(CMD_SOURCE_VOLTAGE);
    d.sendData(pgm_read_byte(&lutData[106]));
    d.sendData(pgm_read_byte(&lutData[107]));
    d.sendData(pgm_read_byte(&lutData[108]));
    d.sendCommand(CMD_WRITE_VCOM);
    d.sendData(pgm_read_byte(&lutData[109]));
    d.customLutActive = true;
    if (Serial) Serial.printf("[%lu]   Custom LUT loaded\n", millis());
  } else {
    d.customLutActive = false;
    if (Serial) Serial.printf("[%lu]   Custom LUT disabled\n", millis());
  }
}

void X4Panel::deepSleep(EInkDisplay& d) {
  if (Serial) Serial.printf("[%lu]   Preparing display for deep sleep...\n", millis());
  if (d.isScreenOn) {
    d.sendCommand(CMD_DISPLAY_UPDATE_CTRL1);
    d.sendData(0x00);
    d.sendCommand(CMD_DISPLAY_UPDATE_CTRL2);
    d.sendData(0x83);
    d.sendCommand(CMD_MASTER_ACTIVATION);
    d.waitWhileBusy("display_off");
    d.isScreenOn = false;
  }
  d.sendCommand(CMD_DEEP_SLEEP);
  d.sendData(0x01);  // DSM1
  if (Serial) Serial.printf("[%lu]   Display in deep sleep\n", millis());
}

extern const unsigned char lut_grayscale_revert[];
void X4Panel::grayscaleRevert(EInkDisplay& d) {
  setCustomLUT(d, true, lut_grayscale_revert);
  refreshDisplay(d, RefreshMode::FAST_REFRESH, false);
  setCustomLUT(d, false);
}

void X4Panel::pollBusy(EInkDisplay& d, const char* comment, const char* completeWord) const {
  unsigned long start = millis();
  while (digitalRead(d._busy) == HIGH) {
    delay(1);
    EInkDisplay::tickIdleHook();
    if (millis() - start > 30000) break;
  }
  if (comment && Serial) {
    Serial.printf("[%lu]   %s: %s (%lu ms)\n", millis(), completeWord, comment, millis() - start);
  }
}
