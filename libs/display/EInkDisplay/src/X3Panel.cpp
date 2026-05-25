#include "X3Panel.h"

#include <Arduino.h>
#include <SPI.h>

#include <cstdio>
#include <cstring>

#include "EInkDisplay.h"
#include "X3Constants.h"
#include "X3Luts.h"

// X3-only state-machine hooks, dispatched here via the Panel virtual
// override (EInkDisplay's public-API entry points just forward).
// requestResync arms the next refresh to be a forced full + N settle
// passes; the dispatch paths consume these flags. skipInitialResync
// zeroes the warm-restart counter so the first two paints aren't
// promoted to FULL.
void X3Panel::requestResync(uint8_t settlePasses) {
  _x3ForceFullSyncNext = true;
  _x3ForcedConditionPassesNext = settlePasses;
}

void X3Panel::skipInitialResync() {
  _x3InitialFullSyncsRemaining = 0;
  _x3RedRamSynced = true;
}

void X3Panel::postResetDelay() const { delay(50); }

void X3Panel::onBegin() {
  _x3RedRamSynced = false;
  _x3InitialFullSyncsRemaining = initialFullSyncsAfterBegin();
  _x3ForceFullSyncNext = false;
  _x3ForcedConditionPassesNext = 0;
  _x3GrayState = {};
}

// X3 refreshDisplay just routes through displayBuffer — the X3 path
// owns power-on/LUT-load/plane-write/trigger/post-conditioning, so
// re-issuing it is the natural "refresh from current state".
void X3Panel::refreshDisplay(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) {
  displayBuffer(d, mode, turnOffScreen);
}

// X3 grayscaleRevert: scrub the panel to clean white using the OEM
// half (scrub) bank. After differential grayscale, DTM1 holds the
// AA LSB plane and DTM2 holds the AA MSB plane — neither is a valid
// "previous BW frame" for a normal diff. Write all-white to both
// planes, then load the scrub bank (WW=BW, WB=BB collapse) so the
// controller drives every pixel toward its DTM2-target regardless
// of DTM1. With both planes white, every pixel gets "drive to white".
void X3Panel::grayscaleRevert(EInkDisplay& d) {
  fillPlaneX3(d, CMD_X3_DTM1, 0xFF);
  d.sendCommand(CMD_X3_DATA_STOP);
  fillPlaneX3(d, CMD_X3_DTM2, 0xFF);
  d.sendCommand(CMD_X3_DATA_STOP);
  // CDI 0xA9 (absolute mode) per OEM scrub loader (FUN_420a0e7c).
  loadLutBankX3WithCdi(d, 0xA9, 0x07, lut_x3_vcom_half, lut_x3_ww_half, lut_x3_bw_half, lut_x3_wb_half, lut_x3_bb_half);
  triggerRefreshX3(d, /*turnOffScreen=*/false, "(revert)");
  _x3RedRamSynced = true;
}

// X3 displayWindow: route through displayBuffer (full-screen refresh)
// rather than maintain a second X3-specific partial-update path. Visual
// difference: unchanged regions also refresh. displayBuffer already
// handles inGrayscaleMode revert and the wake-from-off HALF policy.
void X3Panel::displayWindow(EInkDisplay& d, uint16_t /*x*/, uint16_t /*y*/, uint16_t /*w*/, uint16_t /*h*/,
                            bool turnOffScreen) {
  displayBuffer(d, RefreshMode::FAST_REFRESH, turnOffScreen);
}

// X3 grayscale RAM load — LSB plane to DTM1, MSB plane to DTM2. Each
// plane is Y-flipped in place (X3 scans gates upward), bulk-sent, then
// flipped back. const_cast is safe because the buffer is fully
// restored before returning.
static void x3FlipRowsInPlace(uint8_t* p, uint16_t height, uint16_t widthBytes) {
  uint8_t rowTmp[128];
  for (uint16_t top = 0, bot = height - 1; top < bot; top++, bot--) {
    uint8_t* rowA = p + static_cast<uint32_t>(top) * widthBytes;
    uint8_t* rowB = p + static_cast<uint32_t>(bot) * widthBytes;
    memcpy(rowTmp, rowA, widthBytes);
    memcpy(rowA, rowB, widthBytes);
    memcpy(rowB, rowTmp, widthBytes);
  }
}

void X3Panel::copyGrayscaleLsbBuffers(EInkDisplay& d, const uint8_t* lsbBuffer) {
  if (!lsbBuffer) {
    _x3GrayState.lsbValid = false;
    return;
  }
  auto* buf = const_cast<uint8_t*>(lsbBuffer);
  x3FlipRowsInPlace(buf, displayHeight(), displayWidthBytes());
  d.sendCommand(CMD_X3_DTM1);
  d.sendData(buf, static_cast<uint16_t>(bufferSize()));
  d.sendCommand(CMD_X3_DATA_STOP);  // no refresh follows; commit DTM1
  x3FlipRowsInPlace(buf, displayHeight(), displayWidthBytes());
  _x3GrayState.lsbValid = true;
}

void X3Panel::copyGrayscaleMsbBuffers(EInkDisplay& d, const uint8_t* msbBuffer) {
  if (!msbBuffer) return;
  if (!_x3GrayState.lsbValid) return;
  auto* buf = const_cast<uint8_t*>(msbBuffer);
  x3FlipRowsInPlace(buf, displayHeight(), displayWidthBytes());
  d.sendCommand(CMD_X3_DTM2);
  d.sendData(buf, static_cast<uint16_t>(bufferSize()));
  d.sendCommand(CMD_X3_DATA_STOP);  // no refresh follows; commit DTM2
  x3FlipRowsInPlace(buf, displayHeight(), displayWidthBytes());
}

void X3Panel::copyGrayscaleBuffers(EInkDisplay& d, const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  copyGrayscaleLsbBuffers(d, lsbBuffer);
  copyGrayscaleMsbBuffers(d, msbBuffer);
}

// X3 streaming: PTL partial-window to the band, then bulk-write the
// rows (bottom-first within the band to reproduce the whole-plane
// Y-flip; X3 scans gates upward). Each band is its own CS-low burst
// so SD-card font reads can interleave between bands.
void X3Panel::writeGrayscalePlaneStrip(EInkDisplay& d, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                       uint16_t numRows) {
  if (!rows || numRows == 0) return;
  const uint8_t ramCmd = (plane == GrayPlane::GRAY_PLANE_LSB) ? CMD_X3_DTM1 : CMD_X3_DTM2;
  const uint16_t xEnd = displayWidth() - 1;
  const uint16_t yEnd = yStart + numRows - 1;
  const uint8_t win[9] = {0,
                          0,
                          static_cast<uint8_t>(xEnd >> 8),
                          static_cast<uint8_t>(xEnd & 0xFF),
                          static_cast<uint8_t>(yStart >> 8),
                          static_cast<uint8_t>(yStart & 0xFF),
                          static_cast<uint8_t>(yEnd >> 8),
                          static_cast<uint8_t>(yEnd & 0xFF),
                          0x01};
  d.sendCommand(CMD_X3_PARTIAL_IN);
  sendCommandDataX3(d, CMD_X3_PARTIAL_WINDOW, win, 9);
  d.sendCommand(ramCmd);
  SPI.beginTransaction(d.spiSettings);
  digitalWrite(d._dc, HIGH);
  digitalWrite(d._cs, LOW);
  for (int r = static_cast<int>(numRows) - 1; r >= 0; r--) {
    SPI.writeBytes(rows + static_cast<uint32_t>(r) * displayWidthBytes(), displayWidthBytes());
  }
  digitalWrite(d._cs, HIGH);
  SPI.endTransaction();
  d.sendCommand(CMD_X3_PARTIAL_OUT);
  // displayGrayBuffer gates on lsbValid; the tiled path bypasses
  // copyGrayscaleLsbBuffers, so mark it when the LSB plane lands.
  if (plane == GrayPlane::GRAY_PLANE_LSB) _x3GrayState.lsbValid = true;
}

// X3 cleanupGrayscaleBuffers: rebase BOTH planes from a restored BW
// buffer (Y-flip once, send to both RAMs, flip back). After this both
// planes hold the BW frame, so the next FAST diff has matching DTM1
// and a target DTM2 = caller's next frame.
void X3Panel::cleanupGrayscaleBuffers(EInkDisplay& d, const uint8_t* bwBuffer) {
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  if (!bwBuffer) return;
  auto* buf = const_cast<uint8_t*>(bwBuffer);
  x3FlipRowsInPlace(buf, displayHeight(), displayWidthBytes());
  d.sendCommand(CMD_X3_DTM2);
  d.sendData(buf, static_cast<uint16_t>(bufferSize()));
  d.sendCommand(CMD_X3_DATA_STOP);
  d.sendCommand(CMD_X3_DTM1);
  d.sendData(buf, static_cast<uint16_t>(bufferSize()));
  d.sendCommand(CMD_X3_DATA_STOP);
  x3FlipRowsInPlace(buf, displayHeight(), displayWidthBytes());
  _x3RedRamSynced = true;
  _x3ForceFullSyncNext = false;
  _x3ForcedConditionPassesNext = 0;
  d.inGrayscaleMode = false;
#else
  (void)d;
  (void)bwBuffer;
#endif
}

// X3 displayGrayBuffer: X3 has no SSD1677-style setCustomLUT path;
// LUTs go through the banked loadLutBankX3WithCdi helpers.
// factoryMode picks the OEM _full bank (no separate fast factory LUTs
// on X3 — fast falls back to quality with a log line). Differential
// mode uses the OEM gc (grayscale/AA) bank with CDI 0x97 → 0xD7.
void X3Panel::displayGrayBuffer(EInkDisplay& d, bool turnOffScreen, const unsigned char* lut, bool factoryMode) {
  d.drawGrayscale = false;
  if (!_x3GrayState.lsbValid) return;

  // Differential mode leaves the gray bank loaded in the LUT registers,
  // so a subsequent BW page turn must run grayscaleRevert first.
  d.inGrayscaleMode = !factoryMode;

  if (factoryMode) {
    if (Serial) {
      // lut_factory_fast/quality are X4 externs; equality-compare to
      // tag the log line. X3 still uses _full bank either way.
      extern const unsigned char lut_factory_fast[];
      const char* modeTag = (lut == lut_factory_fast) ? "factory_fast (fallback to quality)" : "factory_quality";
      Serial.printf("[%lu]   X3_GRAY_MODE=%s\n", millis(), modeTag);
    }
    // CDI 0x29 (differential) — _full bank's OEM CDI per FUN_420a1218.
    loadLutBankX3WithCdi(d, 0x29, 0x07, lut_x3_vcom_full, lut_x3_ww_full, lut_x3_bw_full, lut_x3_wb_full,
                         lut_x3_bb_full);
  } else {
    if (Serial) Serial.printf("[%lu]   X3_GRAY_MODE=oem_gc\n", millis());
    loadLutBankX3WithCdi(d, 0x97, lut_x3_vcom_gc, lut_x3_ww_gc, lut_x3_bw_gc, lut_x3_wb_gc, lut_x3_bb_gc);
  }

  triggerRefreshX3(d, turnOffScreen, "(gray)");
  if (!factoryMode) {
    // OEM's GC path leaves CDI at 0xD7 after the grayscale refresh.
    sendCommandDataByteX3(d, CMD_X3_VCOM_DATA_INTERVAL, 0xD7);
  }

  _x3RedRamSynced = false;
  _x3ForceFullSyncNext = false;
  _x3ForcedConditionPassesNext = 0;
  _x3GrayState.lsbValid = false;
}

// X3 deepSleep: UC81xx Deep Sleep (DSLP = 0x07) with check-code 0xA5.
// Power down via POWER_OFF first if the panel is on.
void X3Panel::deepSleep(EInkDisplay& d) {
  if (Serial) Serial.printf("[%lu]   Preparing X3 for deep sleep...\n", millis());
  if (d.isScreenOn) {
    d.sendCommand(CMD_X3_POWER_OFF);
    d.waitForRefresh(" X3_POF(sleep)");
    d.isScreenOn = false;
  }
  sendCommandDataByteX3(d, /*DSLP*/ 0x07, /*check-code*/ 0xA5);
  if (Serial) Serial.printf("[%lu]   X3 in deep sleep\n", millis());
}

// X3 displayBuffer: three-tier refresh hierarchy with a state machine
// that decides FAST / HALF / FULL per call. Honors the resync hooks
// (initial-full-syncs counter, forced-full flag) and posts the cond
// passes + the post-full settle pass (the PR #14 ghost fix). Per-mode
// LUT bank is loaded via the X3 SPI helpers; DTM2 always gets the new
// frame; DTM1 gets a white baseline for FULL (we don't keep a software
// previous-frame copy on the C3) and is synced to the just-displayed
// frame at the end of every refresh so the next FAST diff has the
// right "previous" plane.
void X3Panel::displayBuffer(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) {
  const bool fastMode = (mode == RefreshMode::FAST_REFRESH);
  const bool halfMode = (mode == RefreshMode::HALF_REFRESH);
  const bool forcedFullSync = _x3ForceFullSyncNext;
  const bool doFullSync =
      (!fastMode && !halfMode) || !_x3RedRamSynced || _x3InitialFullSyncsRemaining > 0 || forcedFullSync;
  const bool doHalfSync = halfMode && !doFullSync;

  if (Serial) {
    const char* tag = doFullSync ? "FULL" : doHalfSync ? "HALF" : "FAST";
    Serial.printf("[%lu]   X3_OEM_%s\n", millis(), tag);
  }
  _x3GrayState.lastBaseWasPartial = !doFullSync;

  if (doFullSync) {
    loadLutBankX3WithCdi(d, 0x29, 0x07, lut_x3_vcom_full, lut_x3_ww_full, lut_x3_bw_full, lut_x3_wb_full,
                         lut_x3_bb_full);
    // FULL plane semantics: DTM1 = all-white baseline (no software
    // previous-frame copy on the C3), DTM2 = new frame. Controller
    // diffs per pixel: black-target pixels get strong WB drive (cleans
    // ghost residue), white-target pixels get a light WW drive. DTM1
    // is re-synced to the current frame after the refresh below.
    fillPlaneX3(d, CMD_X3_DTM1, 0xFF);
    d.sendCommand(CMD_X3_DATA_STOP);
    sendPlaneX3(d, CMD_X3_DTM2, d.frameBuffer, false);
  } else if (doHalfSync) {
    // HALF: _half (scrub) LUTs in absolute mode. WW=BW and WB=BB
    // collapse drive to depend on the target frame (DTM2) only; DTM1's
    // contents become irrelevant. OEM uses CDI 0xA9 with this bank
    // (FUN_420a0e7c); 0x29 caused DC bias accumulation under repeat.
    loadLutBankX3WithCdi(d, 0xA9, 0x07, lut_x3_vcom_half, lut_x3_ww_half, lut_x3_bw_half, lut_x3_wb_half,
                         lut_x3_bb_half);
    sendPlaneX3(d, CMD_X3_DTM2, d.frameBuffer, false);
  } else {
    // FAST differential: turbo LUTs, DTM1 retains previous frame.
    loadLutBankX3WithCdi(d, 0x29, 0x07, lut_x3_vcom_fast, lut_x3_ww_fast, lut_x3_bw_fast, lut_x3_wb_fast,
                         lut_x3_bb_fast);
    sendPlaneX3(d, CMD_X3_DTM2, d.frameBuffer, false);
  }

  // Re-issue POWER_ON for FULL even when the screen is already on (the
  // charge pump needs the bump for FULL's higher-current drive).
  // triggerRefreshX3 only power-ons when !isScreenOn, so inline here.
  if (!d.isScreenOn || doFullSync) {
    d.sendCommand(CMD_X3_POWER_ON);
    d.waitForRefresh(" X3_PON");
    d.isScreenOn = true;
  }
  if (Serial) Serial.printf("[%lu]   X3_OEM_TRIGGER=DRF\n", millis());
  d.sendCommand(CMD_X3_DISPLAY_REFRESH);
  d.waitForRefresh(" X3_DRF");
  if (turnOffScreen) {
    d.sendCommand(CMD_X3_POWER_OFF);
    d.waitForRefresh(" X3_POF");
    d.isScreenOn = false;
  }

  if (!fastMode) delay(200);

  uint8_t postConditionPasses = 0;
  if (doFullSync) {
    if (forcedFullSync)
      postConditionPasses = _x3ForcedConditionPassesNext;
    else if (_x3InitialFullSyncsRemaining == 1)
      postConditionPasses = 1;
  }

  if (postConditionPasses > 0) {
    const uint16_t xStart = 0;
    const uint16_t xEnd = static_cast<uint16_t>(displayWidth() - 1);
    const uint16_t yStart = 0;
    const uint16_t yEnd = static_cast<uint16_t>(displayHeight() - 1);
    const uint8_t w[9] = {
        static_cast<uint8_t>(xStart >> 8), static_cast<uint8_t>(xStart & 0xFF), static_cast<uint8_t>(xEnd >> 8),
        static_cast<uint8_t>(xEnd & 0xFF), static_cast<uint8_t>(yStart >> 8),   static_cast<uint8_t>(yStart & 0xFF),
        static_cast<uint8_t>(yEnd >> 8),   static_cast<uint8_t>(yEnd & 0xFF),   0x01};

    // CDI 0xA9 (absolute) — _normal bank from OEM's normal loader
    // (FUN_420a12a0) which sets CDI 0xA9 before loading.
    loadLutBankX3WithCdi(d, 0xA9, 0x07, lut_x3_vcom_normal, lut_x3_ww_normal, lut_x3_bw_normal, lut_x3_wb_normal,
                         lut_x3_bb_normal);

    for (uint8_t i = 0; i < postConditionPasses; i++) {
      if (Serial)
        Serial.printf("[%lu]   X3_OEM_COND %u/%u\n", millis(), static_cast<unsigned>(i + 1),
                      static_cast<unsigned>(postConditionPasses));
      d.sendCommand(CMD_X3_PARTIAL_IN);
      sendCommandDataX3(d, CMD_X3_PARTIAL_WINDOW, w, 9);
      sendPlaneX3(d, CMD_X3_DTM2, d.frameBuffer, false);
      d.sendCommand(CMD_X3_PARTIAL_OUT);
      triggerRefreshX3(d, /*turnOffScreen=*/false, "(cond)");
    }
  }

  // Sync DTM1 ("old" RAM) with the just-displayed frame so the next
  // FAST diff has the right baseline.
  sendPlaneX3(d, CMD_X3_DTM1, d.frameBuffer, false);
  d.sendCommand(CMD_X3_DATA_STOP);  // commit DTM1 — no refresh follows
  _x3RedRamSynced = true;

  // Post-full settle (PR #14 ghost fix): the first differential after a
  // FULL garbles on X3 because the controller's post-full state corrupts
  // the next fast/half diff. Spend that slot here with a no-op FAST of
  // the just-displayed frame. DTM1 and DTM2 both hold it, so nothing
  // visibly changes, but the controller is left in the post-fast state
  // so the caller's next diff (menu open, first page turn) is clean.
  if (doFullSync) {
    loadLutBankX3WithCdi(d, 0x29, 0x07, lut_x3_vcom_fast, lut_x3_ww_fast, lut_x3_bw_fast, lut_x3_wb_fast,
                         lut_x3_bb_fast);
    sendPlaneX3(d, CMD_X3_DTM2, d.frameBuffer, false);
    triggerRefreshX3(d, turnOffScreen, "(post-full settle)");
    sendPlaneX3(d, CMD_X3_DTM1, d.frameBuffer, false);
    d.sendCommand(CMD_X3_DATA_STOP);
  }

  if (doFullSync && _x3InitialFullSyncsRemaining > 0) {
    _x3InitialFullSyncsRemaining--;
  }
  _x3ForceFullSyncNext = false;
  _x3ForcedConditionPassesNext = 0;
}

// UC81xx init: panel-setting / resolution / gate-source start /
// power-off-seq / power-setting / VCOM DC / booster soft-start / PLL /
// LV-selection. Then a manual RAM-clear via DTM1/DTM2 + DATA_STOP
// commits (UC81xx has no AUTO_WRITE_*_RAM convenience opcodes like
// SSD1677 does); without this the first differential refresh diffs
// against stale RAM and the prior screen bleeds through. Leaves
// frameBuffer at 0xFF so it matches the RAM state just written and
// matches begin()'s earlier memset(frameBuffer0, 0xFF).
void X3Panel::init(EInkDisplay& d) const {
  d.sendCommand(CMD_X3_PANEL_SETTING);
  d.sendData(0x3F);  // OEM value
  d.sendData(0x0A);  // OEM value (was 0x08)
  d.sendCommand(CMD_X3_RESOLUTION);
  d.sendData(0x03);
  d.sendData(0x18);
  d.sendData(0x02);
  d.sendData(0x58);
  d.sendCommand(CMD_X3_GATE_SOURCE_START);
  d.sendData(0x00);
  d.sendData(0x00);
  d.sendData(0x00);
  d.sendData(0x00);
  d.sendCommand(CMD_X3_POWER_OFF_SEQ);
  d.sendData(0x20);  // OEM value (was 0x1D)
  d.sendCommand(CMD_X3_POWER_SETTING);
  d.sendData(0x07);
  d.sendData(0x17);
  d.sendData(0x3F);
  d.sendData(0x3F);
  d.sendData(0x17);
  d.sendCommand(CMD_X3_VCOM_DC);
  d.sendData(0x24);  // OEM value (was 0x1D)
  d.sendCommand(CMD_X3_BOOSTER_SOFT_START);
  d.sendData(0x25);
  d.sendData(0x25);
  d.sendData(0x3C);
  d.sendData(0x37);
  d.sendCommand(CMD_X3_PLL_CONTROL);
  d.sendData(0x09);
  d.sendCommand(CMD_X3_LV_SELECTION);
  d.sendData(0x02);

  if (d.frameBuffer) {
    memset(d.frameBuffer, 0xFF, bufferSize());
    d.sendCommand(CMD_X3_DTM1);
    d.sendData(d.frameBuffer, static_cast<uint16_t>(bufferSize()));
    d.sendCommand(CMD_X3_DATA_STOP);
    d.sendCommand(CMD_X3_DTM2);
    d.sendData(d.frameBuffer, static_cast<uint16_t>(bufferSize()));
    d.sendCommand(CMD_X3_DATA_STOP);
  }

  d.isScreenOn = false;
}

// X3 (UC81xx-class) BUSY polarity: active LOW. Idle = HIGH, working
// = LOW. After a command that does work, BUSY transitions HIGH -> LOW
// (work starts) -> HIGH (work done). We poll up to 1s for the
// HIGH -> LOW edge (race protection: the controller may not assert
// BUSY until shortly after the trigger returns), then up to 30s for
// the LOW -> HIGH edge. If we never observe the LOW phase the
// operation either completed faster than we could see or was a no-op,
// and we skip the completion log line.
void X3Panel::pollBusy(EInkDisplay& d, const char* comment, const char* completeWord) const {
  unsigned long start = millis();
  bool sawLow = false;
  while (digitalRead(d._busy) == HIGH) {
    delay(1);
    EInkDisplay::tickIdleHook();
    if (millis() - start > 1000) break;
  }
  if (digitalRead(d._busy) == LOW) {
    sawLow = true;
    while (digitalRead(d._busy) == LOW) {
      delay(1);
      EInkDisplay::tickIdleHook();
      if (millis() - start > 30000) break;
    }
  }
  if (!sawLow) return;
  if (comment && Serial) {
    Serial.printf("[%lu]   %s: %s (%lu ms)\n", millis(), completeWord, comment, millis() - start);
  }
}

// `sendCommandDataX3` / `sendCommandDataByteX3` bundle a command byte and a
// short data payload into a single CS-low SPI transaction. Used for LUT
// register writes (cmd 0x20-0x24 + 42 bytes), mode select (cmd 0x50 + 2
// bytes), and partial-window descriptors (cmd 0x90 + 9 bytes). Saves one
// CS toggle vs the separated form.
//
// The bulk plane-write helpers (`sendPlaneX3`, `fillPlaneX3`) and the init
// RAM-clear use the separated `sendCommand()` + `sendData()` form instead.
// UC81xx accepts both for DTM1/DTM2 streams; the separation makes the
// in-place Y-flip and row-streaming patterns simpler to express. This is
// not a hard atomicity requirement of the controller.
void X3Panel::sendCommandDataX3(EInkDisplay& d, uint8_t cmd, const uint8_t* data, uint16_t len) const {
  SPI.beginTransaction(d.spiSettings);
  digitalWrite(d._cs, LOW);
  digitalWrite(d._dc, LOW);
  SPI.transfer(cmd);
  if (len > 0 && data != nullptr) {
    digitalWrite(d._dc, HIGH);
    SPI.writeBytes(data, len);
  }
  digitalWrite(d._cs, HIGH);
  SPI.endTransaction();
}

void X3Panel::sendCommandDataByteX3(EInkDisplay& d, uint8_t cmd, uint8_t d0) const {
  const uint8_t buf[1] = {d0};
  sendCommandDataX3(d, cmd, buf, 1);
}

void X3Panel::sendCommandDataByteX3(EInkDisplay& d, uint8_t cmd, uint8_t d0, uint8_t d1) const {
  const uint8_t buf[2] = {d0, d1};
  sendCommandDataX3(d, cmd, buf, 2);
}

void X3Panel::sendPlaneX3(EInkDisplay& d, uint8_t ramCmd, uint8_t* buf, bool invert) const {
  // The X3 controller scans gates upward (UD=1), so the first byte sent
  // maps to the bottom-left pixel. Our framebuffer stores row 0 at offset
  // 0 (top), so we Y-flip rows before sending and restore after. Avoids
  // allocating a transposed copy.
  auto flipRowsInPlace = [&](uint8_t* p) {
    uint8_t rowTmp[128];
    for (uint16_t top = 0, bot = displayHeight() - 1; top < bot; top++, bot--) {
      uint8_t* rowA = p + static_cast<uint32_t>(top) * displayWidthBytes();
      uint8_t* rowB = p + static_cast<uint32_t>(bot) * displayWidthBytes();
      memcpy(rowTmp, rowA, displayWidthBytes());
      memcpy(rowA, rowB, displayWidthBytes());
      memcpy(rowB, rowTmp, displayWidthBytes());
    }
  };
  auto invertBuffer = [&](uint8_t* p) {
    auto* w = reinterpret_cast<uint32_t*>(p);
    for (uint32_t i = 0; i < bufferSize() / 4; i++) w[i] = ~w[i];
  };
  if (invert) invertBuffer(buf);
  flipRowsInPlace(buf);
  d.sendCommand(ramCmd);
  d.sendData(buf, static_cast<uint16_t>(bufferSize()));
  flipRowsInPlace(buf);
  if (invert) invertBuffer(buf);
}

void X3Panel::fillPlaneX3(EInkDisplay& d, uint8_t ramCmd, uint8_t fillByte) const {
  // Fill an entire RAM plane with a constant byte. Streams a small stack
  // row buffer repeatedly inside a single SPI transaction so the
  // framebuffer (~50 KB) doesn't need to be touched or memset.
  uint8_t rowBuf[128];
  memset(rowBuf, fillByte, displayWidthBytes());
  d.sendCommand(ramCmd);
  SPI.beginTransaction(d.spiSettings);
  digitalWrite(d._dc, HIGH);
  digitalWrite(d._cs, LOW);
  for (uint16_t y = 0; y < displayHeight(); y++) {
    SPI.writeBytes(rowBuf, displayWidthBytes());
  }
  digitalWrite(d._cs, HIGH);
  SPI.endTransaction();
}

void X3Panel::loadLutBankX3(EInkDisplay& d, const uint8_t* vcom, const uint8_t* ww, const uint8_t* bw,
                            const uint8_t* wb, const uint8_t* bb) const {
  sendCommandDataX3(d, CMD_X3_LUT_VCOM, vcom, 42);
  sendCommandDataX3(d, CMD_X3_LUT_WW, ww, 42);
  sendCommandDataX3(d, CMD_X3_LUT_BW, bw, 42);
  sendCommandDataX3(d, CMD_X3_LUT_WB, wb, 42);
  sendCommandDataX3(d, CMD_X3_LUT_BB, bb, 42);
}

void X3Panel::loadLutBankX3WithCdi(EInkDisplay& d, uint8_t cdi0, const uint8_t* vcom, const uint8_t* ww,
                                   const uint8_t* bw, const uint8_t* wb, const uint8_t* bb) const {
  sendCommandDataByteX3(d, CMD_X3_VCOM_DATA_INTERVAL, cdi0);
  loadLutBankX3(d, vcom, ww, bw, wb, bb);
}

void X3Panel::loadLutBankX3WithCdi(EInkDisplay& d, uint8_t cdi0, uint8_t cdi1, const uint8_t* vcom, const uint8_t* ww,
                                   const uint8_t* bw, const uint8_t* wb, const uint8_t* bb) const {
  sendCommandDataByteX3(d, CMD_X3_VCOM_DATA_INTERVAL, cdi0, cdi1);
  loadLutBankX3(d, vcom, ww, bw, wb, bb);
}

void X3Panel::triggerRefreshX3(EInkDisplay& d, bool turnOffScreen, const char* tag) const {
  if (!d.isScreenOn) {
    d.sendCommand(CMD_X3_POWER_ON);
    char buf[32];
    snprintf(buf, sizeof(buf), " X3_PON%s", tag);
    d.waitForRefresh(buf);
    d.isScreenOn = true;
  }
  if (Serial) Serial.printf("[%lu]   X3_OEM_TRIGGER=DRF%s\n", millis(), tag);
  d.sendCommand(CMD_X3_DISPLAY_REFRESH);
  {
    char buf[32];
    snprintf(buf, sizeof(buf), " X3_DRF%s", tag);
    d.waitForRefresh(buf);
  }
  if (turnOffScreen) {
    d.sendCommand(CMD_X3_POWER_OFF);
    char buf[32];
    snprintf(buf, sizeof(buf), " X3_POF%s", tag);
    d.waitForRefresh(buf);
    d.isScreenOn = false;
  }
}

// =====================================================================
