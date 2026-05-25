#pragma once

#include <cstdint>

class EInkDisplay;  // forward decl — pollBusy takes EInkDisplay& by friendship

// Which grayscale plane a writeGrayscalePlaneStrip call targets.
// Underlying uint8_t keeps any struct field holding this enum to one
// byte; the implicit int default would inflate cache structures.
enum class GrayPlane : uint8_t { GRAY_PLANE_LSB, GRAY_PLANE_MSB };

// Refresh policy hint passed to Panel::displayBuffer.
//   FULL_REFRESH: complete waveform; strongest drive, slowest.
//   HALF_REFRESH: state-collapsed waveform; cleans ghosting cadence.
//   FAST_REFRESH: differential against the previous frame; cheapest.
// Lives at top level so Panel.h can use it without circular include
// of EInkDisplay.h.
enum class RefreshMode : uint8_t {
  FULL_REFRESH,
  HALF_REFRESH,
  FAST_REFRESH,
};

// Panel owns the per-device differences EInkDisplay needs to know
// about: geometry, controller init sequence, refresh dispatch, LUT
// data, busy-wait policy, grayscale flow, deep-sleep sequence. Adding
// a new panel is additive (a new subclass + a friend declaration);
// no edits to EInkDisplay's dispatch code.
//
// === Design: why friend, not an interface ===
//
// Panel methods reach into EInkDisplay's private SPI / pin / state via
// a `friend class XPanel;` declaration on EInkDisplay (see the body of
// EInkDisplay.h). The alternative — a DisplayContext interface that
// enumerates the hardware ops Panel may call — was considered and
// rejected for this SDK for these reasons:
//
//   1. Hot-path performance. Bulk plane writes (sendPlaneX3,
//      fillPlaneX3) and busy polling iterate inside tight loops.
//      Routing every `sendCommand`/`sendData`/`digitalRead` through a
//      virtual method on a DisplayContext interface adds a vtable
//      lookup per call. The optimizer cannot devirtualize through
//      `Panel*` without whole-program LTO that this project does not
//      uniformly enable. Direct friend access compiles to the same
//      machine code as if the body still lived on EInkDisplay.
//
//   2. The panel set is bounded. There are 2 panels today and at most
//      a small handful ever expected — not 10, not 100. The "edit
//      EInkDisplay.h to friend YPanel" cost is one line per panel,
//      paid by the panel author at the moment they're already adding
//      their subclass.
//
//   3. Same-author maintenance. Panel subclasses live in this SDK
//      alongside EInkDisplay; both are written by the same hands. The
//      encapsulation benefit of "panels can only see what
//      DisplayContext exposes" defends against authors outside the
//      threat model.
//
//   4. EInkDisplay and Panel co-evolve. Treating EInkDisplay's
//      private layout as a stable contract that Panel must abide by
//      would impose synchronization cost without buying isolation —
//      every legitimate Panel change crosses that boundary anyway.
//
// Where this calculus would flip: external consumers writing their
// own Panel implementations against a stable EInkDisplay ABI, or
// first-class mocking of EInkDisplay for testing. Neither is in scope
// for this SDK today.
//
// To add a new panel:
//   1. Subclass Panel, implement the virtuals.
//   2. If your panel needs access to EInkDisplay privates (it almost
//      certainly does — SPI, pins, framebuffer), add `friend class
//      YPanel;` to EInkDisplay's class body.
//   3. Each device-specific method takes `EInkDisplay& d` as its
//      first parameter; reach `d.sendCommand`, `d._cs`, etc. via the
//      friend privilege.
//   4. Wire up panel selection at construction:
//        EInkDisplay d(std::make_unique<YPanel>(), sclk, mosi, cs, dc, rst, busy);
class Panel {
 public:
  virtual ~Panel() = default;

  virtual uint16_t displayWidth() const = 0;
  virtual uint16_t displayHeight() const = 0;

  uint16_t displayWidthBytes() const { return displayWidth() / 8; }
  uint32_t bufferSize() const { return static_cast<uint32_t>(displayWidthBytes()) * displayHeight(); }

  // X3-only state-machine hooks. Default no-op so X4 (and any future
  // panel without a resync state) needs no explicit override. The
  // public-API entry points on EInkDisplay just delegate here.
  virtual void requestResync(uint8_t /*settlePasses*/) {}
  virtual void skipInitialResync() {}

  // Wait for the panel's BUSY line to indicate "operation complete".
  // X4 (SSD1677) is active HIGH: BUSY high while working, drops LOW
  // when done. X3 (UC81xx) is active LOW with a two-phase HIGH→LOW→
  // HIGH transition plus an early-return when no LOW edge is seen.
  // Each panel owns its own polling policy.
  virtual void pollBusy(EInkDisplay& d, const char* comment, const char* completeWord) const = 0;

  // Run the controller-specific init sequence (panel-setting / power /
  // booster / resolution / LUT bank). Called once from
  // EInkDisplay::begin() after the GPIO reset.
  virtual void init(EInkDisplay& d) const = 0;

  // Render the current framebuffer to the panel using `mode`. Owns
  // mode-specific LUT loading, plane writing, refresh trigger, and
  // (for X3) the resync state machine. EInkDisplay::displayBuffer
  // handles only the panel-agnostic wake/grayscale-revert prologue
  // and then delegates here.
  virtual void displayBuffer(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) = 0;

  // Trigger one refresh of the current RAM contents with `mode`. On
  // X4 this drives the SSD1677 power/clock/master-activation sequence
  // directly; on X3 it just routes through displayBuffer (which owns
  // the X3 LUT + plane plumbing).
  virtual void refreshDisplay(EInkDisplay& d, RefreshMode mode, bool turnOffScreen) = 0;

  // Render a 4-level grayscale buffer that was previously deposited via
  // copyGrayscaleBuffers / writeGrayscalePlaneStrip. `lut` is the
  // waveform LUT (nullable — panels pick a default). `factoryMode`
  // selects absolute-mode rendering for image content vs differential
  // mode for text-overlay AA.
  virtual void displayGrayBuffer(EInkDisplay& d, bool turnOffScreen, const unsigned char* lut, bool factoryMode) = 0;

  // Repaint after a differential grayscale leaves the gray bank loaded
  // in the LUT registers. Called by EInkDisplay::displayBuffer's
  // prologue when inGrayscaleMode is set.
  virtual void grayscaleRevert(EInkDisplay& d) = 0;

  // Window-only update: refresh a rectangular region without touching
  // the rest of the screen. X3 currently routes through displayBuffer
  // (full-screen refresh) since the partial-window plumbing wasn't
  // worth a second X3-specific impl; X4 does a real windowed write.
  virtual void displayWindow(EInkDisplay& d, uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool turnOffScreen) = 0;

  // Two-plane grayscale RAM load. Panels know which controller RAM
  // commands correspond to LSB vs MSB and whether a Y-flip is needed.
  virtual void copyGrayscaleLsbBuffers(EInkDisplay& d, const uint8_t* lsbBuffer) = 0;
  virtual void copyGrayscaleMsbBuffers(EInkDisplay& d, const uint8_t* msbBuffer) = 0;
  virtual void copyGrayscaleBuffers(EInkDisplay& d, const uint8_t* lsbBuffer, const uint8_t* msbBuffer) = 0;

  // Streaming variant: load one horizontal band of a plane straight to
  // controller RAM without a full plane buffer in MCU heap. Bands may
  // be submitted in any order.
  virtual void writeGrayscalePlaneStrip(EInkDisplay& d, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                        uint16_t numRows) = 0;

  // SINGLE_BUFFER_MODE: re-sync the RAM planes from a restored BW
  // buffer so the next fast diff has a valid baseline after grayscale.
  // Default no-op for panels that don't need it (or that operate in
  // dual-buffer mode where the host preserves the previous frame).
  virtual void cleanupGrayscaleBuffers(EInkDisplay& /*d*/, const uint8_t* /*bwBuffer*/) {}

  // Custom LUT injection for the X4 SSD1677 waveform registers. X3
  // doesn't expose registers in this shape (its LUTs go through the
  // banked loadLutBankX3 path); X3Panel inherits the no-op default.
  virtual void setCustomLUT(EInkDisplay& /*d*/, bool /*enabled*/, const unsigned char* /*lutData*/ = nullptr) {}

  // Park the controller in its lowest-power state. Each panel knows
  // its own deep-sleep opcode sequence.
  virtual void deepSleep(EInkDisplay& d) = 0;

  // SPI clock rate the panel can sustain. SSD1677 tops out at 40 MHz
  // on this hardware; UC81xx caps at 16 MHz.
  virtual uint32_t spiClockHz() const = 0;

  // How many forced FULL refreshes the panel needs at the top of
  // begin() to settle into a clean state. Defaults to 0 (no warmup);
  // X3 overrides to 2 because its UC81xx controller can latch garbled
  // content from the prior session into the first user-visible diff.
  virtual uint8_t initialFullSyncsAfterBegin() const { return 0; }

  // Extra settle time after the GPIO reset pulse before init() runs.
  // Default 0 (rely on the reset pulse itself); X3 overrides with 50ms.
  virtual void postResetDelay() const {}

  // Reset per-panel state at begin() time. Called after framebuffer
  // memset but before init(). X3 uses this to zero its resync state
  // machine + arm initialFullSyncsAfterBegin(); other panels can leave
  // the default empty.
  virtual void onBegin() {}
};

// SSD1677, 800x480 mono. The original X4 hardware.
class X4Panel : public Panel {
 public:
  uint16_t displayWidth() const override { return 800; }
  uint16_t displayHeight() const override { return 480; }
  uint32_t spiClockHz() const override { return 40000000; }

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
  void setCustomLUT(EInkDisplay& d, bool enabled, const unsigned char* lutData = nullptr) override;
  void deepSleep(EInkDisplay& d) override;
};

// X3Panel (UC81xx-class, 792x528) lives in X3Panel.h since it carries
// the X3-specific SPI helper surface.
