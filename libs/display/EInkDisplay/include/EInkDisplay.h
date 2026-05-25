#pragma once
#include <Arduino.h>
#include <SPI.h>

#include <memory>

#include "Panel.h"
#include "X3Panel.h"

class EInkDisplay {
  friend class X3Panel;  // grants X3-side helpers access to SPI/pin/state privates
  friend class X4Panel;  // ditto for X4-side panel logic

 public:
  // Preferred constructor: inject the panel at construction time.
  // Example:
  //   EInkDisplay d(std::make_unique<X3Panel>(), sclk, mosi, cs, dc, rst, busy);
  //   EInkDisplay d(std::make_unique<X4Panel>(), sclk, mosi, cs, dc, rst, busy);
  // `panel` must be non-null; ownership transfers to the display.
  EInkDisplay(std::unique_ptr<Panel> panel, int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy);

  // Legacy constructor. Defaults the panel to X4Panel so code that
  // expects to call setDisplayX3() after construction still works.
  // New code should use the panel-injecting constructor above;
  // setDisplayX3() is deprecated.
  EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy);

  // Destructor
  ~EInkDisplay() = default;

  // RefreshMode lives in Panel.h; reach via the qualified name.

  // Set X3 panel geometry and mode (must be called before begin()).
  // Deprecated: pass the panel to the constructor instead. Kept as a
  // shim so existing callers can migrate at their own pace.
  [[deprecated(
      "Inject the panel at construction: EInkDisplay(std::make_unique<X3Panel>(), sclk, mosi, cs, dc, rst, busy)")]]
  void setDisplayX3();

  // Initialize the display hardware and driver
  void begin();

  // Legacy compile-time dimensions kept for compatibility.
  static constexpr uint16_t DISPLAY_WIDTH = 800;
  static constexpr uint16_t DISPLAY_HEIGHT = 480;
  static constexpr uint16_t DISPLAY_WIDTH_BYTES = DISPLAY_WIDTH / 8;
  static constexpr uint32_t BUFFER_SIZE = DISPLAY_WIDTH_BYTES * DISPLAY_HEIGHT;
  static constexpr uint16_t X3_DISPLAY_WIDTH = 792;
  static constexpr uint16_t X3_DISPLAY_HEIGHT = 528;
  static constexpr uint16_t X3_DISPLAY_WIDTH_BYTES = X3_DISPLAY_WIDTH / 8;
  static constexpr uint32_t X3_BUFFER_SIZE = X3_DISPLAY_WIDTH_BYTES * X3_DISPLAY_HEIGHT;
  static constexpr uint32_t MAX_BUFFER_SIZE = 52272;  // max(800x480, 792x528) / 8

  // Runtime dimensions (delegate to active panel)
  uint16_t getDisplayWidth() const { return _panel->displayWidth(); }
  uint16_t getDisplayHeight() const { return _panel->displayHeight(); }
  uint16_t getDisplayWidthBytes() const { return _panel->displayWidthBytes(); }
  uint32_t getBufferSize() const { return _panel->bufferSize(); }

  // Frame buffer operations
  void clearScreen(uint8_t color = 0xFF) const;
  void drawImage(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                 bool fromProgmem = false) const;
  void drawImageTransparent(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            bool fromProgmem = false) const;
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  void swapBuffers();
#endif
  void setFramebuffer(const uint8_t* bwBuffer) const;

  void copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer);
  void copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer);
  void copyGrayscaleMsbBuffers(const uint8_t* msbBuffer);

  // Stream one horizontal band of a grayscale plane straight to controller RAM
  // so the caller never holds a full plane buffer in MCU heap (the heap win
  // behind tiled grayscale). `plane` selects LSB/MSB RAM; `rows` points at
  // `numRows` physical rows (displayWidthBytes wide) whose top is logical
  // `yStart`. X4 writes each band as an independent windowed RAM write via
  // setRamArea; X3 (UC81xx) windows each band via PTL. Either way bands may be
  // streamed in any order.
  // GrayPlane lives in Panel.h; reach via the qualified name.
  void writeGrayscalePlaneStrip(GrayPlane plane, const uint8_t* rows, uint16_t yStart, uint16_t numRows);

  // True when the tiled/strip grayscale path is supported. X4 (SSD1677) windows
  // each band via setRamArea; X3 (UC81xx) windows via PTL. Both implemented.
  bool supportsStripGrayscale() const { return true; }
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  void cleanupGrayscaleBuffers(const uint8_t* bwBuffer);
#endif

  void displayBuffer(RefreshMode mode = RefreshMode::FAST_REFRESH, bool turnOffScreen = false);
  // EXPERIMENTAL: Windowed update - display only a rectangular region
  void displayWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool turnOffScreen = false);
  void displayGrayBuffer(bool turnOffScreen = false, const unsigned char* lut = nullptr, bool factoryMode = false);

  void refreshDisplay(RefreshMode mode = RefreshMode::FAST_REFRESH, bool turnOffScreen = false);

  // Hint the X3 policy to run a one-shot full resync on next update.
  void requestResync(uint8_t settlePasses = 0);

  // Zero the X3 initial-full-sync counter and mark the RED RAM as already
  // synced. Call after a warm restart (the panel was active ms ago); without
  // this, the first two paints after begin() are promoted to FULL (~770ms
  // each) regardless of the requested mode.
  void skipInitialResync();

  // debug function
  void grayscaleRevert();

  // LUT control
  void setCustomLUT(bool enabled, const unsigned char* lutData = nullptr);

  // Power management
  void deepSleep();

  // Access to frame buffer
  uint8_t* getFrameBuffer() const { return frameBuffer; }

  // Save the current framebuffer to a PBM file (desktop/test builds only)
  void saveFrameBufferAsPBM(const char* filename);

  // Refresh-wait idle hook.
  //
  // The e-ink refresh wait polls the BUSY line in a 1 ms `delay(1)` loop
  // for ~400-700 ms per page turn on typical reader content. That window
  // is CPU-idle and a natural place to run other work — chunked
  // background section indexing in the reader, for example. Callers
  // install a void(void*) function pointer + context via setIdleHook;
  // each panel's BUSY-poll loop invokes it after every 1 ms poll. The
  // hook is intentionally NOT a std::function (binary-size discipline,
  // see open-x4-sdk Resource Protocol).
  //
  // Threading: the hook fires on whatever task calls into the wait (in
  // practice the render task). The hook implementation MUST be safe to
  // call from that task and MUST NOT take more than ~50-100 ms or it
  // delays detection of the BUSY transition. Returning quickly when
  // there is no work to do (one cheap pointer check) is the common case.
  //
  // Storage is a single static slot for the whole SDK — installing a
  // second hook replaces the first.
  using IdleHook = void (*)(void* ctx);
  static void setIdleHook(IdleHook hook, void* ctx);

  // Invoked by Panel::pollBusy implementations from their 1 ms poll
  // loops. Inline for zero overhead on the common null-hook case.
  static inline void tickIdleHook() {
    if (_idleHook) _idleHook(_idleHookCtx);
  }

 private:
  // Pin configuration
  int8_t _sclk, _mosi, _cs, _dc, _rst, _busy;

  // Active panel: source of truth for all per-device behavior.
  // Set by each constructor (X4Panel default for the legacy form,
  // caller-supplied for the panel-injecting form). Never null after
  // construction; the deprecated setDisplayX3() shim swaps it in place.
  std::unique_ptr<Panel> _panel;
  // Frame buffer (statically allocated)
  uint8_t frameBuffer0[MAX_BUFFER_SIZE];
  uint8_t* frameBuffer;
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  uint8_t frameBuffer1[MAX_BUFFER_SIZE];
  uint8_t* frameBufferActive;
#endif

  // SPI settings
  SPISettings spiSettings;

  // State
  bool isScreenOn = false;
  bool customLutActive = false;
  bool inGrayscaleMode = false;
  bool drawGrayscale = false;

  // Idle-hook storage (single slot for the whole SDK; see setIdleHook
  // comment above). Static because the hook policy doesn't depend on
  // which EInkDisplay instance the wait was driven from.
  static IdleHook _idleHook;
  static void* _idleHookCtx;

  // Low-level display control
  void resetDisplay();
  void sendCommand(uint8_t command);
  void sendData(uint8_t data);
  void sendData(const uint8_t* data, uint16_t length);
  void waitForRefresh(const char* comment = nullptr);
  void waitWhileBusy(const char* comment = nullptr);
  // Delegates to Panel::pollBusy. Wait policy (X4 single-phase active-HIGH
  // vs X3 two-phase active-LOW with sawLow early-return) lives on each
  // Panel subclass.
  void pollBusy(const char* comment, const char* completeWord);
  void initDisplayController();

  // Low-level display operations
  void setRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void writeRamBuffer(uint8_t ramBuffer, const uint8_t* data, uint32_t size);

};

// Factory LUTs extracted from firmware V3.1.9_CH_X4_0117.bin.
// Uses absolute 2-bit pixel encoding for single-pass grayscale refresh.
// See EInkDisplay.cpp for encoding details.
extern const unsigned char lut_factory_fast[];     // 110 bytes, 60 frames, FR=0x44
extern const unsigned char lut_factory_quality[];  // 110 bytes, 50 frames, FR=0x22
