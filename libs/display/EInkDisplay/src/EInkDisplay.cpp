#include "EInkDisplay.h"

#include <cstring>
#include <fstream>
#include <vector>

// setRamArea / writeRamBuffer here still use SSD1677 opcodes (those
// methods are X4-specific helpers reached via friend from X4Panel).
// X3 opcodes and LUT data are X3-only; EInkDisplay touches neither.
#include "X4Constants.h"

void EInkDisplay::setDisplayX3() { _panel = std::make_unique<X3Panel>(); }

// Refresh-wait idle hook — single slot, see setIdleHook header comment.
EInkDisplay::IdleHook EInkDisplay::_idleHook = nullptr;
void* EInkDisplay::_idleHookCtx = nullptr;

void EInkDisplay::setIdleHook(IdleHook hook, void* ctx) {
  _idleHook = hook;
  _idleHookCtx = ctx;
}

void EInkDisplay::requestResync(uint8_t settlePasses) { _panel->requestResync(settlePasses); }

void EInkDisplay::skipInitialResync() { _panel->skipInitialResync(); }

EInkDisplay::EInkDisplay(std::unique_ptr<Panel> panel, int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst,
                         int8_t busy)
    : _sclk(sclk),
      _mosi(mosi),
      _cs(cs),
      _dc(dc),
      _rst(rst),
      _busy(busy),
      _panel(std::move(panel)),
      frameBuffer(nullptr),
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
      frameBufferActive(nullptr),
#endif
      customLutActive(false) {
  if (Serial) Serial.printf("[%lu] EInkDisplay: Constructor called (panel-injected)\n", millis());
  if (Serial)
    Serial.printf("[%lu]   SCLK=%d, MOSI=%d, CS=%d, DC=%d, RST=%d, BUSY=%d\n", millis(), sclk, mosi, cs, dc, rst, busy);
}

// Legacy form — defaults the panel to X4Panel so the deprecated
// setDisplayX3() shim can still swap it before begin().
EInkDisplay::EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy)
    : EInkDisplay(std::make_unique<X4Panel>(), sclk, mosi, cs, dc, rst, busy) {}

void EInkDisplay::begin() {
  if (Serial) Serial.printf("[%lu] EInkDisplay: begin() called\n", millis());

  isScreenOn = false;
  customLutActive = false;
  inGrayscaleMode = false;
  drawGrayscale = false;

  frameBuffer = frameBuffer0;
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  frameBufferActive = frameBuffer1;
#endif

  // Initialize to white
  memset(frameBuffer0, 0xFF, _panel->bufferSize());
  _panel->onBegin();
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  if (Serial) Serial.printf("[%lu]   Static frame buffer (%lu bytes)\n", millis(), _panel->bufferSize());
#else
  memset(frameBuffer1, 0xFF, _panel->bufferSize());
  if (Serial) Serial.printf("[%lu]   Static frame buffers (2 x %lu bytes)\n", millis(), _panel->bufferSize());
#endif

  if (Serial) Serial.printf("[%lu]   Initializing e-ink display driver...\n", millis());

  // Initialize SPI with custom pins
  SPI.begin(_sclk, -1, _mosi, _cs);
  const uint32_t spiHz = _panel->spiClockHz();
  spiSettings = SPISettings(spiHz, MSBFIRST, SPI_MODE0);
  if (Serial) Serial.printf("[%lu]   SPI initialized at %lu Hz, Mode 0\n", millis(), spiHz);

  // Setup GPIO pins
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
  digitalWrite(_rst, HIGH);
  delay(20);
  digitalWrite(_rst, LOW);
  delay(2);
  digitalWrite(_rst, HIGH);
  delay(20);
  if (Serial) Serial.printf("[%lu]   Display reset complete\n", millis());
  _panel->postResetDelay();
}

void EInkDisplay::waitForRefresh(const char* comment) { pollBusy(comment, "Refresh done"); }

void EInkDisplay::pollBusy(const char* comment, const char* completeWord) {
  _panel->pollBusy(*this, comment, completeWord);
}

void EInkDisplay::sendCommand(uint8_t command) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);  // Command mode
  digitalWrite(_cs, LOW);  // Select chip
  SPI.transfer(command);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendData(uint8_t data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);  // Data mode
  digitalWrite(_cs, LOW);   // Select chip
  SPI.transfer(data);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendData(const uint8_t* data, uint16_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);       // Data mode
  digitalWrite(_cs, LOW);        // Select chip
  SPI.writeBytes(data, length);  // Transfer all bytes
  digitalWrite(_cs, HIGH);       // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::waitWhileBusy(const char* comment) { pollBusy(comment, "Wait complete"); }

void EInkDisplay::initDisplayController() { _panel->init(*this); }

void EInkDisplay::setRamArea(const uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  constexpr uint8_t DATA_ENTRY_X_INC_Y_DEC = 0x01;

  // Reverse Y coordinate (gates are reversed on this display)
  y = _panel->displayHeight() - y - h;

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

void EInkDisplay::clearScreen(const uint8_t color) const { memset(frameBuffer, color, _panel->bufferSize()); }

void EInkDisplay::drawImage(const uint8_t* imageData, const uint16_t x, const uint16_t y, const uint16_t w,
                            const uint16_t h, const bool fromProgmem) const {
  if (!frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  // Calculate bytes per line for the image
  const uint16_t imageWidthBytes = w / 8;

  // Copy image data to frame buffer
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t destY = y + row;
    if (destY >= _panel->displayHeight()) break;

    const uint16_t destOffset = destY * _panel->displayWidthBytes() + (x / 8);
    const uint16_t srcOffset = row * imageWidthBytes;

    for (uint16_t col = 0; col < imageWidthBytes; col++) {
      if ((x / 8 + col) >= _panel->displayWidthBytes()) break;

      if (fromProgmem) {
        frameBuffer[destOffset + col] = pgm_read_byte(&imageData[srcOffset + col]);
      } else {
        frameBuffer[destOffset + col] = imageData[srcOffset + col];
      }
    }
  }

  if (Serial) Serial.printf("[%lu]   Image drawn to frame buffer\n", millis());
}

// Draws only black pixels from the image, leaves white pixels clear (unchanged
// in framebuffer)
void EInkDisplay::drawImageTransparent(const uint8_t* imageData, const uint16_t x, const uint16_t y, const uint16_t w,
                                       const uint16_t h, const bool fromProgmem) const {
  if (!frameBuffer) {
    Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  // Calculate bytes per line for the image
  const uint16_t imageWidthBytes = w / 8;

  // Copy only black pixels to frame buffer
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t destY = y + row;
    if (destY >= _panel->displayHeight()) break;

    const uint16_t destOffset = destY * _panel->displayWidthBytes() + (x / 8);
    const uint16_t srcOffset = row * imageWidthBytes;

    for (uint16_t col = 0; col < imageWidthBytes; col++) {
      if ((x / 8 + col) >= _panel->displayWidthBytes()) break;

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

void EInkDisplay::setFramebuffer(const uint8_t* bwBuffer) const { memcpy(frameBuffer, bwBuffer, _panel->bufferSize()); }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
void EInkDisplay::swapBuffers() {
  uint8_t* temp = frameBuffer;
  frameBuffer = frameBufferActive;
  frameBufferActive = temp;
}
#endif

void EInkDisplay::grayscaleRevert() {
  if (!inGrayscaleMode) return;
  inGrayscaleMode = false;
  _panel->grayscaleRevert(*this);
}

void EInkDisplay::copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer) {
  _panel->copyGrayscaleLsbBuffers(*this, lsbBuffer);
}

void EInkDisplay::copyGrayscaleMsbBuffers(const uint8_t* msbBuffer) {
  _panel->copyGrayscaleMsbBuffers(*this, msbBuffer);
}

void EInkDisplay::copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  _panel->copyGrayscaleBuffers(*this, lsbBuffer, msbBuffer);
}

void EInkDisplay::writeGrayscalePlaneStrip(GrayPlane plane, const uint8_t* rows, uint16_t yStart, uint16_t numRows) {
  _panel->writeGrayscalePlaneStrip(*this, plane, rows, yStart, numRows);
}

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
/**
 * In single buffer mode, this should be called with the previously written BW
 * buffer to reconstruct the RED buffer for proper differential fast refreshes
 * following a grayscale display.
 */
void EInkDisplay::cleanupGrayscaleBuffers(const uint8_t* bwBuffer) { _panel->cleanupGrayscaleBuffers(*this, bwBuffer); }
#endif

void EInkDisplay::displayBuffer(RefreshMode mode, const bool turnOffScreen) {
  if (!isScreenOn && !turnOffScreen) {
    // Wake from off: force HALF so the wake transition gets a stronger
    // waveform than a fast differential. Applies to both X4 and X3.
    mode = RefreshMode::HALF_REFRESH;
  }
  if (inGrayscaleMode) grayscaleRevert();
  _panel->displayBuffer(*this, mode, turnOffScreen);
}

// EXPERIMENTAL: Windowed update support
// Displays only a rectangular region of the frame buffer, preserving the rest
// of the screen. Requirements: x and w must be byte-aligned (multiples of 8
// pixels)
void EInkDisplay::displayWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const bool turnOffScreen) {
  _panel->displayWindow(*this, x, y, w, h, turnOffScreen);
}

void EInkDisplay::displayGrayBuffer(const bool turnOffScreen, const unsigned char* lut, const bool factoryMode) {
  _panel->displayGrayBuffer(*this, turnOffScreen, lut, factoryMode);
}

void EInkDisplay::refreshDisplay(const RefreshMode mode, const bool turnOffScreen) {
  _panel->refreshDisplay(*this, mode, turnOffScreen);
}

void EInkDisplay::setCustomLUT(const bool enabled, const unsigned char* lutData) {
  _panel->setCustomLUT(*this, enabled, lutData);
}

void EInkDisplay::deepSleep() { _panel->deepSleep(*this); }

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
