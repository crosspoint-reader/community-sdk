#pragma once

#include <Arduino.h>
#include <BoardConfig.h>

class InputManager {
 public:
  struct TouchPoint {
    bool valid;
    uint16_t x;
    uint16_t y;
    unsigned long timestamp;
  };

  InputManager();
  void begin();
  uint8_t getState();

  /**
   * Updates the button states. Should be called regularly in the main loop.
   */
  void update();
  void clearState();

  /**
   * Returns true if the button was being held at the time of the last #update() call.
   *
   * @param buttonIndex the button indexes
   * @return the button current press state
   */
  bool isPressed(uint8_t buttonIndex) const;

 /**
   * Returns true if the button went from unpressed to pressed between the last two #update() calls.
   *
   * This differs from #isPressed() in that pressing and holding a button will cause this function
   * to return true after the first #update() call, but false on subsequent calls, whereas #isPressed()
   * will continue to return true.
   *
   * @param buttonIndex
   * @return the button pressed state
   */
  bool wasPressed(uint8_t buttonIndex) const;

  /**
   * Returns true if any button started being pressed between the last two #update() calls
   *
   * @return true if any button started being pressed between the last two #update() calls
   */
  bool wasAnyPressed() const;

  /**
   * Returns true if the button went from pressed to unpressed between the last two #update() calls
   *
   * @param buttonIndex the button indexes
   * @return the button release state
   */
  bool wasReleased(uint8_t buttonIndex) const;

  /**
   * Returns true if any button was released between the last two #update() calls
   *
   * @return  true if any button was released between the last two #update() calls
   */
  bool wasAnyReleased() const;

  /**
   * Returns the time between any button starting to be depressed and all buttons between released
   *
   * @return duration in milliseconds
   */
  unsigned long getHeldTime() const;

  /**
   * Returns the time the power button has been held.
   */
  unsigned long getPowerButtonHeldTime() const;

  bool hasTouch() const;
  bool isTouchPressed() const;
  bool wasTouchPressed() const;
  bool wasTouchReleased() const;
  TouchPoint getTouchPoint() const;

  // Button indices
  static constexpr uint8_t BTN_BACK = 0;
  static constexpr uint8_t BTN_CONFIRM = 1;
  static constexpr uint8_t BTN_LEFT = 2;
  static constexpr uint8_t BTN_RIGHT = 3;
  static constexpr uint8_t BTN_UP = 4;
  static constexpr uint8_t BTN_DOWN = 5;
  static constexpr uint8_t BTN_POWER = 6;

  // Pins
  static constexpr int BUTTON_ADC_PIN_1 = 1;
  static constexpr int BUTTON_ADC_PIN_2 = 2;
  static constexpr int POWER_BUTTON_PIN = BoardConfig::ACTIVE.input.power;

  // Power button methods
  bool isPowerButtonPressed() const;

  // Button names
  static const char* getButtonName(uint8_t buttonIndex);

 private:
  int getButtonFromADC(int adcValue, const int ranges[], int numButtons);
  bool isDigitalPressed(int8_t pin) const;
  uint8_t getTouchIrqState();
  bool touchIrqActive(int irqRaw) const;
  void updateTouchFromIrq(unsigned long now, int irqRaw);
  bool readTouchPoint(TouchPoint& point);
  bool decodeTouchFrame(const uint8_t* data, size_t len, TouchPoint& point) const;
  bool decodeMurphyChsc6xFrame(const uint8_t* data, size_t len, TouchPoint& point) const;
  uint16_t mapTouchAxis(uint16_t raw, uint16_t rawMin, uint16_t rawMax, uint16_t outMax) const;

  uint8_t currentState;
  uint8_t lastState;
  uint8_t pressedEvents;
  uint8_t releasedEvents;
  bool touchIrqEnabled;
  bool touchDataEnabled;
  int touchIrqBaseline;
  int touchIrqLast;
  unsigned long touchIrqLastChangeTime;
  unsigned long touchIrqPulseUntil;
  bool touchReadPending;
  unsigned long touchReadAt;
  unsigned long touchReleaseAt;
  bool touchPressed;
  bool touchPressedEvent;
  bool touchReleasedEvent;
  TouchPoint touchPoint;
  unsigned long lastDebounceTime;
  unsigned long buttonPressStart;
  unsigned long buttonPressFinish;
  unsigned long powerButtonPressStart;
  unsigned long powerButtonPressFinish;

  static constexpr int NUM_BUTTONS_1 = 4;
  static const int ADC_RANGES_1[];

  static constexpr int NUM_BUTTONS_2 = 2;
  static const int ADC_RANGES_2[];

  static constexpr int ADC_NO_BUTTON = 3900;
  static constexpr unsigned long DEBOUNCE_DELAY = 5;
  static constexpr unsigned long TOUCH_IRQ_DEBOUNCE_MS = 5;
  static constexpr unsigned long TOUCH_IRQ_PULSE_MS = 120;
  static constexpr unsigned long TOUCH_SAMPLE_DELAY_MS = 8;
  static constexpr uint8_t TOUCH_READ_COMMAND = 0x00;
  static constexpr uint8_t TOUCH_FRAME_SIZE = 16;

  static const char* BUTTON_NAMES[];
};
