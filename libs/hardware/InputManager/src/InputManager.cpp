#include "InputManager.h"

#include <Wire.h>

// Recorded ADC values from real devices
// BACK CONF LEFT RGHT   UP DOWN
// 3597 2760 1530    6 2300    6
// 3470 2666 1480    6 2222    5
// 3470 2655 1470    3 2205    3

// Averages
// BACK CONF LEFT RGHT   UP DOWN
// 3512 2694 1493    5 2242    5

// Setup ranges, if ADC value is between value `i` and `i + 1`, button `i` is being pressed
// These ranges are based on real world values above, and are much more tolerant of different
// devices than a fixed threshold check
// These values are calculated by taking the midpoint of the pairs of averaged values above
const int InputManager::ADC_RANGES_1[] = {ADC_NO_BUTTON, 3100, 2090, 750, INT32_MIN};
const int InputManager::ADC_RANGES_2[] = {ADC_NO_BUTTON, 1120, INT32_MIN};
const char* InputManager::BUTTON_NAMES[] = {"Back", "Confirm", "Left", "Right", "Up", "Down", "Power"};

InputManager::InputManager()
    : currentState(0),
      lastState(0),
      pressedEvents(0),
      releasedEvents(0),
      touchIrqEnabled(false),
      touchDataEnabled(false),
      touchIrqBaseline(HIGH),
      touchIrqLast(HIGH),
      touchIrqLastChangeTime(0),
      touchIrqPulseUntil(0),
      touchReadPending(false),
      touchReadAt(0),
      touchReleaseAt(0),
      touchPressed(false),
      touchPressedEvent(false),
      touchReleasedEvent(false),
      touchPoint({false, 0, 0, 0}),
      lastDebounceTime(0),
      buttonPressStart(0),
      buttonPressFinish(0),
      powerButtonPressStart(0),
      powerButtonPressFinish(0) {}

void InputManager::begin() {
  if (BoardConfig::ACTIVE.inputStyle == BoardConfig::InputStyle::DigitalFiveKey) {
    const int8_t pins[] = {BoardConfig::ACTIVE.input.back, BoardConfig::ACTIVE.input.confirm, BoardConfig::ACTIVE.input.up,
                           BoardConfig::ACTIVE.input.down, BoardConfig::ACTIVE.input.left,    BoardConfig::ACTIVE.input.right,
                           BoardConfig::ACTIVE.input.power};
    for (const int8_t pin : pins) {
      if (pin >= 0) {
        pinMode(pin, INPUT_PULLUP);
      }
    }
  } else {
    pinMode(BUTTON_ADC_PIN_1, INPUT);
    pinMode(BUTTON_ADC_PIN_2, INPUT);
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
    analogSetAttenuation(ADC_11db);
  }

  if (BoardConfig::ACTIVE.hasTouch && BoardConfig::ACTIVE.touch.irq >= 0) {
    pinMode(BoardConfig::ACTIVE.touch.irq, INPUT);
    touchIrqBaseline = digitalRead(BoardConfig::ACTIVE.touch.irq);
    touchIrqLast = touchIrqBaseline;
    touchIrqLastChangeTime = millis();
    touchIrqPulseUntil = 0;
    touchIrqEnabled = true;

    if (BoardConfig::ACTIVE.touch.sda >= 0 && BoardConfig::ACTIVE.touch.scl >= 0 &&
        BoardConfig::ACTIVE.touch.i2cAddress != 0) {
      Wire.begin(BoardConfig::ACTIVE.touch.sda, BoardConfig::ACTIVE.touch.scl, 100000);
      Wire.setTimeOut(4);
      touchDataEnabled = true;
    }
  }
}

int InputManager::getButtonFromADC(const int adcValue, const int ranges[], const int numButtons) {
  for (int i = 0; i < numButtons; i++) {
    if (ranges[i + 1] < adcValue && adcValue <= ranges[i]) {
      return i;
    }
  }

  return -1;
}

uint8_t InputManager::getState() {
  uint8_t state = 0;

  if (BoardConfig::ACTIVE.inputStyle == BoardConfig::InputStyle::DigitalFiveKey) {
    if (isDigitalPressed(BoardConfig::ACTIVE.input.back)) state |= (1 << BTN_BACK);
    if (isDigitalPressed(BoardConfig::ACTIVE.input.confirm)) state |= (1 << BTN_CONFIRM);
    if (isDigitalPressed(BoardConfig::ACTIVE.input.left)) state |= (1 << BTN_LEFT);
    if (isDigitalPressed(BoardConfig::ACTIVE.input.right)) state |= (1 << BTN_RIGHT);
    if (isDigitalPressed(BoardConfig::ACTIVE.input.up)) state |= (1 << BTN_UP);
    if (isDigitalPressed(BoardConfig::ACTIVE.input.down)) state |= (1 << BTN_DOWN);
    if (isDigitalPressed(BoardConfig::ACTIVE.input.power)) state |= (1 << BTN_POWER);
    state |= getTouchIrqState();
    return state;
  }

  // Read GPIO1 buttons
  const int adcValue1 = analogRead(BUTTON_ADC_PIN_1);
  const int button1 = getButtonFromADC(adcValue1, ADC_RANGES_1, NUM_BUTTONS_1);
  if (button1 >= 0) {
    state |= (1 << button1);
  }

  // Read GPIO2 buttons
  const int adcValue2 = analogRead(BUTTON_ADC_PIN_2);
  const int button2 = getButtonFromADC(adcValue2, ADC_RANGES_2, NUM_BUTTONS_2);
  if (button2 >= 0) {
    state |= (1 << (button2 + 4));
  }

  // Read power button (digital, active LOW)
  if (digitalRead(POWER_BUTTON_PIN) == LOW) {
    state |= (1 << BTN_POWER);
  }

  state |= getTouchIrqState();
  return state;
}

bool InputManager::isDigitalPressed(int8_t pin) const {
  return pin >= 0 && digitalRead(pin) == LOW;
}

uint8_t InputManager::getTouchIrqState() {
  if (!touchIrqEnabled) {
    return 0;
  }

  const unsigned long now = millis();
  const int raw = digitalRead(BoardConfig::ACTIVE.touch.irq);
  updateTouchFromIrq(now, raw);

  if (raw != touchIrqLast && now - touchIrqLastChangeTime >= TOUCH_IRQ_DEBOUNCE_MS) {
    touchIrqLast = raw;
    touchIrqLastChangeTime = now;
    if (raw != touchIrqBaseline) {
      touchIrqPulseUntil = now + TOUCH_IRQ_PULSE_MS;
    }
  }

  return now < touchIrqPulseUntil ? (1 << BTN_CONFIRM) : 0;
}

void InputManager::updateTouchFromIrq(const unsigned long now, const int irqRaw) {
  if (!touchDataEnabled) {
    return;
  }

  if (irqRaw != touchIrqLast && now - touchIrqLastChangeTime >= TOUCH_IRQ_DEBOUNCE_MS && irqRaw != touchIrqBaseline) {
    touchReadPending = true;
    touchReadAt = now + TOUCH_SAMPLE_DELAY_MS;
  }

  if (touchReadPending && now >= touchReadAt) {
    TouchPoint point = {false, 0, 0, 0};
    touchReadPending = false;
    if (readTouchPoint(point)) {
      touchPoint = point;
      touchPressed = true;
      touchPressedEvent = true;
      touchReleaseAt = now + TOUCH_IRQ_PULSE_MS;
#ifdef ENABLE_SERIAL_LOG
      Serial.printf("[%lu] [TOUCH] point x=%u y=%u ts=%lu\n", now, point.x, point.y, point.timestamp);
#endif
    }
  }

  if (touchPressed && irqRaw != touchIrqBaseline) {
    touchReleaseAt = now + TOUCH_IRQ_PULSE_MS;
  }

  if (touchPressed && now >= touchReleaseAt) {
    touchPressed = false;
    touchReleasedEvent = true;
  }
}

bool InputManager::readTouchPoint(TouchPoint& point) {
  Wire.beginTransmission(BoardConfig::ACTIVE.touch.i2cAddress);
  Wire.write(TOUCH_READ_COMMAND);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t data[TOUCH_FRAME_SIZE] = {};
  const uint8_t received = Wire.requestFrom(BoardConfig::ACTIVE.touch.i2cAddress, TOUCH_FRAME_SIZE, true);
  if (received != TOUCH_FRAME_SIZE) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < TOUCH_FRAME_SIZE; ++i) {
    data[i] = Wire.read();
  }

  return decodeMurphyTouchFrame(data, TOUCH_FRAME_SIZE, point);
}

bool InputManager::decodeMurphyTouchFrame(const uint8_t* data, const size_t len, TouchPoint& point) const {
  if (len < 7 || (data[0] != 0x00 && data[0] != 0x36)) {
    return false;
  }

  const uint16_t rawX = data[4];
  const uint16_t rawY = (static_cast<uint16_t>(data[5]) << 8) | data[6];
  if ((rawX == 0 && rawY == 0) || (rawX == 0xff && rawY == 0xffff)) {
    return false;
  }

  point.valid = true;
  point.x = mapTouchAxis(rawX, MURPHY_TOUCH_X_MIN, MURPHY_TOUCH_X_MAX, BoardConfig::ACTIVE.displayWidth - 1);
  point.y = mapTouchAxis(rawY, MURPHY_TOUCH_Y_MIN, MURPHY_TOUCH_Y_MAX, BoardConfig::ACTIVE.displayHeight - 1);
  point.timestamp = millis();
  return true;
}

uint16_t InputManager::mapTouchAxis(uint16_t raw, const uint16_t rawMin, const uint16_t rawMax,
                                    const uint16_t outMax) const {
  if (raw <= rawMin) {
    return 0;
  }
  if (raw >= rawMax) {
    return outMax;
  }
  return static_cast<uint32_t>(raw - rawMin) * outMax / (rawMax - rawMin);
}

void InputManager::update() {
  const unsigned long currentTime = millis();

  touchPressedEvent = false;
  touchReleasedEvent = false;

  const uint8_t state = getState();

  // Always clear events first
  pressedEvents = 0;
  releasedEvents = 0;

  // Debounce
  if (state != lastState) {
    lastDebounceTime = currentTime;
    lastState = state;
  }

  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (state != currentState) {
      // Calculate pressed and released events
      pressedEvents = state & ~currentState;
      releasedEvents = currentState & ~state;

      // If pressing buttons and wasn't before, start recording time
      if (pressedEvents > 0 && currentState == 0) {
        buttonPressStart = currentTime;
      }

      // If releasing a button and no other buttons being pressed, record finish time
      if (releasedEvents > 0 && state == 0) {
        buttonPressFinish = currentTime;
      }

      if (pressedEvents & (1 << BTN_POWER)) {
        powerButtonPressStart = currentTime;
      }

      if (releasedEvents & (1 << BTN_POWER)) {
        powerButtonPressFinish = currentTime;
      }

      currentState = state;
    }
  }
}

bool InputManager::isPressed(const uint8_t buttonIndex) const {
  return currentState & (1 << buttonIndex);
}

bool InputManager::wasPressed(const uint8_t buttonIndex) const {
  return pressedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyPressed() const {
  return pressedEvents > 0;
}

bool InputManager::wasReleased(const uint8_t buttonIndex) const {
  return releasedEvents & (1 << buttonIndex);
}

bool InputManager::wasAnyReleased() const {
  return releasedEvents > 0;
}

unsigned long InputManager::getHeldTime() const {
  // Still hold a button
  if (currentState > 0) {
    return millis() - buttonPressStart;
  }

  return buttonPressFinish - buttonPressStart;
}

unsigned long InputManager::getPowerButtonHeldTime() const {
  if (isPressed(BTN_POWER)) {
    return millis() - powerButtonPressStart;
  }

  return powerButtonPressFinish - powerButtonPressStart;
}

bool InputManager::hasTouch() const {
  return touchDataEnabled;
}

bool InputManager::isTouchPressed() const {
  return touchPressed;
}

bool InputManager::wasTouchPressed() const {
  return touchPressedEvent;
}

bool InputManager::wasTouchReleased() const {
  return touchReleasedEvent;
}

InputManager::TouchPoint InputManager::getTouchPoint() const {
  return touchPoint;
}

const char* InputManager::getButtonName(const uint8_t buttonIndex) {
  if (buttonIndex <= BTN_POWER) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}

bool InputManager::isPowerButtonPressed() const {
  return isPressed(BTN_POWER);
}
