#include "InputManager.h"

#include <Wire.h>

namespace {
constexpr uint8_t MURPHY_TOUCH_ADDR = 0x2e;
constexpr uint8_t MURPHY_TOUCH_OLD_FT_ADDR = 0x38;
constexpr int MURPHY_TOUCH_SDA = 13;
constexpr int MURPHY_TOUCH_SCL = 12;
constexpr int MURPHY_TOUCH_INT = 44;
constexpr int MURPHY_TOUCH_RST = 45;
constexpr uint32_t MURPHY_TOUCH_I2C_HZ = 100000;
constexpr unsigned long MURPHY_TOUCH_POLL_MS = 25;

// CHSC6x-compatible framing used by ESPHome/espp/Zephyr:
// byte 0 is pressed/nonzero, byte 2 is X, byte 4 is Y.
constexpr uint8_t MURPHY_TOUCH_REG_STATUS = 0x00;
constexpr uint8_t MURPHY_TOUCH_FRAME_LEN = 5;
}  // namespace

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
      murphyTouchAvailable(false),
      murphyTouchLastPressed(false),
      murphyTouchLastX(0),
      murphyTouchLastY(0),
      murphyTouchLastLogTime(0),
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

  if (BoardConfig::isMurphyM3()) {
    beginMurphyTouch();
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
    return state | getMurphyTouchState();
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

  return state | getMurphyTouchState();
}

void InputManager::beginMurphyTouch() {
  pinMode(MURPHY_TOUCH_INT, INPUT_PULLUP);
  pinMode(MURPHY_TOUCH_RST, OUTPUT);
  digitalWrite(MURPHY_TOUCH_RST, LOW);
  delay(5);
  digitalWrite(MURPHY_TOUCH_RST, HIGH);
  delay(50);

  Wire.begin(MURPHY_TOUCH_SDA, MURPHY_TOUCH_SCL, MURPHY_TOUCH_I2C_HZ);
  Wire.setTimeOut(4);

  Wire.beginTransmission(MURPHY_TOUCH_ADDR);
  murphyTouchAvailable = (Wire.endTransmission(true) == 0);

  Wire.beginTransmission(MURPHY_TOUCH_OLD_FT_ADDR);
  const bool oldFtAck = (Wire.endTransmission(true) == 0);

#ifdef ENABLE_SERIAL_LOG
  Serial.printf("[%lu] [TOUCH] Murphy touch init chsc=0x%02X ack=%d old_ft=0x%02X ack=%d sda=%d scl=%d int=%d rst=%d\n", millis(),
                MURPHY_TOUCH_ADDR, murphyTouchAvailable, MURPHY_TOUCH_OLD_FT_ADDR, oldFtAck, MURPHY_TOUCH_SDA,
                MURPHY_TOUCH_SCL, MURPHY_TOUCH_INT, MURPHY_TOUCH_RST);
#endif
}

bool InputManager::readMurphyTouch(uint8_t data[5]) {
  Wire.beginTransmission(MURPHY_TOUCH_ADDR);
  Wire.write(MURPHY_TOUCH_REG_STATUS);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t read = Wire.requestFrom(MURPHY_TOUCH_ADDR, MURPHY_TOUCH_FRAME_LEN, static_cast<uint8_t>(true));
  if (read != MURPHY_TOUCH_FRAME_LEN) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < MURPHY_TOUCH_FRAME_LEN; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

uint8_t InputManager::getMurphyTouchState() {
  if (!BoardConfig::isMurphyM3() || !murphyTouchAvailable) {
    return 0;
  }

  static unsigned long lastPoll = 0;
  const unsigned long now = millis();
  if (now - lastPoll < MURPHY_TOUCH_POLL_MS) {
    if (!murphyTouchLastPressed) {
      return 0;
    }
  } else {
    lastPoll = now;
    uint8_t data[MURPHY_TOUCH_FRAME_LEN] = {};
    if (!readMurphyTouch(data)) {
      murphyTouchLastPressed = false;
      return 0;
    }

    murphyTouchLastPressed = data[0] == 1;
    murphyTouchLastX = data[2];
    murphyTouchLastY = data[4];

#ifdef ENABLE_SERIAL_LOG
    if (murphyTouchLastPressed || now - murphyTouchLastLogTime > 2000) {
      Serial.printf("[%lu] [TOUCH] chsc6x raw=%02X %02X %02X %02X %02X pressed=%d x=%u y=%u int=%d\n", now, data[0],
                    data[1], data[2], data[3], data[4], murphyTouchLastPressed, murphyTouchLastX, murphyTouchLastY,
                    digitalRead(MURPHY_TOUCH_INT));
      murphyTouchLastLogTime = now;
    }
#endif
  }

  if (!murphyTouchLastPressed) {
    return 0;
  }

  // Temporary global touch navigation until per-view hit targets exist:
  // left edge = Back, top third = Up/PageBack, bottom third = Down/PageForward, center = Confirm.
  if (murphyTouchLastX < 32) {
    return 1 << BTN_BACK;
  }
  if (murphyTouchLastY < 85) {
    return 1 << BTN_UP;
  }
  if (murphyTouchLastY > 170) {
    return 1 << BTN_DOWN;
  }
  return 1 << BTN_CONFIRM;
}

bool InputManager::isDigitalPressed(int8_t pin) const {
  return pin >= 0 && digitalRead(pin) == LOW;
}

void InputManager::update() {
  const unsigned long currentTime = millis();
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

const char* InputManager::getButtonName(const uint8_t buttonIndex) {
  if (buttonIndex <= BTN_POWER) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}

bool InputManager::isPowerButtonPressed() const {
  return isPressed(BTN_POWER);
}
