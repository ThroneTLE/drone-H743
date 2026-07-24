#include <Arduino.h>
#include <Servo.h>

namespace {

constexpr uint8_t kMotorCount = 2U;
constexpr uint8_t kEscPins[kMotorCount] = {4U, 5U};  // GPIO4/D2, GPIO5/D1.
constexpr uint8_t kOledSdaPin = 0U;  // D3 / GPIO0.
constexpr uint8_t kOledSclPin = 2U;  // D4 / GPIO2.
constexpr uint8_t kOledAddr7 = 0x3CU;
constexpr uint8_t kOledAddrWrite = 0x78U;
constexpr uint8_t kOledI2cDelayUs = 5U;
constexpr uint8_t kOledWidth = 128U;
constexpr uint8_t kOledHeight = 64U;
constexpr uint8_t kOledPages = kOledHeight / 8U;
constexpr uint16_t kEscMinUs = 1100U;
constexpr uint16_t kEscMaxUs = 1940U;
constexpr uint32_t kEscFrameUs = 20000U;
constexpr uint32_t kArmSettleMs = 3000U;
constexpr uint32_t kManualMaxHoldMs = 5000U;
constexpr uint32_t kIdentKeepaliveMs = 1500U;
constexpr uint32_t kIdentMaxRunMs = 180000U;
constexpr uint32_t kOledTestMs = 3000U;
constexpr size_t kLineCapacity = 96U;

enum class OutputMode : uint8_t {
  kDisarmed,
  kArmedIdle,
  kManual,
  kIdentification,
};

Servo esc[kMotorCount];
OutputMode outputMode = OutputMode::kDisarmed;
uint16_t currentPulseUs[kMotorCount] = {0U, 0U};
uint32_t armStartMs = 0U;
uint32_t manualDeadlineMs = 0U;
uint32_t identKeepaliveDeadlineMs = 0U;
uint32_t identRunDeadlineMs = 0U;
uint32_t identNextStepMs = 0U;
uint32_t identSequence = 0U;
uint8_t identMotor = 1U;
uint8_t identMinPercent = 0U;
uint8_t identMaxPercent = 0U;
uint8_t identStepPercent = 0U;
uint8_t identCurrentPercent = 0U;
uint32_t identDwellMs = 0U;
char lineBuffer[kLineCapacity];
size_t lineLength = 0U;
bool oledPresent = false;
uint32_t lastDisplayMs = 0U;
uint32_t oledTestUntilMs = 0U;
char lastEvent[16] = "BOOT";
uint8_t oledBuffer[kOledWidth * kOledPages];

constexpr uint8_t kFont5x7[36][5] PROGMEM = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
    {0x7E, 0x11, 0x11, 0x11, 0x7E},  // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},  // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},  // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C},  // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},  // E
    {0x7F, 0x09, 0x09, 0x09, 0x01},  // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A},  // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F},  // H
    {0x00, 0x41, 0x7F, 0x41, 0x00},  // I
    {0x20, 0x40, 0x41, 0x3F, 0x01},  // J
    {0x7F, 0x08, 0x14, 0x22, 0x41},  // K
    {0x7F, 0x40, 0x40, 0x40, 0x40},  // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},  // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F},  // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E},  // O
    {0x7F, 0x09, 0x09, 0x09, 0x06},  // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E},  // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46},  // R
    {0x46, 0x49, 0x49, 0x49, 0x31},  // S
    {0x01, 0x01, 0x7F, 0x01, 0x01},  // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F},  // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F},  // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F},  // W
    {0x63, 0x14, 0x08, 0x14, 0x63},  // X
    {0x07, 0x08, 0x70, 0x08, 0x07},  // Y
    {0x61, 0x51, 0x49, 0x45, 0x43},  // Z
};

bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

const char *modeName() {
  switch (outputMode) {
    case OutputMode::kDisarmed:
      return "disarmed";
    case OutputMode::kArmedIdle:
      return "armed_idle";
    case OutputMode::kManual:
      return "manual";
    case OutputMode::kIdentification:
      return "ident";
  }
  return "unknown";
}

uint16_t percentToPulse(uint8_t percent) {
  const uint32_t span = kEscMaxUs - kEscMinUs;
  return static_cast<uint16_t>(kEscMinUs + (static_cast<uint32_t>(percent) * span) / 100U);
}

void setLastEvent(const char *text) {
  snprintf(lastEvent, sizeof(lastEvent), "%s", (text != nullptr) ? text : "NONE");
}

bool validMotor(uint8_t motor) {
  return motor <= kMotorCount;
}

void oledDelay() {
  delayMicroseconds(kOledI2cDelayUs);
}

void oledRelease(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

void oledPullLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void oledBusIdle() {
  oledRelease(kOledSdaPin);
  oledRelease(kOledSclPin);
  oledDelay();
}

void oledStart() {
  oledRelease(kOledSdaPin);
  oledRelease(kOledSclPin);
  oledDelay();
  oledPullLow(kOledSdaPin);
  oledDelay();
  oledPullLow(kOledSclPin);
  oledDelay();
}

void oledStop() {
  oledPullLow(kOledSdaPin);
  oledDelay();
  oledRelease(kOledSclPin);
  oledDelay();
  oledRelease(kOledSdaPin);
  oledDelay();
}

void oledWriteRawByte(uint8_t data) {
  for (uint8_t bit = 0U; bit < 8U; ++bit) {
    if ((data & 0x80U) != 0U) {
      oledRelease(kOledSdaPin);
    } else {
      oledPullLow(kOledSdaPin);
    }
    oledDelay();
    oledRelease(kOledSclPin);
    oledDelay();
    oledPullLow(kOledSclPin);
    oledDelay();
    data <<= 1U;
  }

  // Zhongjingyuan's sample clocks ACK but does not require reading it.
  oledRelease(kOledSdaPin);
  oledDelay();
  oledRelease(kOledSclPin);
  oledDelay();
  oledPullLow(kOledSclPin);
  oledDelay();
}

void oledSendBlock(uint8_t control, const uint8_t *data, size_t length) {
  if (!oledPresent || (data == nullptr)) {
    return;
  }

  oledStart();
  oledWriteRawByte(kOledAddrWrite);
  oledWriteRawByte(control);
  for (size_t index = 0U; index < length; ++index) {
    oledWriteRawByte(data[index]);
  }
  oledStop();
}

void oledCommand(uint8_t command) {
  oledSendBlock(0x00U, &command, 1U);
}

uint8_t glyphColumn(char ch, uint8_t column) {
  if ((ch >= 'a') && (ch <= 'z')) {
    ch = static_cast<char>(ch - 'a' + 'A');
  }
  if ((ch >= '0') && (ch <= '9')) {
    return pgm_read_byte(&kFont5x7[ch - '0'][column]);
  }
  if ((ch >= 'A') && (ch <= 'Z')) {
    return pgm_read_byte(&kFont5x7[10 + ch - 'A'][column]);
  }
  switch (ch) {
    case ':':
      return (column == 1U || column == 3U) ? 0x36U : 0x00U;
    case '-':
      return (column >= 1U && column <= 3U) ? 0x08U : 0x00U;
    case '/':
      return static_cast<uint8_t>(0x40U >> column);
    case '%': {
      static const uint8_t percentGlyph[5] = {0x62, 0x64, 0x08, 0x13, 0x23};
      return percentGlyph[column];
    }
    default:
      return 0x00U;
  }
}

void oledClearBuffer() {
  memset(oledBuffer, 0, sizeof(oledBuffer));
}

void oledDrawText(uint8_t row, uint8_t col, const char *text) {
  if ((row >= kOledPages) || (text == nullptr)) {
    return;
  }

  uint16_t x = static_cast<uint16_t>(col) * 6U;
  const uint16_t base = static_cast<uint16_t>(row) * kOledWidth;
  while ((*text != '\0') && (x + 5U < kOledWidth)) {
    for (uint8_t column = 0U; column < 5U; ++column) {
      oledBuffer[base + x + column] = glyphColumn(*text, column);
    }
    if ((x + 5U) < kOledWidth) {
      oledBuffer[base + x + 5U] = 0x00U;
    }
    x += 6U;
    ++text;
  }
}

void oledFlush() {
  if (!oledPresent) {
    return;
  }

  for (uint8_t page = 0U; page < kOledPages; ++page) {
    oledCommand(static_cast<uint8_t>(0xB0U + page));
    oledCommand(0x00);
    oledCommand(0x10);
    for (uint8_t x = 0U; x < kOledWidth; x += 16U) {
      oledSendBlock(0x40U, &oledBuffer[static_cast<uint16_t>(page) * kOledWidth + x], 16U);
    }
    yield();
  }
}

void oledUpdate(bool force = false) {
  char line[24];
  const uint32_t nowMs = millis();
  uint32_t waitMs = 0U;

  if (!oledPresent) {
    return;
  }
  if (oledTestUntilMs != 0U) {
    if (!timeReached(nowMs, oledTestUntilMs)) {
      return;
    }
    oledTestUntilMs = 0U;
  }
  if (!force && ((nowMs - lastDisplayMs) < 500U)) {
    return;
  }
  lastDisplayMs = nowMs;

  if ((outputMode != OutputMode::kDisarmed) && ((nowMs - armStartMs) < kArmSettleMs)) {
    waitMs = kArmSettleMs - (nowMs - armStartMs);
  }

  oledClearBuffer();
  oledDrawText(0, 0, "ESP PWM CAL");
  snprintf(line, sizeof(line), "MODE %s", modeName());
  oledDrawText(1, 0, line);
  snprintf(line, sizeof(line), "M1 %4u US", currentPulseUs[0]);
  oledDrawText(2, 0, line);
  snprintf(line, sizeof(line), "M2 %4u US", currentPulseUs[1]);
  oledDrawText(3, 0, line);
  snprintf(line, sizeof(line), "MOT %u PCT %3u", identMotor, identCurrentPercent);
  oledDrawText(4, 0, line);
  if (waitMs > 0U) {
    snprintf(line, sizeof(line), "WAIT %lu.%lus",
             static_cast<unsigned long>(waitMs / 1000U),
             static_cast<unsigned long>((waitMs % 1000U) / 100U));
  } else {
    snprintf(line, sizeof(line), "SEQ %lu", static_cast<unsigned long>(identSequence));
  }
  oledDrawText(5, 0, line);
  snprintf(line, sizeof(line), "LAST %s", lastEvent);
  oledDrawText(6, 0, line);
  oledDrawText(7, 0, "OLED D3 SDA D4 SCL");
  oledFlush();
}

bool oledBegin() {
  static const uint8_t initCommands[] = {
      0xFD, 0x12, 0xAE, 0xD5, 0xA0, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
      0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0x82, 0xDB, 0x34,
      0xA4, 0xA6,
  };

  oledPresent = true;
  oledBusIdle();
  delay(40);
  for (uint8_t pulse = 0U; pulse < 9U; ++pulse) {
    oledPullLow(kOledSclPin);
    oledDelay();
    oledRelease(kOledSclPin);
    oledDelay();
  }
  oledStop();

  for (uint8_t index = 0U; index < sizeof(initCommands); ++index) {
    oledCommand(initCommands[index]);
  }
  oledClearBuffer();
  oledFlush();
  oledCommand(0xAF);
  oledUpdate(true);
  return true;
}

void reportOled() {
  if (!oledPresent) {
    oledBegin();
  }
  Serial.printf(
      "OLED mode=manual_noack ready=%u addr7=0x%02X addr8_write=0x%02X sda_gpio=%u scl_gpio=%u test_cmd=\"OLED TEST\"\r\n",
      oledPresent ? 1U : 0U, kOledAddr7, kOledAddrWrite, kOledSdaPin, kOledSclPin);
}

void oledShowTestPattern() {
  if (!oledPresent) {
    oledBegin();
  }
  for (uint8_t page = 0U; page < kOledPages; ++page) {
    for (uint8_t x = 0U; x < kOledWidth; ++x) {
      const bool block = ((((x / 8U) + page) & 0x01U) != 0U);
      oledBuffer[static_cast<uint16_t>(page) * kOledWidth + x] = block ? 0xFFU : 0x00U;
    }
  }
  oledFlush();
  oledTestUntilMs = millis() + kOledTestMs;
  setLastEvent("OLEDTEST");
  Serial.printf("OK oled_test duration_ms=%lu\r\n", static_cast<unsigned long>(kOledTestMs));
}

void setPulseForIndex(uint8_t index, uint16_t pulseUs) {
  if (index >= kMotorCount) {
    return;
  }
  if (!esc[index].attached()) {
    esc[index].attach(kEscPins[index], kEscMinUs, kEscMaxUs);
  }
  esc[index].writeMicroseconds(pulseUs);
  currentPulseUs[index] = pulseUs;
}

void setPulse(uint8_t motor, uint16_t pulseUs) {
  if (motor == 0U) {
    for (uint8_t index = 0U; index < kMotorCount; ++index) {
      setPulseForIndex(index, pulseUs);
    }
    return;
  }
  setPulseForIndex(static_cast<uint8_t>(motor - 1U), pulseUs);
}

void disarm(const char *reason) {
  for (uint8_t index = 0U; index < kMotorCount; ++index) {
    esc[index].detach();
    pinMode(kEscPins[index], OUTPUT);
    digitalWrite(kEscPins[index], LOW);
    currentPulseUs[index] = 0U;
  }
  outputMode = OutputMode::kDisarmed;
  setLastEvent(reason);
  manualDeadlineMs = 0U;
  identKeepaliveDeadlineMs = 0U;
  identRunDeadlineMs = 0U;
  identNextStepMs = 0U;
  Serial.printf("OK disarmed reason=%s\r\n", reason);
}

bool isArmed() {
  return outputMode != OutputMode::kDisarmed;
}

bool arm() {
  if (outputMode == OutputMode::kIdentification) {
    Serial.println("ERR busy identification");
    return false;
  }
  setPulse(0U, kEscMinUs);
  outputMode = OutputMode::kArmedIdle;
  armStartMs = millis();
  setLastEvent("ARM");
  Serial.printf("OK armed pulse=%u settle_ms=%lu\r\n", kEscMinUs,
                static_cast<unsigned long>(kArmSettleMs));
  return true;
}

bool armHasSettled(uint32_t nowMs) {
  return isArmed() && (nowMs - armStartMs >= kArmSettleMs);
}

bool parseUnsigned(const char *text, uint32_t *value) {
  char *end = nullptr;
  unsigned long parsed;

  if ((text == nullptr) || (value == nullptr) || (*text == '\0')) {
    return false;
  }
  parsed = strtoul(text, &end, 10);
  if ((*end != '\0') || (parsed > UINT32_MAX)) {
    return false;
  }
  *value = static_cast<uint32_t>(parsed);
  return true;
}

uint8_t splitTokens(char *line, char **tokens, uint8_t maxTokens) {
  uint8_t count = 0U;
  char *save = nullptr;
  char *token = strtok_r(line, " \t", &save);

  while ((token != nullptr) && (count < maxTokens)) {
    tokens[count++] = token;
    token = strtok_r(nullptr, " \t", &save);
  }
  return count;
}

void reportStatus() {
  Serial.printf(
      "STATUS mode=%s armed=%u m1_pulse_us=%u m2_pulse_us=%u arm_settled=%u ident_motor=%u ident_seq=%lu ident_pct=%u\r\n",
      modeName(), isArmed() ? 1U : 0U, currentPulseUs[0], currentPulseUs[1],
      armHasSettled(millis()) ? 1U : 0U, identMotor,
      static_cast<unsigned long>(identSequence), identCurrentPercent);
}

void startManualPulse(uint8_t motor, uint16_t pulseUs, uint32_t holdMs) {
  const uint32_t nowMs = millis();

  if (!validMotor(motor)) {
    Serial.println("ERR motor must be 0, 1 or 2");
    return;
  }
  if (!isArmed()) {
    Serial.println("ERR not armed; send ARM first");
    return;
  }
  if (!armHasSettled(nowMs)) {
    Serial.printf("ERR arm_wait remaining_ms=%lu\r\n",
                  static_cast<unsigned long>(kArmSettleMs - (nowMs - armStartMs)));
    return;
  }
  setPulse(motor, pulseUs);
  outputMode = OutputMode::kManual;
  manualDeadlineMs = nowMs + holdMs;
  setLastEvent("MANUAL");
  Serial.printf("OK manual motor=%u pulse=%u hold_ms=%lu\r\n", motor, pulseUs,
                static_cast<unsigned long>(holdMs));
}

void emitIdentificationSample() {
  const uint16_t pulseUs = percentToPulse(identCurrentPercent);

  setPulse(identMotor, pulseUs);
  Serial.printf("IDENT sample seq=%lu motor=%u pct=%u pulse=%u dwell_ms=%lu ms=%lu\r\n",
                static_cast<unsigned long>(identSequence), identMotor, identCurrentPercent,
                pulseUs, static_cast<unsigned long>(identDwellMs),
                static_cast<unsigned long>(millis()));
  ++identSequence;
}

void stopIdentification(const char *reason) {
  if (outputMode == OutputMode::kIdentification) {
    Serial.printf("IDENT stop reason=%s seq=%lu ms=%lu\r\n", reason,
                  static_cast<unsigned long>(identSequence),
                  static_cast<unsigned long>(millis()));
  }
  disarm(reason);
}

void startIdentification(uint8_t motor, uint8_t minPercent, uint8_t maxPercent,
                         uint8_t stepPercent, uint32_t dwellMs) {
  const uint32_t nowMs = millis();
  const uint32_t stepCount =
      (static_cast<uint32_t>(maxPercent) - minPercent) / stepPercent + 1U;

  if (!validMotor(motor)) {
    Serial.println("ERR motor must be 0, 1 or 2");
    return;
  }
  if (!isArmed()) {
    Serial.println("ERR not armed; send ARM first");
    return;
  }
  if (!armHasSettled(nowMs)) {
    Serial.printf("ERR arm_wait remaining_ms=%lu\r\n",
                  static_cast<unsigned long>(kArmSettleMs - (nowMs - armStartMs)));
    return;
  }
  if ((stepCount * dwellMs) > kIdentMaxRunMs) {
    Serial.printf("ERR ident duration exceeds %lu ms\r\n",
                  static_cast<unsigned long>(kIdentMaxRunMs));
    return;
  }

  identMotor = motor;
  identMinPercent = minPercent;
  identMaxPercent = maxPercent;
  identStepPercent = stepPercent;
  identCurrentPercent = minPercent;
  identDwellMs = dwellMs;
  identSequence = 0U;
  outputMode = OutputMode::kIdentification;
  setLastEvent("IDENT");
  identKeepaliveDeadlineMs = nowMs + kIdentKeepaliveMs;
  identRunDeadlineMs = nowMs + stepCount * dwellMs;
  identNextStepMs = nowMs + dwellMs;

  Serial.printf("IDENT start motor=%u min=%u max=%u step=%u dwell_ms=%lu ms=%lu\r\n",
                identMotor, identMinPercent, identMaxPercent, identStepPercent,
                static_cast<unsigned long>(identDwellMs), static_cast<unsigned long>(nowMs));
  emitIdentificationSample();
}

void updateOutput() {
  const uint32_t nowMs = millis();

  if ((outputMode == OutputMode::kManual) && timeReached(nowMs, manualDeadlineMs)) {
    disarm("manual_timeout");
    return;
  }
  if (outputMode != OutputMode::kIdentification) {
    return;
  }
  if (timeReached(nowMs, identKeepaliveDeadlineMs)) {
    stopIdentification("keepalive_timeout");
    return;
  }
  if (!timeReached(nowMs, identNextStepMs)) {
    return;
  }
  if ((identCurrentPercent + identStepPercent) > identMaxPercent) {
    Serial.printf("IDENT done seq=%lu ms=%lu\r\n", static_cast<unsigned long>(identSequence),
                  static_cast<unsigned long>(nowMs));
    disarm("complete");
    return;
  }
  if (timeReached(nowMs, identRunDeadlineMs)) {
    stopIdentification("duration_timeout");
    return;
  }

  identCurrentPercent = static_cast<uint8_t>(identCurrentPercent + identStepPercent);
  identNextStepMs = nowMs + identDwellMs;
  emitIdentificationSample();
}

void handleLine(char *line) {
  char *tokens[8];
  const uint8_t count = splitTokens(line, tokens, 8U);
  uint32_t value0;
  uint32_t value1;
  uint32_t value2;
  uint32_t value3;
  uint32_t value4;

  if (count == 0U) {
    return;
  }
  if ((strcmp(tokens[0], "STATUS") == 0) || (strcmp(tokens[0], "STATUS?") == 0) ||
      (strcmp(tokens[0], "IDENT?") == 0)) {
    reportStatus();
    return;
  }
  if ((count == 2U) && (strcmp(tokens[0], "OLED") == 0) &&
      (strcmp(tokens[1], "TEST") == 0)) {
    oledShowTestPattern();
    return;
  }
  if ((count == 1U) && ((strcmp(tokens[0], "OLED") == 0) ||
                        (strcmp(tokens[0], "OLED?") == 0))) {
    reportOled();
    return;
  }
  if ((strcmp(tokens[0], "DISARM") == 0) || (strcmp(tokens[0], "STOP") == 0) ||
      ((count == 2U) && (strcmp(tokens[0], "IDENT") == 0) &&
       (strcmp(tokens[1], "STOP") == 0))) {
    stopIdentification("command");
    return;
  }
  if (strcmp(tokens[0], "ARM") == 0) {
    arm();
    return;
  }
  if ((count == 3U) && (strcmp(tokens[0], "PULSE") == 0) &&
      parseUnsigned(tokens[1], &value0) && parseUnsigned(tokens[2], &value1) &&
      (value0 >= kEscMinUs) && (value0 <= kEscMaxUs) && (value1 > 0U) &&
      (value1 <= kManualMaxHoldMs)) {
    startManualPulse(1U, static_cast<uint16_t>(value0), value1);
    return;
  }
  if ((count == 3U) && (strcmp(tokens[0], "PCT") == 0) &&
      parseUnsigned(tokens[1], &value0) && parseUnsigned(tokens[2], &value1) &&
      (value0 <= 100U) && (value1 > 0U) && (value1 <= kManualMaxHoldMs)) {
    startManualPulse(1U, percentToPulse(static_cast<uint8_t>(value0)), value1);
    return;
  }
  if ((count == 4U) && (strcmp(tokens[0], "PULSE") == 0) &&
      parseUnsigned(tokens[1], &value0) && parseUnsigned(tokens[2], &value1) &&
      parseUnsigned(tokens[3], &value2) && (value0 <= kMotorCount) &&
      (value1 >= kEscMinUs) && (value1 <= kEscMaxUs) && (value2 > 0U) &&
      (value2 <= kManualMaxHoldMs)) {
    startManualPulse(static_cast<uint8_t>(value0), static_cast<uint16_t>(value1), value2);
    return;
  }
  if ((count == 4U) && (strcmp(tokens[0], "PCT") == 0) &&
      parseUnsigned(tokens[1], &value0) && parseUnsigned(tokens[2], &value1) &&
      parseUnsigned(tokens[3], &value2) && (value0 <= kMotorCount) &&
      (value1 <= 100U) && (value2 > 0U) && (value2 <= kManualMaxHoldMs)) {
    startManualPulse(static_cast<uint8_t>(value0),
                     percentToPulse(static_cast<uint8_t>(value1)), value2);
    return;
  }
  if ((count == 2U) && (strcmp(tokens[0], "IDENT") == 0) &&
      (strcmp(tokens[1], "KEEPALIVE") == 0)) {
    if (outputMode == OutputMode::kIdentification) {
      identKeepaliveDeadlineMs = millis() + kIdentKeepaliveMs;
    }
    return;
  }
  if ((count == 7U) && (strcmp(tokens[0], "IDENT") == 0) &&
      (strcmp(tokens[1], "START") == 0) && parseUnsigned(tokens[2], &value0) &&
      parseUnsigned(tokens[3], &value1) && parseUnsigned(tokens[4], &value2) &&
      parseUnsigned(tokens[5], &value3) && parseUnsigned(tokens[6], &value4) &&
      (value0 <= kMotorCount) && (value1 <= 100U) && (value2 <= 100U) &&
      (value1 <= value2) && (value3 > 0U) && (value3 <= 100U) &&
      (value4 >= 500U) && (value4 <= 5000U)) {
    startIdentification(static_cast<uint8_t>(value0), static_cast<uint8_t>(value1),
                        static_cast<uint8_t>(value2), static_cast<uint8_t>(value3), value4);
    return;
  }

  setLastEvent("ERR");
  Serial.println(
      "ERR usage ARM|DISARM|STATUS?|PULSE [motor] <1100..1940> <hold_ms>|PCT [motor] "
      "<0..100> <hold_ms>|IDENT START 0|1|2 <min> <max> <step> <dwell_ms>|IDENT "
      "KEEPALIVE|IDENT STOP");
}

void readSerial() {
  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());

    if (input == '\r') {
      continue;
    }
    if (input == '\n') {
      lineBuffer[lineLength] = '\0';
      handleLine(lineBuffer);
      lineLength = 0U;
      continue;
    }
    if (lineLength >= (kLineCapacity - 1U)) {
      lineLength = 0U;
      Serial.println("ERR line too long");
      continue;
    }
    lineBuffer[lineLength++] = input;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(20);
  bool displayOk = oledBegin();
  for (uint8_t index = 0U; index < kMotorCount; ++index) {
    pinMode(kEscPins[index], OUTPUT);
    digitalWrite(kEscPins[index], LOW);
  }
  Serial.printf(
      "ESP8266 PWM calibrator ready m1=GPIO%u/D2 m2=GPIO%u/D1 oled=%u oled_mode=manual_noack addr7=0x%02X addr8_write=0x%02X range=%u..%u us frame=%lu us\r\n",
      kEscPins[0], kEscPins[1], displayOk ? 1U : 0U, kOledAddr7, kOledAddrWrite,
      kEscMinUs, kEscMaxUs, static_cast<unsigned long>(kEscFrameUs));
}

void loop() {
  readSerial();
  updateOutput();
  oledUpdate();
  yield();
}
