
#include <Arduino.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <string.h>

extern "C" char __heap_start;
extern "C" void *__brkval;

/*
ATmega328P Xplained Mini demo
Board peripherals used:
- User LED (PB5 / D13)
- User push button (PB7 / D21), active-low
- mEDBG virtual COM port via UART (PD0/PD1) using Serial
*/

namespace {

constexpr uint8_t kLedPin = LED_BUILTIN; // PB5 / D13
#if defined(PIN_PB7)
constexpr uint8_t kButtonPin = PIN_PB7; // PB7 / D21
#else
constexpr uint8_t kButtonPin = 21;
#endif

#ifndef DEMO_BAUD
#define DEMO_BAUD 57600UL
#endif
constexpr uint32_t kBaudRate = DEMO_BAUD;
constexpr uint32_t kDebounceMs = 25;
constexpr uint32_t kLongPressMs = 1200;
constexpr size_t kCmdBufferSize = 64;
constexpr size_t kHistorySize = 8;
constexpr char kPrompt[] = "arduino$ ";
constexpr char kBoardName[] = "ATmega328P Xplained Mini";
#ifndef FW_VERSION
#define FW_VERSION "1.1.0"
#endif

struct HeartbeatPhase {
  bool level;
  uint16_t durationMs;
};

constexpr HeartbeatPhase kHeartbeatPattern[] = {
  {true, 70},  // beat 1
  {false, 120},
  {true, 70},  // beat 2
  {false, 740} // pause before next heartbeat
};
constexpr size_t kHeartbeatPatternLen = sizeof(kHeartbeatPattern) / sizeof(kHeartbeatPattern[0]);
enum class EscState : uint8_t { None, SeenEsc, SeenEscBracket };

enum class LedMode : uint8_t { Heartbeat, On, Off };

LedMode gLedMode = LedMode::Heartbeat;
bool gHeartbeatLevel = false;
bool gLedOutputLevel = false;
uint32_t gLastHeartbeatMs = 0;
size_t gHeartbeatPhase = 0;

bool gLastButtonSample = HIGH;
bool gStableButtonState = HIGH;
uint32_t gLastDebounceMs = 0;
uint32_t gPressStartMs = 0;
bool gLongPressReported = false;

uint32_t gPressCount = 0;
uint32_t gReleaseCount = 0;
uint32_t gLongPressCount = 0;
uint8_t gResetFlags = 0;

char gCmdBuffer[kCmdBufferSize];
size_t gCmdLen = 0;
char gHistory[kHistorySize][kCmdBufferSize];
size_t gHistoryCount = 0;
size_t gHistoryHead = 0;
int gHistoryCursor = -1;
char gEditBackup[kCmdBufferSize];
size_t gEditBackupLen = 0;
EscState gEscState = EscState::None;

bool buttonIsPressed() { return gStableButtonState == LOW; }

void printPrompt() { Serial.print(kPrompt); }

void print2Digits(uint32_t value) {
  if (value < 10) {
    Serial.write('0');
  }
  Serial.print(value);
}

void print3Digits(uint32_t value) {
  if (value < 100) {
    Serial.write('0');
  }
  if (value < 10) {
    Serial.write('0');
  }
  Serial.print(value);
}

void printHexByte(uint8_t value) {
  const char hex[] = "0123456789ABCDEF";
  Serial.write(hex[(value >> 4) & 0x0F]);
  Serial.write(hex[value & 0x0F]);
}

void printUptimeFormatted(uint32_t ms) {
  const uint32_t totalSeconds = ms / 1000UL;
  const uint32_t days = totalSeconds / 86400UL;
  const uint32_t hours = (totalSeconds / 3600UL) % 24UL;
  const uint32_t minutes = (totalSeconds / 60UL) % 60UL;
  const uint32_t seconds = totalSeconds % 60UL;
  const uint32_t millisPart = ms % 1000UL;

  if (days > 0) {
    Serial.print(days);
    Serial.print(F("d "));
  }
  print2Digits(hours);
  Serial.write(':');
  print2Digits(minutes);
  Serial.write(':');
  print2Digits(seconds);
  Serial.write('.');
  print3Digits(millisPart);
}

void readDeviceSignature(uint8_t outSig[3]) {
  const uint8_t stepped[3] = {
    boot_signature_byte_get(0x00),
    boot_signature_byte_get(0x02),
    boot_signature_byte_get(0x04)};
  outSig[0] = stepped[0];
  outSig[1] = stepped[1];
  outSig[2] = stepped[2];
}

void printResetCause() {
  bool first = true;
  if (gResetFlags & _BV(PORF)) {
    Serial.print(F("POR"));
    first = false;
  }
  if (gResetFlags & _BV(EXTRF)) {
    if (!first) {
      Serial.print(F(", "));
    }
    Serial.print(F("EXTR"));
    first = false;
  }
  if (gResetFlags & _BV(BORF)) {
    if (!first) {
      Serial.print(F(", "));
    }
    Serial.print(F("BOR"));
    first = false;
  }
  if (gResetFlags & _BV(WDRF)) {
    if (!first) {
      Serial.print(F(", "));
    }
    Serial.print(F("WDR"));
    first = false;
  }
  if (first) {
    Serial.print(F("unknown"));
  }
}

void captureResetFlags() {
  gResetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

int freeRamEstimate() {
  int v;
  return (int)&v - (int)(__brkval == nullptr ? &__heap_start : __brkval);
}

bool startsWithIgnoreCase(const char *text, const char *prefix) {
  while (*prefix != '\0') {
    if (*text == '\0') {
      return false;
    }
    const char tc = static_cast<char>(tolower(static_cast<unsigned char>(*text)));
    const char pc = static_cast<char>(tolower(static_cast<unsigned char>(*prefix)));
    if (tc != pc) {
      return false;
    }
    ++text;
    ++prefix;
  }
  return true;
}

void setCmdBuffer(const char *text) {
  strncpy(gCmdBuffer, text, kCmdBufferSize - 1);
  gCmdBuffer[kCmdBufferSize - 1] = '\0';
  gCmdLen = strlen(gCmdBuffer);
}

void redrawInputLine(size_t previousLen) {
  Serial.write('\r');
  printPrompt();
  for (size_t i = 0; i < gCmdLen; ++i) {
    Serial.write(gCmdBuffer[i]);
  }

  if (previousLen > gCmdLen) {
    const size_t extra = previousLen - gCmdLen;
    for (size_t i = 0; i < extra; ++i) {
      Serial.write(' ');
    }
    for (size_t i = 0; i < extra; ++i) {
      Serial.write('\b');
    }
  }
}

const char *historyEntryFromNewest(size_t newestOffset) {
  const size_t idx = (gHistoryHead + kHistorySize - 1 - newestOffset) % kHistorySize;
  return gHistory[idx];
}

void pushHistory(const char *line) {
  if (line[0] == '\0') {
    return;
  }

  if (gHistoryCount > 0 && strcmp(historyEntryFromNewest(0), line) == 0) {
    return;
  }

  strncpy(gHistory[gHistoryHead], line, kCmdBufferSize - 1);
  gHistory[gHistoryHead][kCmdBufferSize - 1] = '\0';
  gHistoryHead = (gHistoryHead + 1) % kHistorySize;
  if (gHistoryCount < kHistorySize) {
    ++gHistoryCount;
  }
}

void resetHistoryBrowse() {
  gHistoryCursor = -1;
  gEditBackupLen = 0;
  gEditBackup[0] = '\0';
}

void historyUp() {
  if (gHistoryCount == 0) {
    return;
  }

  const size_t previousLen = gCmdLen;
  if (gHistoryCursor < 0) {
    strncpy(gEditBackup, gCmdBuffer, kCmdBufferSize - 1);
    gEditBackup[kCmdBufferSize - 1] = '\0';
    gEditBackupLen = gCmdLen;
    gHistoryCursor = 0;
  } else if ((size_t)(gHistoryCursor + 1) < gHistoryCount) {
    ++gHistoryCursor;
  }

  setCmdBuffer(historyEntryFromNewest((size_t)gHistoryCursor));
  redrawInputLine(previousLen);
}

void historyDown() {
  if (gHistoryCursor < 0) {
    return;
  }

  const size_t previousLen = gCmdLen;
  if (gHistoryCursor > 0) {
    --gHistoryCursor;
    setCmdBuffer(historyEntryFromNewest((size_t)gHistoryCursor));
  } else {
    gHistoryCursor = -1;
    setCmdBuffer(gEditBackup);
    gCmdLen = gEditBackupLen;
    gCmdBuffer[gCmdLen] = '\0';
  }

  redrawInputLine(previousLen);
}

void resetHeartbeat() {
  gHeartbeatPhase = 0;
  gHeartbeatLevel = kHeartbeatPattern[gHeartbeatPhase].level;
  gLastHeartbeatMs = millis();
}

const __FlashStringHelper *ledModeName(LedMode mode) {
  switch (mode) {
    case LedMode::Heartbeat:
      return F("heartbeat");
    case LedMode::On:
      return F("on");
    case LedMode::Off:
      return F("off");
  }
  return F("unknown");
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  help                - show this help"));
  Serial.println(F("  status              - show live demo status"));
  Serial.println(F("  ver                 - firmware/build info"));
  Serial.println(F("  id                  - board + MCU signature"));
  Serial.println(F("  uptime              - formatted uptime"));
  Serial.println(F("  free                - free RAM estimate"));
  Serial.println(F("  reset               - watchdog software reset"));
  Serial.println(F("  echo <text>         - echo text back"));
  Serial.println(F("  led on|off|hb       - set LED mode"));
  Serial.println(F("  led toggle          - toggle ON/OFF mode"));
  Serial.println(F("  counters reset      - reset button counters"));
  Serial.println();
}

void printStatus() {
  Serial.println(F("\n=== Xplained Mini Status ==="));
  Serial.print(F("Uptime [ms]: "));
  const uint32_t upMs = millis();
  Serial.print(upMs);
  Serial.print(F(" ("));
  printUptimeFormatted(upMs);
  Serial.println(F(")"));
  Serial.print(F("LED mode: "));
  Serial.println(ledModeName(gLedMode));
  Serial.print(F("LED level: "));
  Serial.println(gLedOutputLevel ? F("HIGH") : F("LOW"));
  Serial.print(F("Button: "));
  Serial.println(buttonIsPressed() ? F("PRESSED") : F("released"));
  Serial.print(F("Presses: "));
  Serial.println(gPressCount);
  Serial.print(F("Releases: "));
  Serial.println(gReleaseCount);
  Serial.print(F("Long presses: "));
  Serial.println(gLongPressCount);
  Serial.println(F("============================\n"));
}

void normalize(char *s) {
  size_t w = 0;
  bool prevSpace = true;

  for (size_t r = 0; s[r] != '\0'; ++r) {
    char c = static_cast<char>(tolower(static_cast<unsigned char>(s[r])));
    if (c == '\t') {
      c = ' ';
    }
    if (c == ' ') {
      if (prevSpace) {
        continue;
      }
      prevSpace = true;
      s[w++] = c;
    } else {
      prevSpace = false;
      s[w++] = c;
    }
  }

  if (w > 0 && s[w - 1] == ' ') {
    --w;
  }
  s[w] = '\0';
}

void setLedMode(LedMode mode) {
  gLedMode = mode;
  if (gLedMode == LedMode::Heartbeat) {
    resetHeartbeat();
  }
  Serial.print(F("LED mode -> "));
  Serial.println(ledModeName(gLedMode));
}

void handleCommand(char *line) {
  char raw[kCmdBufferSize];
  strncpy(raw, line, kCmdBufferSize - 1);
  raw[kCmdBufferSize - 1] = '\0';

  char *trimmed = raw;
  while (*trimmed != '\0' && isspace(static_cast<unsigned char>(*trimmed))) {
    ++trimmed;
  }

  size_t end = strlen(trimmed);
  while (end > 0 && isspace(static_cast<unsigned char>(trimmed[end - 1]))) {
    trimmed[end - 1] = '\0';
    --end;
  }

  if (trimmed[0] == '\0') {
    return;
  }

  char cmd[kCmdBufferSize];
  strncpy(cmd, trimmed, kCmdBufferSize - 1);
  cmd[kCmdBufferSize - 1] = '\0';
  normalize(cmd);

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }
  if (strcmp(cmd, "ver") == 0) {
    Serial.println(F("\n=== Firmware Info ==="));
    Serial.print(F("Version: "));
    Serial.println(FW_VERSION);
    Serial.print(F("Build: "));
    Serial.print(F(__DATE__));
    Serial.write(' ');
    Serial.println(F(__TIME__));
    Serial.print(F("Board: "));
    Serial.println(kBoardName);
    Serial.print(F("MCU: "));
    Serial.println(F("ATmega328P"));
    Serial.print(F("F_CPU: "));
    Serial.print(F_CPU);
    Serial.println(F(" Hz"));
    Serial.print(F("UART baud: "));
    Serial.println(kBaudRate);
    Serial.print(F("Compiler: "));
    Serial.println(__VERSION__);
    Serial.print(F("Reset cause: "));
    printResetCause();
    Serial.println();
    Serial.println(F("=====================\n"));
    return;
  }
  if (strcmp(cmd, "id") == 0) {
    uint8_t sig[3] = {0, 0, 0};
    readDeviceSignature(sig);
    Serial.print(F("Board: "));
    Serial.println(kBoardName);
    Serial.print(F("Device ID: 0x"));
    printHexByte(sig[0]);
    printHexByte(sig[1]);
    printHexByte(sig[2]);
    Serial.println();
    return;
  }
  if (strcmp(cmd, "uptime") == 0) {
    const uint32_t upMs = millis();
    Serial.print(F("Uptime: "));
    printUptimeFormatted(upMs);
    Serial.print(F(" ("));
    Serial.print(upMs);
    Serial.println(F(" ms)"));
    return;
  }
  if (strcmp(cmd, "free") == 0) {
    Serial.print(F("Free RAM (estimate): "));
    Serial.print(freeRamEstimate());
    Serial.println(F(" bytes"));
    return;
  }
  if (strcmp(cmd, "reset") == 0) {
    Serial.println(F("Resetting via watchdog..."));
    Serial.flush();
    delay(20);
    wdt_enable(WDTO_15MS);
    while (true) {
    }
  }
  if (startsWithIgnoreCase(trimmed, "echo")) {
    const char *text = trimmed + 4;
    if (*text != '\0' && !isspace(static_cast<unsigned char>(*text))) {
      Serial.print(F("Unknown command: "));
      Serial.println(trimmed);
      Serial.println(F("Type 'help'"));
      return;
    }
    while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
      ++text;
    }
    Serial.println(text);
    return;
  }
  if (strcmp(cmd, "led on") == 0) {
    setLedMode(LedMode::On);
    return;
  }
  if (strcmp(cmd, "led off") == 0) {
    setLedMode(LedMode::Off);
    return;
  }
  if (strcmp(cmd, "led hb") == 0 || strcmp(cmd, "led heartbeat") == 0) {
    setLedMode(LedMode::Heartbeat);
    return;
  }
  if (strcmp(cmd, "led toggle") == 0) {
    if (gLedMode == LedMode::On) {
      setLedMode(LedMode::Off);
    } else {
      setLedMode(LedMode::On);
    }
    return;
  }
  if (strcmp(cmd, "counters reset") == 0) {
    gPressCount = 0;
    gReleaseCount = 0;
    gLongPressCount = 0;
    Serial.println(F("Counters reset."));
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(trimmed);
  Serial.println(F("Type 'help'"));
}

void updateSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (gEscState == EscState::SeenEsc) {
      gEscState = (c == '[') ? EscState::SeenEscBracket : EscState::None;
      continue;
    }

    if (gEscState == EscState::SeenEscBracket) {
      if (c == 'A') {
        historyUp();
      } else if (c == 'B') {
        historyDown();
      }
      gEscState = EscState::None;
      continue;
    }

    if (c == 0x1B) {
      gEscState = EscState::SeenEsc;
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\b' || c == 127) {
      if (gCmdLen > 0) {
        --gCmdLen;
        Serial.print(F("\b \b"));
      }
      continue;
    }

    if (c == '\n') {
      Serial.println();
      gCmdBuffer[gCmdLen] = '\0';
      pushHistory(gCmdBuffer);
      handleCommand(gCmdBuffer);
      gCmdLen = 0;
      resetHistoryBrowse();
      printPrompt();
      continue;
    }

    if (isprint(static_cast<unsigned char>(c)) && gCmdLen < (kCmdBufferSize - 1)) {
      gCmdBuffer[gCmdLen++] = c;
      Serial.write(c); // Echo typed input.
    }
  }
}

void updateButton() {
  const uint32_t now = millis();
  const bool sample = digitalRead(kButtonPin);

  if (sample != gLastButtonSample) {
    gLastButtonSample = sample;
    gLastDebounceMs = now;
  }

  if ((now - gLastDebounceMs) >= kDebounceMs && gStableButtonState != gLastButtonSample) {
    gStableButtonState = gLastButtonSample;

    if (buttonIsPressed()) {
      gPressStartMs = now;
      gLongPressReported = false;
      ++gPressCount;
      Serial.println(F("[BTN] pressed"));
      printPrompt();
    } else {
      const uint32_t heldMs = now - gPressStartMs;
      ++gReleaseCount;
      Serial.print(F("[BTN] released after "));
      Serial.print(heldMs);
      Serial.println(F(" ms"));
      printPrompt();
    }
  }

  if (buttonIsPressed() && !gLongPressReported && (now - gPressStartMs) >= kLongPressMs) {
    gLongPressReported = true;
    ++gLongPressCount;
    Serial.println(F("[BTN] long press -> LED mode heartbeat"));
    gLedMode = LedMode::Heartbeat;
    resetHeartbeat();
    printPrompt();
  }
}

void updateHeartbeat() {
  const uint32_t now = millis();
  if ((now - gLastHeartbeatMs) >= kHeartbeatPattern[gHeartbeatPhase].durationMs) {
    gHeartbeatPhase = (gHeartbeatPhase + 1) % kHeartbeatPatternLen;
    gHeartbeatLevel = kHeartbeatPattern[gHeartbeatPhase].level;
    gLastHeartbeatMs = now;
  }
}

void updateLed() {
  bool led = false;

  switch (gLedMode) {
    case LedMode::Heartbeat:
      led = gHeartbeatLevel;
      break;
    case LedMode::On:
      led = true;
      break;
    case LedMode::Off:
      led = false;
      break;
  }

  // While the button is held, force the LED on to make interaction obvious.
  if (buttonIsPressed()) {
    led = true;
  }

  gLedOutputLevel = led;
  digitalWrite(kLedPin, led ? HIGH : LOW);
}

} // namespace

void setup() {
  captureResetFlags();
  pinMode(kLedPin, OUTPUT);
  pinMode(kButtonPin, INPUT_PULLUP);
  digitalWrite(kLedPin, LOW);

  gLastButtonSample = digitalRead(kButtonPin);
  gStableButtonState = gLastButtonSample;
  gLastDebounceMs = millis();
  resetHeartbeat();

  Serial.begin(kBaudRate);
  delay(200);

  Serial.println(F("\nATmega328P Xplained Mini command shell"));
  Serial.println(F("By: Dan Tudose"));
  Serial.print(F("Version: "));
  Serial.println(FW_VERSION);
  Serial.print(F("Build: "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));
  Serial.print(F("LED pin: D"));
  Serial.println(kLedPin);
  Serial.print(F("Button pin: D"));
  Serial.println(kButtonPin);
  Serial.println(F("Button is active LOW (pressed = 0)."));
  Serial.println(F("Long press (>1.2s) resets LED mode to heartbeat."));
  printHelp();
  printPrompt();
}

void loop() {
  updateSerial();
  updateButton();
  updateHeartbeat();
  updateLed();
}
