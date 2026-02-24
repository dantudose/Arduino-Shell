
#include <Arduino.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

extern "C" char __heap_start;
extern "C" void *__brkval;

/* ATmega328P Xplained Mini serial command shell */

namespace {

#ifndef DEMO_BAUD
#define DEMO_BAUD 57600UL
#endif
constexpr uint32_t kBaudRate = DEMO_BAUD;
constexpr size_t kCmdBufferSize = 64;
constexpr size_t kHistorySize = 8;
constexpr uint16_t kWatchPeriodMs = 200;
constexpr uint8_t kUserAnalogCount = 6; // A0..A5 for this shell
constexpr char kPrompt[] = "arduino$ ";
constexpr char kBoardName[] = "ATmega328P Xplained Mini";
#ifndef FW_VERSION
#define FW_VERSION "1.1.0"
#endif

enum class EscState : uint8_t { None, SeenEsc, SeenEscBracket };
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

size_t splitArgs(char *text, char *argv[], size_t maxArgs) {
  size_t argc = 0;
  char *p = text;
  while (*p != '\0' && argc < maxArgs) {
    while (*p == ' ') {
      ++p;
    }
    if (*p == '\0') {
      break;
    }
    argv[argc++] = p;
    while (*p != '\0' && *p != ' ') {
      ++p;
    }
    if (*p == ' ') {
      *p = '\0';
      ++p;
    }
  }
  return argc;
}

bool parseUnsigned(const char *token, unsigned long &value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  value = strtoul(token, &end, 10);
  return *end == '\0';
}

bool parsePinToken(const char *token, int &pin) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  if (token[0] == 'a' || token[0] == 'A') {
    unsigned long idx = 0;
    if (!parseUnsigned(token + 1, idx) || idx >= kUserAnalogCount) {
      return false;
    }
    pin = A0 + static_cast<int>(idx);
    return true;
  }

  unsigned long rawPin = 0;
  if (!parseUnsigned(token, rawPin) || rawPin >= NUM_DIGITAL_PINS) {
    return false;
  }
  pin = static_cast<int>(rawPin);
  return true;
}

bool parseAnalogPinToken(const char *token, uint8_t &analogIndex, int &pin) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  if (token[0] == 'a' || token[0] == 'A') {
    unsigned long idx = 0;
    if (!parseUnsigned(token + 1, idx) || idx >= kUserAnalogCount) {
      return false;
    }
    analogIndex = static_cast<uint8_t>(idx);
    pin = A0 + static_cast<int>(idx);
    return true;
  }

  unsigned long raw = 0;
  if (!parseUnsigned(token, raw)) {
    return false;
  }
  if (raw < kUserAnalogCount) {
    analogIndex = static_cast<uint8_t>(raw);
    pin = A0 + static_cast<int>(raw);
    return true;
  }
  if (raw >= A0 && raw < (A0 + kUserAnalogCount)) {
    analogIndex = static_cast<uint8_t>(raw - A0);
    pin = static_cast<int>(raw);
    return true;
  }
  return false;
}

void printPinLabel(int pin) {
  if (pin >= A0 && pin < (A0 + kUserAnalogCount)) {
    Serial.print('A');
    Serial.print(pin - A0);
    Serial.print(F("/"));
  }
  Serial.print('D');
  Serial.print(pin);
}

bool isPwmCapablePin(int pin) {
#if defined(digitalPinHasPWM)
  return digitalPinHasPWM(pin);
#else
  return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;
#endif
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
  Serial.println(F("  pinmode <pin> <in|out|pullup>"));
  Serial.println(F("  digitalread <pin>"));
  Serial.println(F("  digitalwrite <pin> <0|1>"));
  Serial.println(F("  analogread <A0-A5>"));
  Serial.println(F("  pwm <pin> <0-255>"));
  Serial.println(F("  tone <pin> <freq> [ms]"));
  Serial.println(F("  notone <pin>"));
  Serial.println(F("  pulse <pin> <count> <high_ms> <low_ms>"));
  Serial.println(F("  watch <pin>         - press any key to stop"));
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
  Serial.print(F("Free RAM [bytes]: "));
  Serial.println(freeRamEstimate());
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

  char cmdArgs[kCmdBufferSize];
  strncpy(cmdArgs, cmd, kCmdBufferSize - 1);
  cmdArgs[kCmdBufferSize - 1] = '\0';
  char *argv[8] = {};
  const size_t argc = splitArgs(cmdArgs, argv, 8);

  if (argc > 0 && strcmp(argv[0], "pinmode") == 0) {
    if (argc != 3) {
      Serial.println(F("Usage: pinmode <pin> <in|out|pullup>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    if (strcmp(argv[2], "in") == 0 || strcmp(argv[2], "input") == 0) {
      pinMode(pin, INPUT);
      Serial.print(F("pinMode "));
      printPinLabel(pin);
      Serial.println(F(" -> INPUT"));
      return;
    }
    if (strcmp(argv[2], "out") == 0 || strcmp(argv[2], "output") == 0) {
      pinMode(pin, OUTPUT);
      Serial.print(F("pinMode "));
      printPinLabel(pin);
      Serial.println(F(" -> OUTPUT"));
      return;
    }
    if (strcmp(argv[2], "pullup") == 0 || strcmp(argv[2], "input_pullup") == 0) {
      pinMode(pin, INPUT_PULLUP);
      Serial.print(F("pinMode "));
      printPinLabel(pin);
      Serial.println(F(" -> INPUT_PULLUP"));
      return;
    }
    Serial.println(F("Invalid mode. Use in|out|pullup."));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "digitalread") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: digitalread <pin>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    const int value = digitalRead(pin);
    printPinLabel(pin);
    Serial.print(F(" = "));
    Serial.print(value ? F("HIGH") : F("LOW"));
    Serial.print(F(" ("));
    Serial.print(value ? 1 : 0);
    Serial.println(F(")"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "digitalwrite") == 0) {
    if (argc != 3) {
      Serial.println(F("Usage: digitalwrite <pin> <0|1>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    unsigned long bit = 0;
    if (!parseUnsigned(argv[2], bit) || bit > 1) {
      Serial.println(F("Invalid value. Use 0 or 1."));
      return;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, bit ? HIGH : LOW);
    printPinLabel(pin);
    Serial.print(F(" <= "));
    Serial.println(bit ? F("HIGH") : F("LOW"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "analogread") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: analogread <A0-A5>"));
      return;
    }
    uint8_t analogIndex = 0;
    int pin = -1;
    if (!parseAnalogPinToken(argv[1], analogIndex, pin)) {
      Serial.println(F("Invalid analog pin. Use A0-A5."));
      return;
    }
    const int value = analogRead(pin);
    Serial.print(F("A"));
    Serial.print(analogIndex);
    Serial.print(F(" = "));
    Serial.println(value);
    return;
  }

  if (argc > 0 && strcmp(argv[0], "pwm") == 0) {
    if (argc != 3) {
      Serial.println(F("Usage: pwm <pin> <0-255>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    if (!isPwmCapablePin(pin)) {
      Serial.println(F("Pin is not PWM-capable. Use D3,D5,D6,D9,D10,D11."));
      return;
    }
    unsigned long level = 0;
    if (!parseUnsigned(argv[2], level) || level > 255UL) {
      Serial.println(F("Invalid value. Use 0..255."));
      return;
    }
    pinMode(pin, OUTPUT);
    analogWrite(pin, static_cast<uint8_t>(level));
    printPinLabel(pin);
    Serial.print(F(" PWM <= "));
    Serial.println(level);
    return;
  }

  if (argc > 0 && strcmp(argv[0], "tone") == 0) {
    if (argc != 3 && argc != 4) {
      Serial.println(F("Usage: tone <pin> <freq> [ms]"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    unsigned long freq = 0;
    if (!parseUnsigned(argv[2], freq) || freq == 0 || freq > 65535UL) {
      Serial.println(F("Invalid freq. Use 1..65535 Hz."));
      return;
    }

    if (argc == 4) {
      unsigned long durMs = 0;
      if (!parseUnsigned(argv[3], durMs)) {
        Serial.println(F("Invalid duration ms."));
        return;
      }
      tone(pin, static_cast<unsigned int>(freq), static_cast<unsigned long>(durMs));
      printPinLabel(pin);
      Serial.print(F(" tone "));
      Serial.print(freq);
      Serial.print(F(" Hz for "));
      Serial.print(durMs);
      Serial.println(F(" ms"));
      return;
    }

    tone(pin, static_cast<unsigned int>(freq));
    printPinLabel(pin);
    Serial.print(F(" tone "));
    Serial.print(freq);
    Serial.println(F(" Hz"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "notone") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: notone <pin>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    noTone(pin);
    printPinLabel(pin);
    Serial.println(F(" tone OFF"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "pulse") == 0) {
    if (argc != 5) {
      Serial.println(F("Usage: pulse <pin> <count> <high_ms> <low_ms>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    unsigned long count = 0;
    unsigned long highMs = 0;
    unsigned long lowMs = 0;
    if (!parseUnsigned(argv[2], count) || count == 0) {
      Serial.println(F("Invalid count. Use >= 1."));
      return;
    }
    if (!parseUnsigned(argv[3], highMs) || !parseUnsigned(argv[4], lowMs)) {
      Serial.println(F("Invalid timing values."));
      return;
    }

    pinMode(pin, OUTPUT);
    for (unsigned long i = 0; i < count; ++i) {
      digitalWrite(pin, HIGH);
      delay(highMs);
      digitalWrite(pin, LOW);
      if (i + 1 < count) {
        delay(lowMs);
      }
      if (Serial.available() > 0) {
        while (Serial.available() > 0) {
          Serial.read();
        }
        Serial.println(F("Pulse aborted by keypress."));
        return;
      }
    }
    Serial.println(F("Pulse completed."));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "watch") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: watch <pin>"));
      return;
    }
    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }
    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.print(F("Watching "));
    printPinLabel(pin);
    Serial.println(F(" every 200 ms. Press any key to stop."));
    while (true) {
      const int value = digitalRead(pin);
      printPinLabel(pin);
      Serial.print(F(" = "));
      Serial.print(value ? F("HIGH") : F("LOW"));
      Serial.print(F(" @ "));
      Serial.print(millis());
      Serial.println(F(" ms"));

      const uint32_t start = millis();
      while ((millis() - start) < kWatchPeriodMs) {
        if (Serial.available() > 0) {
          while (Serial.available() > 0) {
            Serial.read();
          }
          Serial.println(F("Watch stopped."));
          return;
        }
        delay(5);
      }
    }
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

} // namespace

void setup() {
  captureResetFlags();

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
  printHelp();
  printPrompt();
}

void loop() {
  updateSerial();
}
