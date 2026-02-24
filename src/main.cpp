
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
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
constexpr size_t kMaxArgs = 32;
constexpr size_t kHistorySize = 8;
constexpr uint16_t kWatchPeriodMs = 200;
constexpr uint16_t kDefaultFreqWindowMs = 250;
constexpr uint16_t kMinFreqWindowMs = 10;
constexpr uint16_t kMaxFreqWindowMs = 10000;
constexpr uint8_t kI2cMaxTransferLen = 32; // Wire buffer limit on AVR
constexpr uint32_t kI2cSpeed100kHz = 100000UL;
constexpr uint32_t kI2cSpeed400kHz = 400000UL;
constexpr uint8_t kEepromEraseValue = 0xFF;
constexpr char kEepromEraseToken[] = "confirm";
constexpr uint8_t kFsMagic0 = 'E';
constexpr uint8_t kFsMagic1 = 'F';
constexpr uint8_t kFsMagic2 = 'S';
constexpr uint8_t kFsMagic3 = '1';
constexpr uint8_t kFsVersion = 1;
constexpr uint8_t kFsRootParent = 0xFF;
constexpr uint8_t kFsMaxEntries = 16;
constexpr uint8_t kFsNameBytes = 12; // includes trailing NUL
constexpr uint8_t kFsEntrySize = 20;
constexpr uint16_t kFsHeaderSize = 16;
constexpr uint16_t kFsEntryTableOffset = kFsHeaderSize;
constexpr uint16_t kFsDataStart =
    kFsEntryTableOffset + (static_cast<uint16_t>(kFsMaxEntries) * kFsEntrySize);
constexpr uint8_t kUserAnalogCount = 6; // A0..A5 for this shell
constexpr char kPrompt[] = "arduino$ ";
constexpr char kBoardName[] = "ATmega328P Xplained Mini";
#ifndef FW_VERSION
#define FW_VERSION "1.1.0"
#endif

#ifndef FEATURE_I2C
#define FEATURE_I2C 1
#endif

#ifndef FEATURE_EEPROM
#define FEATURE_EEPROM 1
#endif

#ifndef FEATURE_FS
#define FEATURE_FS 1
#endif

#ifndef FEATURE_TONE
#define FEATURE_TONE 1
#endif

#ifndef FEATURE_LOWLEVEL
#define FEATURE_LOWLEVEL 1
#endif

#if FEATURE_FS && !FEATURE_EEPROM
#error "FEATURE_FS requires FEATURE_EEPROM=1"
#endif

enum class EscState : uint8_t { None, SeenEsc, SeenEscBracket };
uint8_t gResetFlags = 0;
#if FEATURE_I2C
uint32_t gI2cClockHz = kI2cSpeed100kHz;
#endif

char gCmdBuffer[kCmdBufferSize];
size_t gCmdLen = 0;
char gHistory[kHistorySize][kCmdBufferSize];
size_t gHistoryCount = 0;
size_t gHistoryHead = 0;
int gHistoryCursor = -1;
char gEditBackup[kCmdBufferSize];
size_t gEditBackupLen = 0;
EscState gEscState = EscState::None;

struct FsEntry {
  bool used = false;
  bool isDir = false;
  uint8_t parent = kFsRootParent;
  char name[kFsNameBytes] = {0};
  uint16_t dataStart = 0;
  uint16_t dataLen = 0;
};

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

void printHexWord(uint16_t value) {
  printHexByte(static_cast<uint8_t>(value >> 8));
  printHexByte(static_cast<uint8_t>(value & 0xFF));
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

bool equalsIgnoreCase(const char *a, const char *b) {
  if (a == nullptr || b == nullptr) {
    return false;
  }
  while (*a != '\0' && *b != '\0') {
    const char ac = static_cast<char>(tolower(static_cast<unsigned char>(*a)));
    const char bc = static_cast<char>(tolower(static_cast<unsigned char>(*b)));
    if (ac != bc) {
      return false;
    }
    ++a;
    ++b;
  }
  return *a == '\0' && *b == '\0';
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

bool parseUnsignedAuto(const char *token, unsigned long &value) {
  if (token == nullptr || *token == '\0' || token[0] == '-') {
    return false;
  }
  char *end = nullptr;
  value = strtoul(token, &end, 0);
  return *end == '\0';
}

bool parseByteValue(const char *token, uint8_t &value) {
  unsigned long raw = 0;
  if (!parseUnsignedAuto(token, raw) || raw > 0xFFUL) {
    return false;
  }
  value = static_cast<uint8_t>(raw);
  return true;
}

#if FEATURE_LOWLEVEL
bool parseAddressValue(const char *token, uint16_t &value) {
  unsigned long raw = 0;
  if (!parseUnsignedAuto(token, raw) || raw > 0xFFFFUL) {
    return false;
  }
  value = static_cast<uint16_t>(raw);
  return true;
}
#endif

#if FEATURE_I2C
bool parseI2cAddress(const char *token, uint8_t &address) {
  uint8_t raw = 0;
  if (!parseByteValue(token, raw) || raw > 0x7FU) {
    return false;
  }
  address = raw;
  return true;
}

bool parseI2cSpeedToken(const char *token, uint32_t &hz) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  if (strcmp(token, "100k") == 0 || strcmp(token, "100") == 0 ||
      strcmp(token, "100000") == 0) {
    hz = kI2cSpeed100kHz;
    return true;
  }
  if (strcmp(token, "400k") == 0 || strcmp(token, "400") == 0 ||
      strcmp(token, "400000") == 0) {
    hz = kI2cSpeed400kHz;
    return true;
  }
  return false;
}

bool parseI2cLen(const char *token, uint8_t &length) {
  unsigned long raw = 0;
  if (!parseUnsignedAuto(token, raw) || raw == 0 || raw > kI2cMaxTransferLen) {
    return false;
  }
  length = static_cast<uint8_t>(raw);
  return true;
}
#endif

size_t eepromSize() { return static_cast<size_t>(EEPROM.length()); }

#if FEATURE_EEPROM
bool parseEepromAddress(const char *token, uint16_t &address) {
  unsigned long raw = 0;
  if (!parseUnsignedAuto(token, raw)) {
    return false;
  }

  const size_t size = eepromSize();
  if (raw >= size) {
    return false;
  }

  address = static_cast<uint16_t>(raw);
  return true;
}

bool parseEepromLen(const char *token, size_t &length) {
  unsigned long raw = 0;
  if (!parseUnsignedAuto(token, raw) || raw == 0) {
    return false;
  }
  length = static_cast<size_t>(raw);
  return true;
}
#endif

uint16_t eepromReadU16(size_t addr) {
  const uint16_t lo = EEPROM.read(static_cast<int>(addr));
  const uint16_t hi = EEPROM.read(static_cast<int>(addr + 1U));
  return static_cast<uint16_t>(lo | (hi << 8));
}

void eepromWriteU16(size_t addr, uint16_t value) {
  EEPROM.update(static_cast<int>(addr), static_cast<uint8_t>(value & 0xFFU));
  EEPROM.update(static_cast<int>(addr + 1U), static_cast<uint8_t>((value >> 8) & 0xFFU));
}

size_t fsEntryAddress(uint8_t index) {
  return static_cast<size_t>(kFsEntryTableOffset) +
         (static_cast<size_t>(index) * static_cast<size_t>(kFsEntrySize));
}

bool fsIsValidNameToken(const char *name) {
  if (name == nullptr || *name == '\0') {
    return false;
  }
  if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) {
    return false;
  }

  const size_t len = strlen(name);
  if (len == 0 || len >= kFsNameBytes) {
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    const unsigned char c = static_cast<unsigned char>(name[i]);
    if (c < 32 || c == '/') {
      return false;
    }
  }
  return true;
}

void fsSetRootEntry(FsEntry &entry) {
  entry.used = true;
  entry.isDir = true;
  entry.parent = kFsRootParent;
  strncpy(entry.name, "/", kFsNameBytes - 1);
  entry.name[kFsNameBytes - 1] = '\0';
  entry.dataStart = 0;
  entry.dataLen = 0;
}

void fsLoadEntry(uint8_t index, FsEntry &entry) {
  const size_t base = fsEntryAddress(index);
  const uint8_t flags = EEPROM.read(static_cast<int>(base));
  entry.used = (flags & 0x01U) != 0U;
  entry.isDir = (flags & 0x02U) != 0U;
  entry.parent = EEPROM.read(static_cast<int>(base + 1U));

  for (size_t i = 0; i < kFsNameBytes; ++i) {
    entry.name[i] = static_cast<char>(EEPROM.read(static_cast<int>(base + 2U + i)));
  }
  entry.name[kFsNameBytes - 1] = '\0';

  entry.dataStart = eepromReadU16(base + 14U);
  entry.dataLen = eepromReadU16(base + 16U);
}

void fsStoreEntry(uint8_t index, const FsEntry &entry) {
  const size_t base = fsEntryAddress(index);
  uint8_t flags = 0;
  if (entry.used) {
    flags |= 0x01U;
  }
  if (entry.isDir) {
    flags |= 0x02U;
  }

  EEPROM.update(static_cast<int>(base), flags);
  EEPROM.update(static_cast<int>(base + 1U), entry.parent);

  size_t nameLen = strlen(entry.name);
  if (nameLen > (kFsNameBytes - 1U)) {
    nameLen = kFsNameBytes - 1U;
  }
  for (size_t i = 0; i < kFsNameBytes; ++i) {
    const char c = (i < nameLen) ? entry.name[i] : '\0';
    EEPROM.update(static_cast<int>(base + 2U + i), static_cast<uint8_t>(c));
  }

  eepromWriteU16(base + 14U, entry.dataStart);
  eepromWriteU16(base + 16U, entry.dataLen);
  EEPROM.update(static_cast<int>(base + 18U), 0);
  EEPROM.update(static_cast<int>(base + 19U), 0);
}

void fsClearEntry(uint8_t index) {
  const size_t base = fsEntryAddress(index);
  for (size_t i = 0; i < kFsEntrySize; ++i) {
    EEPROM.update(static_cast<int>(base + i), 0);
  }
}

uint16_t fsNextFree() { return eepromReadU16(8U); }

void fsSetNextFree(uint16_t nextFree) { eepromWriteU16(8U, nextFree); }

bool fsIsFormatted() {
  const size_t size = eepromSize();
  if (size <= kFsDataStart) {
    return false;
  }

  if (EEPROM.read(0) != kFsMagic0 || EEPROM.read(1) != kFsMagic1 ||
      EEPROM.read(2) != kFsMagic2 || EEPROM.read(3) != kFsMagic3) {
    return false;
  }
  if (EEPROM.read(4) != kFsVersion) {
    return false;
  }
  if (EEPROM.read(5) != kFsMaxEntries) {
    return false;
  }
  if (eepromReadU16(6U) != kFsDataStart) {
    return false;
  }

  const uint16_t nextFree = fsNextFree();
  return nextFree >= kFsDataStart && nextFree <= size;
}

void fsFormat() {
  EEPROM.update(0, kFsMagic0);
  EEPROM.update(1, kFsMagic1);
  EEPROM.update(2, kFsMagic2);
  EEPROM.update(3, kFsMagic3);
  EEPROM.update(4, kFsVersion);
  EEPROM.update(5, kFsMaxEntries);
  eepromWriteU16(6U, kFsDataStart);
  fsSetNextFree(kFsDataStart);
  for (size_t i = 10; i < kFsHeaderSize; ++i) {
    EEPROM.update(static_cast<int>(i), 0);
  }

  for (uint8_t i = 0; i < kFsMaxEntries; ++i) {
    fsClearEntry(i);
  }
}

#if FEATURE_FS
void fsEnsureInitialized() {
  if (!fsIsFormatted()) {
    fsFormat();
  }
}
#endif

bool fsFindChild(uint8_t parent, const char *name, uint8_t &indexOut, FsEntry &entryOut) {
  for (uint8_t i = 0; i < kFsMaxEntries; ++i) {
    FsEntry entry;
    fsLoadEntry(i, entry);
    if (entry.used && entry.parent == parent && strcmp(entry.name, name) == 0) {
      indexOut = i;
      entryOut = entry;
      return true;
    }
  }
  return false;
}

bool fsFindFreeEntry(uint8_t &indexOut) {
  for (uint8_t i = 0; i < kFsMaxEntries; ++i) {
    FsEntry entry;
    fsLoadEntry(i, entry);
    if (!entry.used) {
      indexOut = i;
      return true;
    }
  }
  return false;
}

bool fsHasChildren(uint8_t parentIndex) {
  for (uint8_t i = 0; i < kFsMaxEntries; ++i) {
    FsEntry entry;
    fsLoadEntry(i, entry);
    if (entry.used && entry.parent == parentIndex) {
      return true;
    }
  }
  return false;
}

bool fsResolvePath(const char *path, uint8_t &indexOut, FsEntry &entryOut) {
  if (path == nullptr) {
    return false;
  }

  char work[kCmdBufferSize];
  strncpy(work, path, kCmdBufferSize - 1);
  work[kCmdBufferSize - 1] = '\0';

  char *start = work;
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) {
    ++start;
  }

  size_t len = strlen(start);
  while (len > 0 && isspace(static_cast<unsigned char>(start[len - 1]))) {
    start[len - 1] = '\0';
    --len;
  }

  while (len > 1 && start[len - 1] == '/') {
    start[len - 1] = '\0';
    --len;
  }

  if (len == 0 || (len == 1 && start[0] == '/')) {
    indexOut = kFsRootParent;
    fsSetRootEntry(entryOut);
    return true;
  }

  while (*start == '/') {
    ++start;
  }
  if (*start == '\0') {
    indexOut = kFsRootParent;
    fsSetRootEntry(entryOut);
    return true;
  }

  uint8_t currentParent = kFsRootParent;
  uint8_t currentIndex = kFsRootParent;
  FsEntry currentEntry;

  char *saveptr = nullptr;
  char *token = strtok_r(start, "/", &saveptr);
  while (token != nullptr) {
    if (!fsIsValidNameToken(token)) {
      return false;
    }
    if (!fsFindChild(currentParent, token, currentIndex, currentEntry)) {
      return false;
    }

    token = strtok_r(nullptr, "/", &saveptr);
    if (token != nullptr && !currentEntry.isDir) {
      return false;
    }
    currentParent = currentIndex;
  }

  indexOut = currentIndex;
  entryOut = currentEntry;
  return true;
}

bool fsResolveDirectory(const char *path, uint8_t &indexOut, FsEntry &entryOut) {
  if (!fsResolvePath(path, indexOut, entryOut)) {
    return false;
  }
  return entryOut.isDir;
}

bool fsSplitParentLeaf(const char *path, char *parentOut, size_t parentOutSize, char *leafOut,
                       size_t leafOutSize) {
  if (path == nullptr || parentOut == nullptr || leafOut == nullptr || parentOutSize == 0 ||
      leafOutSize == 0) {
    return false;
  }

  char work[kCmdBufferSize];
  strncpy(work, path, kCmdBufferSize - 1);
  work[kCmdBufferSize - 1] = '\0';

  char *start = work;
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start))) {
    ++start;
  }

  size_t len = strlen(start);
  while (len > 0 && isspace(static_cast<unsigned char>(start[len - 1]))) {
    start[len - 1] = '\0';
    --len;
  }

  while (len > 1 && start[len - 1] == '/') {
    start[len - 1] = '\0';
    --len;
  }

  if (len == 0 || (len == 1 && start[0] == '/')) {
    return false;
  }

  char *lastSlash = strrchr(start, '/');
  const char *leaf = nullptr;
  const char *parent = nullptr;

  if (lastSlash == nullptr) {
    parent = "/";
    leaf = start;
  } else {
    *lastSlash = '\0';
    leaf = lastSlash + 1;
    parent = (start[0] == '\0') ? "/" : start;
  }

  if (!fsIsValidNameToken(leaf)) {
    return false;
  }

  const size_t leafLen = strlen(leaf);
  const size_t parentLen = strlen(parent);
  if (leafLen >= leafOutSize || parentLen >= parentOutSize) {
    return false;
  }

  strncpy(leafOut, leaf, leafOutSize - 1);
  leafOut[leafOutSize - 1] = '\0';
  strncpy(parentOut, parent, parentOutSize - 1);
  parentOut[parentOutSize - 1] = '\0';
  return true;
}

#if FEATURE_I2C
void setI2cClock(uint32_t hz) {
#if defined(TWBR) && defined(TWPS0) && defined(TWPS1) && defined(F_CPU)
  // Force prescaler=1 and compute TWBR from AVR datasheet formula.
  TWSR = static_cast<uint8_t>(TWSR & ~(_BV(TWPS0) | _BV(TWPS1)));
  const uint32_t twbrValue = ((F_CPU / hz) - 16UL) / 2UL;
  TWBR = static_cast<uint8_t>(twbrValue);
#else
  Wire.setClock(hz);
#endif
  gI2cClockHz = hz;
}
#endif

#if FEATURE_I2C
void printI2cAddress(uint8_t address) {
  Serial.print(F("0x"));
  printHexByte(address);
}

void printI2cTxStatus(uint8_t status) {
  Serial.print(F("I2C error "));
  Serial.print(status);
  Serial.print(F(" ("));
  switch (status) {
    case 1:
      Serial.print(F("data too long"));
      break;
    case 2:
      Serial.print(F("NACK on address"));
      break;
    case 3:
      Serial.print(F("NACK on data"));
      break;
    case 4:
      Serial.print(F("other bus error"));
      break;
    case 5:
      Serial.print(F("timeout"));
      break;
    default:
      Serial.print(F("unknown"));
      break;
  }
  Serial.println(F(")"));
}
#endif

#if FEATURE_LOWLEVEL
enum class PortId : uint8_t { B, C, D };

bool parsePortId(const char *token, PortId &port) {
  if (token == nullptr || *token == '\0') {
    return false;
  }

  if (strcmp(token, "b") == 0 || strcmp(token, "portb") == 0 ||
      strcmp(token, "ddrb") == 0 || strcmp(token, "pinb") == 0) {
    port = PortId::B;
    return true;
  }
  if (strcmp(token, "c") == 0 || strcmp(token, "portc") == 0 ||
      strcmp(token, "ddrc") == 0 || strcmp(token, "pinc") == 0) {
    port = PortId::C;
    return true;
  }
  if (strcmp(token, "d") == 0 || strcmp(token, "portd") == 0 ||
      strcmp(token, "ddrd") == 0 || strcmp(token, "pind") == 0) {
    port = PortId::D;
    return true;
  }
  return false;
}

char portLetter(PortId port) {
  switch (port) {
    case PortId::B:
      return 'B';
    case PortId::C:
      return 'C';
    case PortId::D:
      return 'D';
  }
  return '?';
}

volatile uint8_t &ddrForPort(PortId port) {
  switch (port) {
    case PortId::B:
      return DDRB;
    case PortId::C:
      return DDRC;
    case PortId::D:
      return DDRD;
  }
  return DDRB;
}

volatile uint8_t &portForPort(PortId port) {
  switch (port) {
    case PortId::B:
      return PORTB;
    case PortId::C:
      return PORTC;
    case PortId::D:
      return PORTD;
  }
  return PORTB;
}

volatile uint8_t &pinForPort(PortId port) {
  switch (port) {
    case PortId::B:
      return PINB;
    case PortId::C:
      return PINC;
    case PortId::D:
      return PIND;
  }
  return PINB;
}
#endif

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
  Serial.println(F("  micros              - current micros()"));
  Serial.println(F("  delay <ms>          - blocking delay"));
  Serial.println(F("  freq <pin> [ms]     - estimate input frequency"));
#if FEATURE_I2C
  Serial.println(F("  i2cscan             - scan I2C bus"));
  Serial.println(F("  i2cspeed <100k|400k>"));
  Serial.println(F("  i2cread <addr> <n>"));
  Serial.println(F("  i2cwrite <addr> <bytes...>"));
  Serial.println(F("  i2cwr <addr> <reg> <bytes...>"));
  Serial.println(F("  i2crr <addr> <reg> <n>"));
#endif
#if FEATURE_EEPROM
  Serial.println(F("  eepread <addr> [len]"));
  Serial.println(F("  eepwrite <addr> <bytes...>"));
  Serial.println(F("  eeperase confirm    - clear EEPROM"));
#endif
#if FEATURE_FS
  Serial.println(F("  fs ...              - EEPROM mini filesystem"));
  Serial.println(F("    fs help           - filesystem commands"));
#endif
  Serial.println(F("  pinmode <pin> <in|out|pullup>"));
  Serial.println(F("  digitalread <pin>"));
  Serial.println(F("  digitalwrite <pin> <0|1>"));
  Serial.println(F("  analogread <A0-A5>"));
  Serial.println(F("  pwm <pin> <0-255>"));
#if FEATURE_TONE
  Serial.println(F("  tone <pin> <freq> [ms]"));
  Serial.println(F("  notone <pin>"));
#endif
  Serial.println(F("  pulse <pin> <count> <high_ms> <low_ms>"));
  Serial.println(F("  watch <pin>         - press any key to stop"));
#if FEATURE_LOWLEVEL
  Serial.println(F("  ddr <port> [value]  - view/set DDRx"));
  Serial.println(F("  port <port> [value] - view/set PORTx"));
  Serial.println(F("  pin <port>          - read PINx"));
  Serial.println(F("  peek <addr>         - read memory byte"));
  Serial.println(F("  poke <addr> <val>   - write memory byte"));
  Serial.println(F("  reg                 - dump AVR core registers"));
#endif
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

#if FEATURE_FS
void printFsHelp() {
  Serial.println(F("\nFS commands:"));
  Serial.println(F("  fs help"));
  Serial.println(F("  fs format confirm"));
  Serial.println(F("  fs ls [path]"));
  Serial.println(F("  fs cat <path>"));
  Serial.println(F("  fs mkdir <path>"));
  Serial.println(F("  fs touch <path>"));
  Serial.println(F("  fs write <path> <text>"));
  Serial.println(F("  fs rm <path>"));
  Serial.println(F("  fs stat"));
  Serial.println();
}

void handleFsCommand(const char *rawLine) {
  if (rawLine == nullptr) {
    return;
  }

  char argsLine[kCmdBufferSize];
  strncpy(argsLine, rawLine, kCmdBufferSize - 1);
  argsLine[kCmdBufferSize - 1] = '\0';

  char *argv[kMaxArgs] = {};
  const size_t argc = splitArgs(argsLine, argv, kMaxArgs);
  if (argc == 0 || !equalsIgnoreCase(argv[0], "fs")) {
    return;
  }

  if (argc == 1 || equalsIgnoreCase(argv[1], "help")) {
    printFsHelp();
    return;
  }

  if (equalsIgnoreCase(argv[1], "format")) {
    if (argc != 3 || !equalsIgnoreCase(argv[2], kEepromEraseToken)) {
      Serial.print(F("Usage: fs format "));
      Serial.println(kEepromEraseToken);
      return;
    }
    fsFormat();
    Serial.print(F("FS formatted. Capacity: "));
    Serial.print(eepromSize() - kFsDataStart);
    Serial.println(F(" bytes data."));
    return;
  }

  if (!fsIsFormatted()) {
    Serial.print(F("FS not initialized. Run: fs format "));
    Serial.println(kEepromEraseToken);
    return;
  }

  if (equalsIgnoreCase(argv[1], "ls")) {
    if (argc != 2 && argc != 3) {
      Serial.println(F("Usage: fs ls [path]"));
      return;
    }
    const char *path = (argc == 3) ? argv[2] : "/";
    uint8_t dirIndex = kFsRootParent;
    FsEntry dirEntry;
    if (!fsResolveDirectory(path, dirIndex, dirEntry)) {
      Serial.println(F("Path is not a directory or does not exist."));
      return;
    }

    Serial.print(F("Listing "));
    Serial.println(path);

    uint8_t shown = 0;
    for (uint8_t i = 0; i < kFsMaxEntries; ++i) {
      FsEntry entry;
      fsLoadEntry(i, entry);
      if (!entry.used || entry.parent != dirIndex) {
        continue;
      }
      ++shown;
      Serial.print(entry.isDir ? F("d ") : F("f "));
      Serial.print(entry.name);
      if (!entry.isDir) {
        Serial.print(F(" ("));
        Serial.print(entry.dataLen);
        Serial.print(F("B)"));
      }
      Serial.println();
    }
    if (shown == 0) {
      Serial.println(F("(empty)"));
    }
    return;
  }

  if (equalsIgnoreCase(argv[1], "cat")) {
    if (argc != 3) {
      Serial.println(F("Usage: fs cat <path>"));
      return;
    }
    uint8_t nodeIndex = kFsRootParent;
    FsEntry entry;
    if (!fsResolvePath(argv[2], nodeIndex, entry) || entry.isDir) {
      Serial.println(F("File not found."));
      return;
    }

    if (entry.dataLen == 0) {
      Serial.println(F("(empty file)"));
      return;
    }

    for (uint16_t i = 0; i < entry.dataLen; ++i) {
      const uint8_t value = EEPROM.read(static_cast<int>(entry.dataStart + i));
      if (value == '\n' || value == '\r' || value == '\t' || isprint(value)) {
        Serial.write(value);
      } else {
        Serial.print(F("\\x"));
        printHexByte(value);
      }
    }
    Serial.println();
    return;
  }

  if (equalsIgnoreCase(argv[1], "mkdir")) {
    if (argc != 3) {
      Serial.println(F("Usage: fs mkdir <path>"));
      return;
    }

    char parentPath[kCmdBufferSize];
    char leaf[kFsNameBytes];
    if (!fsSplitParentLeaf(argv[2], parentPath, sizeof(parentPath), leaf, sizeof(leaf))) {
      Serial.println(F("Invalid path."));
      return;
    }

    uint8_t parentIndex = kFsRootParent;
    FsEntry parentEntry;
    if (!fsResolveDirectory(parentPath, parentIndex, parentEntry)) {
      Serial.println(F("Parent directory does not exist."));
      return;
    }

    uint8_t existingIndex = 0;
    FsEntry existingEntry;
    if (fsFindChild(parentIndex, leaf, existingIndex, existingEntry)) {
      Serial.println(F("Path already exists."));
      return;
    }

    uint8_t newIndex = 0;
    if (!fsFindFreeEntry(newIndex)) {
      Serial.println(F("FS entry table full."));
      return;
    }

    FsEntry newEntry;
    newEntry.used = true;
    newEntry.isDir = true;
    newEntry.parent = parentIndex;
    strncpy(newEntry.name, leaf, kFsNameBytes - 1);
    newEntry.name[kFsNameBytes - 1] = '\0';
    fsStoreEntry(newIndex, newEntry);

    Serial.print(F("Directory created: "));
    Serial.println(argv[2]);
    return;
  }

  if (equalsIgnoreCase(argv[1], "touch")) {
    if (argc != 3) {
      Serial.println(F("Usage: fs touch <path>"));
      return;
    }

    char parentPath[kCmdBufferSize];
    char leaf[kFsNameBytes];
    if (!fsSplitParentLeaf(argv[2], parentPath, sizeof(parentPath), leaf, sizeof(leaf))) {
      Serial.println(F("Invalid path."));
      return;
    }

    uint8_t parentIndex = kFsRootParent;
    FsEntry parentEntry;
    if (!fsResolveDirectory(parentPath, parentIndex, parentEntry)) {
      Serial.println(F("Parent directory does not exist."));
      return;
    }

    uint8_t nodeIndex = 0;
    FsEntry nodeEntry;
    if (fsFindChild(parentIndex, leaf, nodeIndex, nodeEntry)) {
      if (nodeEntry.isDir) {
        Serial.println(F("Path exists as directory."));
        return;
      }
      Serial.println(F("File already exists."));
      return;
    }

    if (!fsFindFreeEntry(nodeIndex)) {
      Serial.println(F("FS entry table full."));
      return;
    }

    FsEntry newEntry;
    newEntry.used = true;
    newEntry.isDir = false;
    newEntry.parent = parentIndex;
    strncpy(newEntry.name, leaf, kFsNameBytes - 1);
    newEntry.name[kFsNameBytes - 1] = '\0';
    fsStoreEntry(nodeIndex, newEntry);
    Serial.print(F("File created: "));
    Serial.println(argv[2]);
    return;
  }

  if (equalsIgnoreCase(argv[1], "write")) {
    // Parse path + raw text from the original command to preserve text case and spacing.
    const char *p = rawLine;
    while (*p != '\0' && isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }
    while (*p != '\0' && !isspace(static_cast<unsigned char>(*p))) {
      ++p; // fs
    }
    while (*p != '\0' && isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }
    while (*p != '\0' && !isspace(static_cast<unsigned char>(*p))) {
      ++p; // write
    }
    while (*p != '\0' && isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }
    if (*p == '\0') {
      Serial.println(F("Usage: fs write <path> <text>"));
      return;
    }

    const char *pathStart = p;
    while (*p != '\0' && !isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }
    const size_t pathLen = static_cast<size_t>(p - pathStart);
    if (pathLen == 0 || pathLen >= kCmdBufferSize) {
      Serial.println(F("Invalid path."));
      return;
    }

    char path[kCmdBufferSize];
    memcpy(path, pathStart, pathLen);
    path[pathLen] = '\0';

    while (*p != '\0' && isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }
    const char *text = p; // May be empty.
    const size_t textLen = strlen(text);

    char parentPath[kCmdBufferSize];
    char leaf[kFsNameBytes];
    if (!fsSplitParentLeaf(path, parentPath, sizeof(parentPath), leaf, sizeof(leaf))) {
      Serial.println(F("Invalid path."));
      return;
    }

    uint8_t parentIndex = kFsRootParent;
    FsEntry parentEntry;
    if (!fsResolveDirectory(parentPath, parentIndex, parentEntry)) {
      Serial.println(F("Parent directory does not exist."));
      return;
    }

    uint8_t nodeIndex = 0;
    FsEntry nodeEntry;
    bool exists = fsFindChild(parentIndex, leaf, nodeIndex, nodeEntry);
    if (exists && nodeEntry.isDir) {
      Serial.println(F("Path exists as directory."));
      return;
    }
    if (!exists) {
      if (!fsFindFreeEntry(nodeIndex)) {
        Serial.println(F("FS entry table full."));
        return;
      }
      nodeEntry.used = true;
      nodeEntry.isDir = false;
      nodeEntry.parent = parentIndex;
      strncpy(nodeEntry.name, leaf, kFsNameBytes - 1);
      nodeEntry.name[kFsNameBytes - 1] = '\0';
    }

    if (textLen == 0) {
      nodeEntry.dataLen = 0;
      nodeEntry.dataStart = 0;
      fsStoreEntry(nodeIndex, nodeEntry);
      Serial.print(F("Wrote 0 bytes to "));
      Serial.println(path);
      return;
    }

    const size_t size = eepromSize();
    const uint16_t nextFree = fsNextFree();
    if (nextFree > size || textLen > (size - nextFree)) {
      Serial.println(F("Not enough EEPROM data space. Run 'fs format confirm'."));
      return;
    }

    for (size_t i = 0; i < textLen; ++i) {
      EEPROM.update(static_cast<int>(nextFree + i), static_cast<uint8_t>(text[i]));
    }

    nodeEntry.dataStart = nextFree;
    nodeEntry.dataLen = static_cast<uint16_t>(textLen);
    fsStoreEntry(nodeIndex, nodeEntry);
    fsSetNextFree(static_cast<uint16_t>(nextFree + textLen));

    Serial.print(F("Wrote "));
    Serial.print(textLen);
    Serial.print(F(" byte(s) to "));
    Serial.println(path);
    return;
  }

  if (equalsIgnoreCase(argv[1], "rm")) {
    if (argc != 3) {
      Serial.println(F("Usage: fs rm <path>"));
      return;
    }

    uint8_t nodeIndex = kFsRootParent;
    FsEntry nodeEntry;
    if (!fsResolvePath(argv[2], nodeIndex, nodeEntry) || nodeIndex == kFsRootParent) {
      Serial.println(F("Path not found."));
      return;
    }
    if (nodeEntry.isDir && fsHasChildren(nodeIndex)) {
      Serial.println(F("Directory not empty."));
      return;
    }

    fsClearEntry(nodeIndex);
    Serial.print(F("Removed: "));
    Serial.println(argv[2]);
    return;
  }

  if (equalsIgnoreCase(argv[1], "stat")) {
    if (argc != 2) {
      Serial.println(F("Usage: fs stat"));
      return;
    }

    uint8_t used = 0;
    uint8_t dirs = 0;
    uint8_t files = 0;
    for (uint8_t i = 0; i < kFsMaxEntries; ++i) {
      FsEntry entry;
      fsLoadEntry(i, entry);
      if (!entry.used) {
        continue;
      }
      ++used;
      if (entry.isDir) {
        ++dirs;
      } else {
        ++files;
      }
    }

    const size_t total = eepromSize();
    const uint16_t nextFree = fsNextFree();
    const size_t dataCapacity = total - kFsDataStart;
    const size_t dataUsed = nextFree - kFsDataStart;
    const size_t dataFree = total - nextFree;

    Serial.println(F("\n=== FS Stat ==="));
    Serial.print(F("Entries: "));
    Serial.print(used);
    Serial.print(F("/"));
    Serial.println(kFsMaxEntries);
    Serial.print(F("Dirs: "));
    Serial.print(dirs);
    Serial.print(F(", Files: "));
    Serial.println(files);
    Serial.print(F("Data start: 0x"));
    printHexWord(kFsDataStart);
    Serial.print(F(", next free: 0x"));
    printHexWord(nextFree);
    Serial.println();
    Serial.print(F("Data used/free: "));
    Serial.print(dataUsed);
    Serial.print(F("/"));
    Serial.print(dataCapacity);
    Serial.print(F(" bytes (free "));
    Serial.print(dataFree);
    Serial.println(F(")"));
    Serial.println(F("==============\n"));
    return;
  }

  Serial.println(F("Unknown fs command. Use 'fs help'."));
}
#endif

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

#if FEATURE_FS
  if (startsWithIgnoreCase(trimmed, "fs")) {
    const char next = trimmed[2];
    if (next == '\0' || isspace(static_cast<unsigned char>(next))) {
      handleFsCommand(trimmed);
      return;
    }
  }
#endif

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
  if (strcmp(cmd, "micros") == 0) {
    Serial.print(F("micros(): "));
    Serial.println(micros());
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
  char *argv[kMaxArgs] = {};
  const size_t argc = splitArgs(cmdArgs, argv, kMaxArgs);

#if FEATURE_I2C
  if (argc > 0 && strcmp(argv[0], "i2cspeed") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: i2cspeed <100k|400k>"));
      return;
    }

    uint32_t hz = 0;
    if (!parseI2cSpeedToken(argv[1], hz)) {
      Serial.println(F("Invalid speed. Use 100k or 400k."));
      return;
    }

    setI2cClock(hz);
    Serial.print(F("I2C speed set to "));
    Serial.print(gI2cClockHz / 1000UL);
    Serial.println(F(" kHz"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "i2cscan") == 0) {
    if (argc != 1) {
      Serial.println(F("Usage: i2cscan"));
      return;
    }

    uint8_t found = 0;
    Serial.println(F("Scanning I2C addresses 0x01..0x7F..."));
    for (uint8_t address = 1; address <= 0x7F; ++address) {
      Wire.beginTransmission(address);
      const uint8_t status = Wire.endTransmission();
      if (status == 0) {
        Serial.print(F("  found @ "));
        printI2cAddress(address);
        Serial.println();
        ++found;
      } else if (status == 4) {
        Serial.print(F("  bus error @ "));
        printI2cAddress(address);
        Serial.println();
      }
    }

    if (found == 0) {
      Serial.println(F("No I2C devices found."));
    } else {
      Serial.print(F("I2C devices found: "));
      Serial.println(found);
    }
    return;
  }

  if (argc > 0 && strcmp(argv[0], "i2cread") == 0) {
    if (argc != 3) {
      Serial.println(F("Usage: i2cread <addr> <n>"));
      Serial.print(F("n range: 1.."));
      Serial.println(kI2cMaxTransferLen);
      return;
    }

    uint8_t address = 0;
    uint8_t length = 0;
    if (!parseI2cAddress(argv[1], address)) {
      Serial.println(F("Invalid address. Use 0x00..0x7F."));
      return;
    }
    if (!parseI2cLen(argv[2], length)) {
      Serial.print(F("Invalid length. Use 1.."));
      Serial.println(kI2cMaxTransferLen);
      return;
    }

    const uint8_t received =
        Wire.requestFrom(static_cast<int>(address), static_cast<int>(length));
    Serial.print(F("i2cread "));
    printI2cAddress(address);
    Serial.print(F(" -> "));
    Serial.print(received);
    Serial.print(F(" byte(s):"));

    for (uint8_t i = 0; i < received && Wire.available() > 0; ++i) {
      const uint8_t value = static_cast<uint8_t>(Wire.read());
      Serial.write(' ');
      Serial.print(F("0x"));
      printHexByte(value);
    }
    Serial.println();

    if (received != length) {
      Serial.print(F("Short read (requested "));
      Serial.print(length);
      Serial.println(F(")."));
    }
    return;
  }

  if (argc > 0 && strcmp(argv[0], "i2cwrite") == 0) {
    if (argc < 3) {
      Serial.println(F("Usage: i2cwrite <addr> <bytes...>"));
      return;
    }

    uint8_t address = 0;
    if (!parseI2cAddress(argv[1], address)) {
      Serial.println(F("Invalid address. Use 0x00..0x7F."));
      return;
    }

    const size_t dataLen = argc - 2;
    if (dataLen == 0 || dataLen > kI2cMaxTransferLen) {
      Serial.print(F("Data length must be 1.."));
      Serial.println(kI2cMaxTransferLen);
      return;
    }

    uint8_t data[kI2cMaxTransferLen] = {};
    for (size_t i = 0; i < dataLen; ++i) {
      if (!parseByteValue(argv[2 + i], data[i])) {
        Serial.print(F("Invalid data byte: "));
        Serial.println(argv[2 + i]);
        return;
      }
    }

    Wire.beginTransmission(address);
    for (size_t i = 0; i < dataLen; ++i) {
      Wire.write(data[i]);
    }
    const uint8_t status = Wire.endTransmission();
    if (status != 0) {
      printI2cTxStatus(status);
      return;
    }

    Serial.print(F("Wrote "));
    Serial.print(dataLen);
    Serial.print(F(" byte(s) to "));
    printI2cAddress(address);
    Serial.println();
    return;
  }

  if (argc > 0 && strcmp(argv[0], "i2cwr") == 0) {
    if (argc < 4) {
      Serial.println(F("Usage: i2cwr <addr> <reg> <bytes...>"));
      return;
    }

    uint8_t address = 0;
    uint8_t reg = 0;
    if (!parseI2cAddress(argv[1], address)) {
      Serial.println(F("Invalid address. Use 0x00..0x7F."));
      return;
    }
    if (!parseByteValue(argv[2], reg)) {
      Serial.println(F("Invalid register. Use 0..255 or 0x00..0xFF."));
      return;
    }

    const size_t dataLen = argc - 3;
    if (dataLen == 0 || (1 + dataLen) > kI2cMaxTransferLen) {
      Serial.print(F("Payload too long. reg + data must be <= "));
      Serial.print(kI2cMaxTransferLen);
      Serial.println(F(" bytes."));
      return;
    }

    uint8_t data[kI2cMaxTransferLen - 1] = {};
    for (size_t i = 0; i < dataLen; ++i) {
      if (!parseByteValue(argv[3 + i], data[i])) {
        Serial.print(F("Invalid data byte: "));
        Serial.println(argv[3 + i]);
        return;
      }
    }

    Wire.beginTransmission(address);
    Wire.write(reg);
    for (size_t i = 0; i < dataLen; ++i) {
      Wire.write(data[i]);
    }
    const uint8_t status = Wire.endTransmission();
    if (status != 0) {
      printI2cTxStatus(status);
      return;
    }

    Serial.print(F("Wrote reg 0x"));
    printHexByte(reg);
    Serial.print(F(" + "));
    Serial.print(dataLen);
    Serial.print(F(" byte(s) to "));
    printI2cAddress(address);
    Serial.println();
    return;
  }

  if (argc > 0 && strcmp(argv[0], "i2crr") == 0) {
    if (argc != 4) {
      Serial.println(F("Usage: i2crr <addr> <reg> <n>"));
      Serial.print(F("n range: 1.."));
      Serial.println(kI2cMaxTransferLen);
      return;
    }

    uint8_t address = 0;
    uint8_t reg = 0;
    uint8_t length = 0;
    if (!parseI2cAddress(argv[1], address)) {
      Serial.println(F("Invalid address. Use 0x00..0x7F."));
      return;
    }
    if (!parseByteValue(argv[2], reg)) {
      Serial.println(F("Invalid register. Use 0..255 or 0x00..0xFF."));
      return;
    }
    if (!parseI2cLen(argv[3], length)) {
      Serial.print(F("Invalid length. Use 1.."));
      Serial.println(kI2cMaxTransferLen);
      return;
    }

    Wire.beginTransmission(address);
    Wire.write(reg);
    const uint8_t txStatus = Wire.endTransmission(false);
    if (txStatus != 0) {
      printI2cTxStatus(txStatus);
      return;
    }

    const uint8_t received =
        Wire.requestFrom(static_cast<int>(address), static_cast<int>(length));
    Serial.print(F("i2crr "));
    printI2cAddress(address);
    Serial.print(F(" reg 0x"));
    printHexByte(reg);
    Serial.print(F(" -> "));
    Serial.print(received);
    Serial.print(F(" byte(s):"));

    for (uint8_t i = 0; i < received && Wire.available() > 0; ++i) {
      const uint8_t value = static_cast<uint8_t>(Wire.read());
      Serial.write(' ');
      Serial.print(F("0x"));
      printHexByte(value);
    }
    Serial.println();

    if (received != length) {
      Serial.print(F("Short read (requested "));
      Serial.print(length);
      Serial.println(F(")."));
    }
    return;
  }
#endif

#if FEATURE_EEPROM
  if (argc > 0 && strcmp(argv[0], "eepread") == 0) {
    if (argc != 2 && argc != 3) {
      Serial.println(F("Usage: eepread <addr> [len]"));
      return;
    }

    const size_t size = eepromSize();
    uint16_t address = 0;
    if (!parseEepromAddress(argv[1], address)) {
      Serial.print(F("Invalid EEPROM address. Use 0.."));
      Serial.println(size - 1);
      return;
    }

    size_t length = 1;
    if (argc == 3 && !parseEepromLen(argv[2], length)) {
      Serial.println(F("Invalid length. Use >= 1."));
      return;
    }

    const size_t start = static_cast<size_t>(address);
    if (length > (size - start)) {
      Serial.println(F("Read range exceeds EEPROM."));
      return;
    }

    Serial.print(F("EEPROM read "));
    Serial.print(length);
    Serial.print(F(" byte(s) @ 0x"));
    printHexWord(address);
    Serial.println();

    for (size_t i = 0; i < length; ++i) {
      const size_t index = start + i;
      if ((i % 16) == 0) {
        Serial.print(F("0x"));
        printHexWord(static_cast<uint16_t>(index));
        Serial.print(F(":"));
      }

      Serial.write(' ');
      printHexByte(EEPROM.read(static_cast<int>(index)));

      if ((i % 16) == 15 || (i + 1) == length) {
        Serial.println();
      }
    }
    return;
  }

  if (argc > 0 && strcmp(argv[0], "eepwrite") == 0) {
    if (argc < 3) {
      Serial.println(F("Usage: eepwrite <addr> <bytes...>"));
      return;
    }

    const size_t size = eepromSize();
    uint16_t address = 0;
    if (!parseEepromAddress(argv[1], address)) {
      Serial.print(F("Invalid EEPROM address. Use 0.."));
      Serial.println(size - 1);
      return;
    }

    const size_t start = static_cast<size_t>(address);
    const size_t dataLen = argc - 2;
    if (dataLen > (size - start)) {
      Serial.println(F("Write range exceeds EEPROM."));
      return;
    }

    uint8_t data[kMaxArgs] = {};
    for (size_t i = 0; i < dataLen; ++i) {
      if (!parseByteValue(argv[2 + i], data[i])) {
        Serial.print(F("Invalid byte: "));
        Serial.println(argv[2 + i]);
        return;
      }
    }

    for (size_t i = 0; i < dataLen; ++i) {
      EEPROM.update(static_cast<int>(start + i), data[i]);
    }

    Serial.print(F("EEPROM wrote "));
    Serial.print(dataLen);
    Serial.print(F(" byte(s) @ 0x"));
    printHexWord(address);
    Serial.println();
    return;
  }

  if (argc > 0 && strcmp(argv[0], "eeperase") == 0) {
    if (argc != 2 || strcmp(argv[1], kEepromEraseToken) != 0) {
      Serial.print(F("Usage: eeperase "));
      Serial.println(kEepromEraseToken);
      return;
    }

    const size_t size = eepromSize();
    for (size_t i = 0; i < size; ++i) {
      EEPROM.update(static_cast<int>(i), kEepromEraseValue);
    }

    Serial.print(F("EEPROM cleared to 0x"));
    printHexByte(kEepromEraseValue);
    Serial.print(F(" ("));
    Serial.print(size);
    Serial.println(F(" bytes)."));
    return;
  }
#endif

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

  if (argc > 0 && strcmp(argv[0], "delay") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: delay <ms>"));
      return;
    }
    unsigned long delayMs = 0;
    if (!parseUnsignedAuto(argv[1], delayMs) || delayMs > 600000UL) {
      Serial.println(F("Invalid delay. Use 0..600000 ms."));
      return;
    }
    Serial.print(F("Delaying "));
    Serial.print(delayMs);
    Serial.println(F(" ms..."));
    delay(delayMs);
    Serial.println(F("Done."));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "freq") == 0) {
    if (argc != 2 && argc != 3) {
      Serial.println(F("Usage: freq <pin> [ms]"));
      Serial.print(F("Window: "));
      Serial.print(kMinFreqWindowMs);
      Serial.print(F(".."));
      Serial.print(kMaxFreqWindowMs);
      Serial.println(F(" ms"));
      return;
    }

    int pin = -1;
    if (!parsePinToken(argv[1], pin)) {
      Serial.println(F("Invalid pin. Use D0-D22 or A0-A5."));
      return;
    }

    unsigned long windowMs = kDefaultFreqWindowMs;
    if (argc == 3) {
      if (!parseUnsignedAuto(argv[2], windowMs) || windowMs < kMinFreqWindowMs ||
          windowMs > kMaxFreqWindowMs) {
        Serial.print(F("Invalid window. Use "));
        Serial.print(kMinFreqWindowMs);
        Serial.print(F(".."));
        Serial.print(kMaxFreqWindowMs);
        Serial.println(F(" ms."));
        return;
      }
    }

    const uint32_t startUs = micros();
    const uint32_t windowUs = static_cast<uint32_t>(windowMs) * 1000UL;
    uint32_t risingEdges = 0;
    int prev = digitalRead(pin);

    while ((uint32_t)(micros() - startUs) < windowUs) {
      const int curr = digitalRead(pin);
      if (prev == LOW && curr == HIGH) {
        ++risingEdges;
      }
      prev = curr;
    }

    const uint32_t elapsedUs = micros() - startUs;
    uint32_t hzWhole = 0;
    uint8_t hzFrac2 = 0;
    if (elapsedUs > 0) {
      const uint64_t hzX100 =
          (static_cast<uint64_t>(risingEdges) * 100000000ULL) / elapsedUs;
      hzWhole = static_cast<uint32_t>(hzX100 / 100ULL);
      hzFrac2 = static_cast<uint8_t>(hzX100 % 100ULL);
    }

    Serial.print(F("freq "));
    printPinLabel(pin);
    Serial.print(F(" ~= "));
    Serial.print(hzWhole);
    Serial.write('.');
    if (hzFrac2 < 10) {
      Serial.write('0');
    }
    Serial.print(hzFrac2);
    Serial.print(F(" Hz (edges="));
    Serial.print(risingEdges);
    Serial.print(F(", window="));
    Serial.print(elapsedUs);
    Serial.println(F(" us)"));
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

#if FEATURE_TONE
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
#endif

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

#if FEATURE_LOWLEVEL
  if (argc > 0 && strcmp(argv[0], "ddr") == 0) {
    if (argc != 2 && argc != 3) {
      Serial.println(F("Usage: ddr <port> [value]"));
      Serial.println(F("Ports: b|c|d"));
      return;
    }
    PortId portId = PortId::B;
    if (!parsePortId(argv[1], portId)) {
      Serial.println(F("Invalid port. Use b|c|d."));
      return;
    }
    volatile uint8_t &reg = ddrForPort(portId);
    if (argc == 3) {
      uint8_t value = 0;
      if (!parseByteValue(argv[2], value)) {
        Serial.println(F("Invalid value. Use 0..255 (decimal or 0x..)."));
        return;
      }
      reg = value;
    }

    Serial.print(F("DDR"));
    Serial.print(portLetter(portId));
    Serial.print(F(" = 0x"));
    printHexByte(reg);
    Serial.print(F(" ("));
    Serial.print(reg);
    Serial.println(F(")"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "port") == 0) {
    if (argc != 2 && argc != 3) {
      Serial.println(F("Usage: port <port> [value]"));
      Serial.println(F("Ports: b|c|d"));
      return;
    }
    PortId portId = PortId::B;
    if (!parsePortId(argv[1], portId)) {
      Serial.println(F("Invalid port. Use b|c|d."));
      return;
    }
    volatile uint8_t &reg = portForPort(portId);
    if (argc == 3) {
      uint8_t value = 0;
      if (!parseByteValue(argv[2], value)) {
        Serial.println(F("Invalid value. Use 0..255 (decimal or 0x..)."));
        return;
      }
      reg = value;
    }

    Serial.print(F("PORT"));
    Serial.print(portLetter(portId));
    Serial.print(F(" = 0x"));
    printHexByte(reg);
    Serial.print(F(" ("));
    Serial.print(reg);
    Serial.println(F(")"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "pin") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: pin <port>"));
      Serial.println(F("Ports: b|c|d"));
      return;
    }
    PortId portId = PortId::B;
    if (!parsePortId(argv[1], portId)) {
      Serial.println(F("Invalid port. Use b|c|d."));
      return;
    }
    volatile uint8_t &reg = pinForPort(portId);
    Serial.print(F("PIN"));
    Serial.print(portLetter(portId));
    Serial.print(F(" = 0x"));
    printHexByte(reg);
    Serial.print(F(" ("));
    Serial.print(reg);
    Serial.println(F(")"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "peek") == 0) {
    if (argc != 2) {
      Serial.println(F("Usage: peek <addr>"));
      Serial.println(F("Address: 0..65535 or 0x0000..0xFFFF"));
      return;
    }

    uint16_t addr = 0;
    if (!parseAddressValue(argv[1], addr)) {
      Serial.println(F("Invalid address. Use 0..65535 or 0x...."));
      return;
    }

    volatile uint8_t *const ptr = reinterpret_cast<volatile uint8_t *>(addr);
    const uint8_t value = *ptr;

    Serial.print(F("[0x"));
    printHexWord(addr);
    Serial.print(F("] = 0x"));
    printHexByte(value);
    Serial.print(F(" ("));
    Serial.print(value);
    Serial.println(F(")"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "poke") == 0) {
    if (argc != 3) {
      Serial.println(F("Usage: poke <addr> <val>"));
      Serial.println(F("Addr: 0..65535 or 0x0000..0xFFFF"));
      Serial.println(F("Val: 0..255 or 0x00..0xFF"));
      return;
    }

    uint16_t addr = 0;
    if (!parseAddressValue(argv[1], addr)) {
      Serial.println(F("Invalid address. Use 0..65535 or 0x...."));
      return;
    }

    uint8_t value = 0;
    if (!parseByteValue(argv[2], value)) {
      Serial.println(F("Invalid value. Use 0..255 or 0x.."));
      return;
    }

    volatile uint8_t *const ptr = reinterpret_cast<volatile uint8_t *>(addr);
    *ptr = value;
    const uint8_t readBack = *ptr;

    Serial.print(F("[0x"));
    printHexWord(addr);
    Serial.print(F("] <= 0x"));
    printHexByte(value);
    Serial.print(F(" (readback 0x"));
    printHexByte(readBack);
    Serial.println(F(")"));
    return;
  }

  if (argc > 0 && strcmp(argv[0], "reg") == 0) {
    if (argc != 1) {
      Serial.println(F("Usage: reg"));
      return;
    }

    Serial.println(F("\n=== AVR Registers ==="));
    Serial.print(F("SP   : 0x"));
    printHexWord(SP);
    Serial.println();

    Serial.print(F("SPL  : 0x"));
    printHexByte(SPL);
    Serial.print(F("  SPH: 0x"));
    printHexByte(SPH);
    Serial.println();

    Serial.print(F("SREG : 0x"));
    printHexByte(SREG);
    Serial.print(F("  MCUSR(now): 0x"));
    printHexByte(MCUSR);
    Serial.print(F("  MCUSR(boot): 0x"));
    printHexByte(gResetFlags);
    Serial.println();

    Serial.print(F("DDRB : 0x"));
    printHexByte(DDRB);
    Serial.print(F("  PORTB: 0x"));
    printHexByte(PORTB);
    Serial.print(F("  PINB: 0x"));
    printHexByte(PINB);
    Serial.println();

    Serial.print(F("DDRC : 0x"));
    printHexByte(DDRC);
    Serial.print(F("  PORTC: 0x"));
    printHexByte(PORTC);
    Serial.print(F("  PINC: 0x"));
    printHexByte(PINC);
    Serial.println();

    Serial.print(F("DDRD : 0x"));
    printHexByte(DDRD);
    Serial.print(F("  PORTD: 0x"));
    printHexByte(PORTD);
    Serial.print(F("  PIND: 0x"));
    printHexByte(PIND);
    Serial.println();

    Serial.println(F("=====================\n"));
    return;
  }
#endif

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
#if FEATURE_FS
  fsEnsureInitialized();
#endif

  Serial.begin(kBaudRate);
#if FEATURE_I2C
  Wire.begin();
  setI2cClock(gI2cClockHz);
#endif
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
