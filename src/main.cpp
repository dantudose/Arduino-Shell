// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Dan Tudose

#include "shell.hpp"

void setup() {
  shell::captureResetFlags();
#if FEATURE_FS
  shell::fsEnsureInitialized();
  shell::startupScriptInit();
#endif

  Serial.begin(shell::kBaudRate);
#if FEATURE_I2C
  Wire.begin();
  shell::setI2cClock(shell::gI2cClockHz);
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
  Serial.println(F("Type 'help' for full command list."));
  //shell::printHelp();
  shell::printPrompt();
}

void loop() {
  shell::updateBackgroundTasks();
  shell::updateSerial();
}
