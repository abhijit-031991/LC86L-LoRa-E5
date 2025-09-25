/*
  MPR121_LoRaE5.cpp
  Implementation for the header above.
*/


#include "MPR121_LoRaE5.h"

MPR121_LoRaE5::MPR121_LoRaE5()
  : _i2cAddr(DEFAULT_ADDR), _wire(&Wire), _irqPin(-1) {}

bool MPR121_LoRaE5::begin(uint8_t i2c_addr, TwoWire &wire, uint8_t touchThreshold, uint8_t releaseThreshold, bool autoconfig) {
  _i2cAddr = i2c_addr;
  _wire = &wire;
  _wire->begin();

  softReset();
  delay(1);

  // Stop mode before changing registers
  stopMode();

  // Set default filter/baseline registers (sensible defaults similar to Adafruit)
  _defaultFilterAndBaseline();

  // Set thresholds
  setThresholds(touchThreshold, releaseThreshold);

  // Set CDC default
  setCDC(0x10);

  // set CONFIG1 default (not altering) and CONFIG2 left for sample interval

  if (autoconfig) {
    enableAutoConfig(0x10, 0x10);
  } else {
    disableAutoConfig();
  }

  // Enter run mode enabling all 12 electrodes by using Adafruit-style ECR 0x8F
  writeRegister(MPR121_REG::ECR, 0x8F);

  return true;
}

void MPR121_LoRaE5::softReset() {
  writeRegister(MPR121_REG::SOFTRESET, 0x63);
  delay(1);
}

void MPR121_LoRaE5::stopMode() {
  writeRegister(MPR121_REG::ECR, 0x00);
}

void MPR121_LoRaE5::runMode(uint8_t electrodesEnableMask) {
  if (electrodesEnableMask == 0) {
    stopMode();
    return;
  }
  // For simplicity set a run mode pattern; for fine control user can write ECR directly
  // If user wants precise electrode enable mapping they can call writeRegister(ECR, value) manually.
  writeRegister(MPR121_REG::ECR, 0x8F);
}

void MPR121_LoRaE5::setSampleIntervalMs(uint16_t ms, uint8_t sfi) {
  uint8_t esi;
  switch (ms) {
    case 1: esi = 0; break;
    case 2: esi = 1; break;
    case 4: esi = 2; break;
    case 8: esi = 3; break;
    case 16: esi = 4; break;
    case 32: esi = 5; break;
    case 64: esi = 6; break;
    case 128: default: esi = 7; break;
  }
  if (sfi > 3) sfi = 0;
  uint8_t val = (sfi << 3) | (esi & 0x07);
  writeRegister(MPR121_REG::CONFIG2, val);
}

void MPR121_LoRaE5::setCDC(uint8_t cdc) {
  writeRegister(MPR121_REG::CDC, cdc & 0x3F);
}

uint8_t MPR121_LoRaE5::getCDC() {
  return readRegister(MPR121_REG::CDC);
}

void MPR121_LoRaE5::setThresholds(uint8_t touch, uint8_t release) {
  for (uint8_t i = 0; i < 12; i++) {
    setThreshold(i, touch, release);
  }
}

void MPR121_LoRaE5::setThreshold(uint8_t channel, uint8_t touch, uint8_t release) {
  if (channel > 11) return;
  uint8_t tReg = MPR121_REG::TOUCH_THRESH_0 + channel * 2;
  uint8_t rReg = MPR121_REG::RELEASE_THRESH_0 + channel * 2;
  writeRegister(tReg, touch);
  writeRegister(rReg, release);
}

void MPR121_LoRaE5::getThreshold(uint8_t channel, uint8_t &touch, uint8_t &release) {
  if (channel > 11) { touch = 0; release = 0; return; }
  uint8_t tReg = MPR121_REG::TOUCH_THRESH_0 + channel * 2;
  uint8_t rReg = MPR121_REG::RELEASE_THRESH_0 + channel * 2;
  touch = readRegister(tReg);
  release = readRegister(rReg);
}

uint16_t MPR121_LoRaE5::touched() {
  uint8_t lo = readRegister(MPR121_REG::TOUCH_STATUS_L);
  uint8_t hi = readRegister(MPR121_REG::TOUCH_STATUS_H);
  return (((uint16_t)hi << 8) | lo) & 0x0FFF;
}

bool MPR121_LoRaE5::isTouched(uint8_t pin) {
  if (pin > 11) return false;
  uint16_t t = touched();
  return (t & (1 << pin));
}

uint16_t MPR121_LoRaE5::filteredData(uint8_t channel) {
  if (channel > 11) return 0;
  uint8_t addr = MPR121_REG::FILT_DATA_0_L + (channel * 2);
  uint8_t lo = readRegister(addr);
  uint8_t hi = readRegister(addr + 1);
  // 10-bit value, hi uses only bits 0..1
  return (((uint16_t)(hi & 0x03) << 8) | lo);
}

uint8_t MPR121_LoRaE5::baselineData(uint8_t channel) {
  if (channel > 11) return 0;
  return readRegister(MPR121_REG::BASELINE_0 + channel);
}

void MPR121_LoRaE5::configureIRQPin(int irqPin) {
  _irqPin = irqPin;
  if (_irqPin >= 0) pinMode(_irqPin, INPUT_PULLUP);
}

uint16_t MPR121_LoRaE5::handleIRQ_and_getTouchMask() {
  // Reading touch status clears interrupt on device
  return touched();
}

uint8_t MPR121_LoRaE5::readRegister(uint8_t reg) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom((int)_i2cAddr, 1);
  if (_wire->available()) return _wire->read();
  return 0;
}

void MPR121_LoRaE5::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

void MPR121_LoRaE5::readRegisterRegion(uint8_t reg, uint8_t *buffer, uint8_t len) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  _wire->endTransmission(false);
  uint8_t idx = 0;
  _wire->requestFrom((int)_i2cAddr, len);
  while (_wire->available() && idx < len) {
    buffer[idx++] = _wire->read();
  }
}

void MPR121_LoRaE5::writeRegisterRegion(uint8_t reg, const uint8_t *buffer, uint8_t len) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  for (uint8_t i = 0; i < len; i++) {
    _wire->write(buffer[i]);
  }
  _wire->endTransmission();
}

bool MPR121_LoRaE5::enableAutoConfig(uint8_t targetLevel, uint8_t chargeCurrent) {
  // Enable autoconfig: Set USL/LSL/TARG and the control register
  // targetLevel is 7-bit value (datasheet uses series of steps). We'll write values directly.
  // chargeCurrent sets CDC to a reasonable value (0..63)
  setCDC(chargeCurrent);
  // example values: USL = target*1.2, LSL = target*0.65 -> user should compute externally; keep simple
  uint8_t usl = min(0xFF, (uint16_t)targetLevel + (uint16_t)(targetLevel>>2));
  uint8_t lsl = max(0, (int)targetLevel - (int)(targetLevel>>2));
  writeRegister(MPR121_REG::AUTO_CONFIG_USL, usl);
  writeRegister(MPR121_REG::AUTO_CONFIG_LSL, lsl);
  writeRegister(MPR121_REG::AUTO_CONFIG_TARGET, targetLevel);
  // Set auto config control to enable autoconfig (bit definitions in datasheet). Use 0x0B as an enabling example
  writeRegister(MPR121_REG::AUTO_CONFIG_CONTROL, 0x0B);
  return true;
}

void MPR121_LoRaE5::disableAutoConfig() {
  writeRegister(MPR121_REG::AUTO_CONFIG_CONTROL, 0x00);
}

void MPR121_LoRaE5::_defaultFilterAndBaseline() {
  // These values are a conservative set based on examples in application notes and Adafruit library
  // Rising (touch) filter
  writeRegister(MPR121_REG::MHD_R, 0x01);
  writeRegister(MPR121_REG::NHD_R, 0x01);
  writeRegister(MPR121_REG::NCL_R, 0x00);
  writeRegister(MPR121_REG::FDL_R, 0x00);

  // Falling (release) filter
  writeRegister(MPR121_REG::MHD_F, 0x01);
  writeRegister(MPR121_REG::NHD_F, 0x01);
  writeRegister(MPR121_REG::NCL_F, 0xFF);
  writeRegister(MPR121_REG::FDL_F, 0x02);

  // Baseline tracking and others left at datasheet defaults unless user overrides
}
