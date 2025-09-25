#include "LSM6DSL.h"

// Some macros (verify these for your chip!):
#define MASTER_ON_BIT       0x04  // For LSM6DSL, bit 2 in MASTER_CONFIG(0x1A) might be MASTER_ON
#define PASS_THRU_BIT       0x10  // Possibly bit 4 for pass-through
///////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////
LSM6DSL::LSM6DSL(TwoWire &wirePort, uint8_t i2cAddress)
  : _wire(&wirePort), _i2cAddress(i2cAddress)
{
}

///////////////////////////////////////////////////////////
// begin()
// Initialize I2C and configure the sensor with default settings.
///////////////////////////////////////////////////////////
bool LSM6DSL::begin()
{
  // Optionally, start the I2C if you haven’t done so globally:
  // _wire->begin(); 
  
  uint8_t id = whoAmI();
  // According to many datasheets, LSM6DS3/LSM6DSL WHO_AM_I can be 0x6A or 0x69,
  // but check your device. Some modules might respond with 0x6B, etc.
  // For LSM6DSL, the typical value is 0x6A.
  if (id != 0x6A && id != 0x69 && id != 0x6B)
  {
    return false; // Could not find the sensor
  }

  // CTRL3_C (0x12) - Example: enable register auto-increment + BDU
  //   Bit 2 = IF_INC = 1 => auto address increment
  //   Bit 6 = BDU = 1 => block data update
  //   The rest bits can be default or as needed
  //   0x44 = 0100 0100b => BDU=1, IF_INC=1
  writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x44);

  // Now set some default ODR and full scale for accel
  setAccelODR(LSM6DSL_XL_ODR_104Hz);
  setAccelFS(LSM6DSL_XL_FS_2g);

  // Set some default ODR and full scale for gyro
  setGyroODR(LSM6DSL_G_ODR_104Hz);
  setGyroFS(LSM6DSL_G_FS_245dps);

  return true;
}

///////////////////////////////////////////////////////////
// whoAmI()
// Reads the WHO_AM_I register (0x0F).
///////////////////////////////////////////////////////////
uint8_t LSM6DSL::whoAmI()
{
  return readRegister(LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
}

///////////////////////////////////////////////////////////
// setAccelODR()
// Configure the accelerometer output data rate.
///////////////////////////////////////////////////////////
bool LSM6DSL::setAccelODR(lsm6dsl_xl_odr_t odr)
{
  // Read current CTRL1_XL (0x10)
  uint8_t ctrl1_xl = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // CTRL1_XL: ODR_XL is bits [7:4], FS_XL is bits [3:2]
  // We only modify bits [7:4] for the ODR
  ctrl1_xl &= ~0xF0; // Clear bits 7:4
  ctrl1_xl |= (odr << 4);

  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl1_xl);
  _accelODR = odr;
  _accelEnabled = (odr != LSM6DSL_XL_ODR_POWER_DOWN);
  return true;
}

///////////////////////////////////////////////////////////
// setAccelFS()
// Configure the accelerometer full-scale range.
///////////////////////////////////////////////////////////
bool LSM6DSL::setAccelFS(lsm6dsl_xl_fs_t fs)
{
  // Read current CTRL1_XL
  uint8_t ctrl1_xl = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // Bits [3:2] are FS_XL
  ctrl1_xl &= ~0x0C; // Clear bits 3:2
  ctrl1_xl |= (static_cast<uint8_t>(fs) << 2);

  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl1_xl);

  _accelFS = fs;
  
  return true;
}

bool LSM6DSL::setAccelConfig(lsm6dsl_xl_odr_t odr, lsm6dsl_xl_fs_t fs)
{
  uint8_t value = 0;
  value |= (odr << 4);        // ODR bits [7:4]
  value |= (fs << 2);         // FS_XL bits [3:2]
  // Other bits (LPF, BW) left default for now

  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, value);

  _accelFS = fs;
  _accelODR = odr;
  _accelEnabled = (odr != LSM6DSL_XL_ODR_POWER_DOWN);
  return true;
}


///////////////////////////////////////////////////////////
// setGyroODR()
// Configure the gyroscope output data rate.
///////////////////////////////////////////////////////////
bool LSM6DSL::setGyroODR(lsm6dsl_g_odr_t odr)
{
  // Read CTRL2_G (0x11)
  uint8_t ctrl2_g = readRegister(LSM6DS3_ACC_GYRO_CTRL2_G);

  // CTRL2_G: ODR_G is bits [7:4], FS_G is bits [3:0] but not all bits are used the same
  ctrl2_g &= ~0xF0; // Clear bits 7:4
  ctrl2_g |= (odr << 4);

  writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, ctrl2_g);
  _gyroODR = odr;
  _gyroEnabled = (odr != LSM6DSL_G_ODR_POWER_DOWN);
  return true;
}

///////////////////////////////////////////////////////////
// setGyroFS()
// Configure the gyroscope full-scale range.
///////////////////////////////////////////////////////////
bool LSM6DSL::setGyroFS(lsm6dsl_g_fs_t fs)
{
  // Read CTRL2_G
  uint8_t ctrl2_g = readRegister(LSM6DS3_ACC_GYRO_CTRL2_G);

  // For LSM6DSL:
  //   FS_G is bits [3:2] in many configurations, but 125 dps uses bit [1]
  //   Check the datasheet for exact mapping. For example:
  //     245 dps => FS_G=00
  //     500 dps => FS_G=10
  //     1000 dps => FS_G=100
  //     2000 dps => FS_G=110
  //   The code below is simplified.
  // 
  // Clear bits that define FS_G. This might be bits 1,2,3 or 1,2
  // depending on if you allow 125dps. Check your device revision carefully.
  ctrl2_g &= ~0x0E;

  // For this example, just shift the value into the correct place:
  // If fs = 125 dps, it might be special handling. 
  // If fs = 245, 500, 1000, 2000, it's typically bits [3:2].
  // The enumerations might need to be masked into the correct location.
  // This is a simplistic approach; adapt to your actual datasheet if needed.
  ctrl2_g |= (static_cast<uint8_t>(fs) & 0x0E);

  writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, ctrl2_g);
  _gyroFS = fs;
  
  return true;
}

///////////////////////////////////////////////////////////
// readAccelerometer()
// Read raw 16-bit X, Y, Z data from the accelerometer.
///////////////////////////////////////////////////////////
void LSM6DSL::readAccelerometer(int16_t &x, int16_t &y, int16_t &z)
{
  // The LSM6DSL/LSM6DS3 outputs accelerometer data in
  // OUTX_L_XL (0x28), OUTX_H_XL (0x29),
  // OUTY_L_XL (0x2A), OUTY_H_XL (0x2B),
  // OUTZ_L_XL (0x2C), OUTZ_H_XL (0x2D)
  uint8_t buffer[6];
  readRegisters(LSM6DS3_ACC_GYRO_OUTX_L_XL, buffer, 6);

  // Combine bytes
  x = (int16_t)((buffer[1] << 8) | buffer[0]);
  y = (int16_t)((buffer[3] << 8) | buffer[2]);
  z = (int16_t)((buffer[5] << 8) | buffer[4]);
}

///////////////////////////////////////////////////////////
// readGyroscope()
// Read raw 16-bit X, Y, Z data from the gyroscope.
///////////////////////////////////////////////////////////
void LSM6DSL::readGyroscope(int16_t &x, int16_t &y, int16_t &z)
{
  // The LSM6DSL/LSM6DS3 outputs gyro data in
  // OUTX_L_G (0x22), OUTX_H_G (0x23),
  // OUTY_L_G (0x24), OUTY_H_G (0x25),
  // OUTZ_L_G (0x26), OUTZ_H_G (0x27)
  uint8_t buffer[6];
  readRegisters(LSM6DS3_ACC_GYRO_OUTX_L_G, buffer, 6);

  x = (int16_t)((buffer[1] << 8) | buffer[0]);
  y = (int16_t)((buffer[3] << 8) | buffer[2]);
  z = (int16_t)((buffer[5] << 8) | buffer[4]);
}

///////////////////////////////////////////////////////////
// readTemperature()
// Read and return the temperature in raw 16-bit form.
// LSM6DSL data: OUT_TEMP_L (0x20), OUT_TEMP_H (0x21)
///////////////////////////////////////////////////////////
int16_t LSM6DSL::readTemperature()
{
  uint8_t tempLow  = readRegister(LSM6DS3_ACC_GYRO_OUT_TEMP_L);
  uint8_t tempHigh = readRegister(LSM6DS3_ACC_GYRO_OUT_TEMP_H);

  int16_t rawTemp = (int16_t)((tempHigh << 8) | tempLow);
  return rawTemp;

  // If you want Celsius, consult the datasheet for the conversion formula.
  // For LSM6DSL, typically the formula is:
  //   Temp(C) = 25 + (rawTemp / 16 LSB/°C) 
  // or something close; check the specific device revision.
}

///////////////////////////////////////////////////////////
// writeRegister()
// Write a single byte to a register via I2C.
///////////////////////////////////////////////////////////
void LSM6DSL::writeRegister(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_i2cAddress);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

///////////////////////////////////////////////////////////
// readRegister()
// Read a single byte from a register via I2C.
///////////////////////////////////////////////////////////
uint8_t LSM6DSL::readRegister(uint8_t reg)
{
  _wire->beginTransmission(_i2cAddress);
  _wire->write(reg);
  // Send repeated start (no STOP) so we can read
  _wire->endTransmission(false);

  _wire->requestFrom((int)_i2cAddress, 1);
  if (_wire->available()) {
    return _wire->read();
  }
  return 0; // Error case
}

///////////////////////////////////////////////////////////
// readRegisters()
// Read multiple consecutive bytes from a starting register.
///////////////////////////////////////////////////////////
void LSM6DSL::readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length)
{
  _wire->beginTransmission(_i2cAddress);
  _wire->write(startReg);
  _wire->endTransmission(false);

  _wire->requestFrom((int)_i2cAddress, (int)length);
  for (uint8_t i = 0; i < length; i++)
  {
    if (_wire->available()) {
      buffer[i] = _wire->read();
    } else {
      buffer[i] = 0; // or handle error
    }
  }
}

bool LSM6DSL::initMagnetometer()
{
  // Enable access to Sensor Hub
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, MASTER_ON_BIT);

  // LIS3MDL: set to continuous mode (X/Y/Z enabled, 10 Hz+ ODR)
  // CTRL_REG3 = 0x00 => continuous conversion
  writeRegister(0x02, (LIS3MDL_I2C_ADDR << 1)); // SLV0_ADDR: LIS3MDL write
  writeRegister(0x03, LIS3MDL_CTRL_REG3);       // SLV0_SUBADDR
  writeRegister(0x04, 0x00);                    // SLV0_CONFIG: 1 byte write

  delay(10); // Let LIS3MDL settle

  return true;
}

bool LSM6DSL::readMagnetometer(int16_t &x, int16_t &y, int16_t &z)
{
  // Configure sensor hub to read 6 bytes from LIS3MDL starting at OUT_X_L
  writeRegister(0x02, (LIS3MDL_I2C_ADDR << 1) | 0x01); // SLV0_ADDR: LIS3MDL read
  writeRegister(0x03, LIS3MDL_OUT_X_L);               // SLV0_SUBADDR
  writeRegister(0x04, 0x06);                          // SLV0_CONFIG: 6 bytes

  // Enable Sensor Hub
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, MASTER_ON_BIT);

  // ⚠️ Cache original CTRL1_XL to preserve FS and LPF bits
  uint8_t ctrl1_orig = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // Set ODR to 104 Hz (required to clock sensor hub), keep FS & LPF unchanged
  uint8_t ctrl1_temp = ctrl1_orig;
  ctrl1_temp &= ~0xF0; // Clear ODR bits
  ctrl1_temp |= (LSM6DSL_XL_ODR_104Hz << 4); // Set ODR = 104Hz
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl1_temp);

  delay(20); // Let data populate

  uint8_t buffer[6];
  readRegisters(LSM6DS3_ACC_GYRO_SENSORHUB1_REG, buffer, 6);

  // Stop sensor hub to prevent continuous I2C polling
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, 0x00);

  // Restore original accel config (ODR, FS, LPF)
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl1_orig);

  // Parse raw data
  x = (int16_t)((buffer[1] << 8) | buffer[0]);
  y = (int16_t)((buffer[3] << 8) | buffer[2]);
  z = (int16_t)((buffer[5] << 8) | buffer[4]);

  return true;
}

float accelSensitivity_g(lsm6dsl_xl_fs_t fs) {
  switch (fs) {
    case LSM6DSL_XL_FS_2g:  return 0.061f / 1000.0f; // 61 mg/LSB
    case LSM6DSL_XL_FS_4g:  return 0.122f / 1000.0f; // 122 mg/LSB
    case LSM6DSL_XL_FS_8g:  return 0.244f / 1000.0f; // 244 mg/LSB
    case LSM6DSL_XL_FS_16g: return 0.488f / 1000.0f; // 488 mg/LSB
    default: return 0.061f / 1000.0f; // Default to 2g
  }
}

float gyroSensitivity_dps(lsm6dsl_g_fs_t fs) {
  switch (fs) {
    case LSM6DSL_G_FS_125dps:  return 4.375f / 1000.0f; // 4.375 mdps/LSB
    case LSM6DSL_G_FS_245dps:  return 8.75f / 1000.0f; // 8.75 mdps/LSB 
    case LSM6DSL_G_FS_500dps:  return 17.5f / 1000.0f; // 17.5 mdps/LSB
    case LSM6DSL_G_FS_1000dps: return 35.0f / 1000.0f; // 35 mdps/LSB
    case LSM6DSL_G_FS_2000dps: return 70.0f / 1000.0f; // 70 mdps/LSB
    default: return 8.75f / 1000.0f; // Default to 245dps
  }
}

void LSM6DSL::readAccelerometer_g(float &x, float &y, float &z, lsm6dsl_xl_fs_t fs) {
  int16_t rawX, rawY, rawZ;
  readAccelerometer(rawX, rawY, rawZ);

  float scale = accelSensitivity_g(fs);
  x = rawX * scale;
  y = rawY * scale;
  z = rawZ * scale;
}

void LSM6DSL::readGyroscope_dps(float &x, float &y, float &z, lsm6dsl_g_fs_t fs) {
  int16_t rawX, rawY, rawZ;
  readGyroscope(rawX, rawY, rawZ);

  float scale = gyroSensitivity_dps(fs);
  x = rawX * scale;
  y = rawY * scale;
  z = rawZ * scale;
}

void LSM6DSL::readMagnetometer_uT(float &x, float &y, float &z) {
  int16_t rawX, rawY, rawZ;
  readMagnetometer(rawX, rawY, rawZ);

  float scale = 0.14f; // mG/LSB
  float scale_uT = scale * 0.1f; // convert to µT

  x = rawX * scale_uT;
  y = rawY * scale_uT;
  z = rawZ * scale_uT;
}

bool LSM6DSL::enableAccelerometer(lsm6dsl_xl_odr_t odr)
{
  // Read current CTRL1_XL register
  uint8_t reg = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // Clear ODR bits (bits 7:4) only, preserve FS, LPF, etc.
  reg &= ~0xF0;

  // Set new ODR
  reg |= (odr << 4);

  // Write back
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, reg);

  // Update internal state
  _accelEnabled = (odr != LSM6DSL_XL_ODR_POWER_DOWN);
  _accelODR = odr;

  return true;
}


bool LSM6DSL::disableAccelerometer() {
  uint8_t reg = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);
  reg &= ~0xF0; // Clear only ODR bits (bits 7:4)
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, reg);
  _accelEnabled = false;
  _accelODR = LSM6DSL_XL_ODR_POWER_DOWN;
  return true;
}

bool LSM6DSL::enableGyroscope(lsm6dsl_g_odr_t odr) {
  uint8_t reg = readRegister(LSM6DS3_ACC_GYRO_CTRL2_G);
  reg &= ~0xF0; // Clear ODR bits
  reg |= (odr << 4);
  writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, reg);
  _gyroEnabled = true;
  _gyroODR = odr;
  return true;
  
}

bool LSM6DSL::disableGyroscope() {
  uint8_t reg = readRegister(LSM6DS3_ACC_GYRO_CTRL2_G);
  reg &= ~0xF0; // Clear ODR bits
  writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, reg);
  _gyroEnabled = false;
  _gyroODR = LSM6DSL_G_ODR_POWER_DOWN;

  return true;
}

bool LSM6DSL::disableMagnetometer()
{
  // Sensor Hub write: LIS3MDL CTRL_REG3 = 0x03 (power-down mode)
  writeRegister(0x02, (LIS3MDL_I2C_ADDR << 1));     // SLV0_ADDR: write
  writeRegister(0x03, LIS3MDL_CTRL_REG3);           // SLV0_SUBADDR
  writeRegister(0x04, 0x03);                        // SLV0_CONFIG: 1 byte

  // Cache CTRL1_XL before modifying
  uint8_t ctrl1_orig = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // Trigger write using Sensor Hub
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, MASTER_ON_BIT);
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x40); // Minimal ODR to activate SH
  delay(20);
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, 0x00);

  // Restore original accel config
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl1_orig);

  return true;
}

bool LSM6DSL::enableMagnetometer()
{
  // Sensor Hub write: LIS3MDL CTRL_REG3 = 0x00 (continuous mode)
  writeRegister(0x02, (LIS3MDL_I2C_ADDR << 1));     // SLV0_ADDR: write
  writeRegister(0x03, LIS3MDL_CTRL_REG3);           // SLV0_SUBADDR
  writeRegister(0x04, 0x00);                        // SLV0_CONFIG: 1 byte write (continuous)

  // Cache original CTRL1_XL to restore after sensor hub write
  uint8_t ctrl1_orig = readRegister(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // Enable Sensor Hub and trigger write using minimal accel config
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, MASTER_ON_BIT);
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x40); // ODR = 104Hz, FS = 2g
  delay(20); // Allow write to complete
  writeRegister(LSM6DS3_ACC_GYRO_MASTER_CONFIG, 0x00);

  // Restore previous accel config
  writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, ctrl1_orig);

  return true;
}

float LSM6DSL::convertRawAccelToG(int16_t raw, lsm6dsl_xl_fs_t fs)
{
  float scale = accelSensitivity_g(fs); // mg/LSB to g/LSB
  return raw * scale;
}

float LSM6DSL::convertRawGyroToDPS(int16_t raw, lsm6dsl_g_fs_t fs)
{
  float scale = gyroSensitivity_dps(fs); // mdps/LSB to dps/LSB
  return raw * scale;
}

float LSM6DSL::convertRawMagTouT(int16_t raw)
{
  float scale = 0.146f; // mG/LSB
  float scale_uT = scale * 0.1f; // Convert to µT
  return raw * scale_uT;
}
