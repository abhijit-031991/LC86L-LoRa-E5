#ifndef LSM6DSL_H
#define LSM6DSL_H

#include <Arduino.h>
#include <Wire.h>

/************************************************************
 * LSM6DSL Register Map
 * (Based on the LSM6DSL datasheet – please verify with your
 *  revision of the datasheet before use.)
 *
 * Note: Some registers in the datasheet are marked as “Reserved”
 *       or are used for advanced embedded functions. This file
 *       attempts to include every register that is defined.
 ************************************************************/

// Default I2C address
#define LSM6DSL_DEFAULT_ADDRESS      0x6B

/************** Device Register  *******************/
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME  		0X04
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_EN  		0X05
#define LSM6DS3_ACC_GYRO_FIFO_CTRL1  			0X06
#define LSM6DS3_ACC_GYRO_FIFO_CTRL2  			0X07
#define LSM6DS3_ACC_GYRO_FIFO_CTRL3  			0X08
#define LSM6DS3_ACC_GYRO_FIFO_CTRL4  			0X09
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5  			0X0A
#define LSM6DS3_ACC_GYRO_ORIENT_CFG_G  			0X0B //////////////////////////////
#define LSM6DS3_ACC_GYRO_REFERENCE_G  			0X0C //////////////////////////////RESERVED
#define LSM6DS3_ACC_GYRO_INT1_CTRL  			0X0D
#define LSM6DS3_ACC_GYRO_INT2_CTRL  			0X0E
#define LSM6DS3_ACC_GYRO_WHO_AM_I_REG  			0X0F
#define LSM6DS3_ACC_GYRO_CTRL1_XL  			0X10
#define LSM6DS3_ACC_GYRO_CTRL2_G  			0X11
#define LSM6DS3_ACC_GYRO_CTRL3_C  			0X12
#define LSM6DS3_ACC_GYRO_CTRL4_C  			0X13
#define LSM6DS3_ACC_GYRO_CTRL5_C  			0X14
#define LSM6DS3_ACC_GYRO_CTRL6_C  			0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G  			0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL  			0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL  			0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C  			0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG  		0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC  			0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC  			0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC  			0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG  			0X1E
#define LSM6DS3_ACC_GYRO_OUT_TEMP_L  			0X20
#define LSM6DS3_ACC_GYRO_OUT_TEMP_H  			0X21
#define LSM6DS3_ACC_GYRO_OUTX_L_G  			0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G  			0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G  			0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G  			0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G  			0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G  			0X27
#define LSM6DS3_ACC_GYRO_OUTX_L_XL  			0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL  			0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL  			0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL  			0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL  			0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL  			0X2D
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG  		0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG  		0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG  		0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG  		0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG  		0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG  		0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG  		0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG  		0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG  		0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG  		0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG  		0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG  		0X39
#define LSM6DS3_ACC_GYRO_SENSORHUB13_REG  		0X4D
#define LSM6DS3_ACC_GYRO_SENSORHUB14_REG  		0X4E
#define LSM6DS3_ACC_GYRO_SENSORHUB15_REG  		0X4F
#define LSM6DS3_ACC_GYRO_SENSORHUB16_REG  		0X50
#define LSM6DS3_ACC_GYRO_SENSORHUB17_REG  		0X51
#define LSM6DS3_ACC_GYRO_SENSORHUB18_REG  		0X52
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1  			0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2  			0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3  			0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4  			0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L  		0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H  		0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG  		0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG  		0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG  		0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L  		0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H  		0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC1  			0X53
#define LSM6DS3_ACC_GYRO_FUNC_SRC2  			0X54
#define LSM6DS3_ACC_GYRO_TAP_CFG1  			0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D  			0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2  			0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS  			0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR  			0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL  			0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG  			0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG  			0X5F
#define LSM6DS3_ACC_GYRO_MASTER_CMD_CODE  			0X60
#define LSM6DS3_ACC_GYRO_OUT_MAG_RAW_X_L  			0X66
#define LSM6DS3_ACC_GYRO_OUT_MAG_RAW_X_H  			0X67
#define LSM6DS3_ACC_GYRO_OUT_MAG_RAW_Y_L  			0X68
#define LSM6DS3_ACC_GYRO_OUT_MAG_RAW_Y_H  			0X69
#define LSM6DS3_ACC_GYRO_OUT_MAG_RAW_Z_L  			0X6A
#define LSM6DS3_ACC_GYRO_OUT_MAG_RAW_Z_H  			0X6B

// Typical LIS3MDL I2C address if SA0=GND:
#define LIS3MDL_I2C_ADDR          0x1E 

// LIS3MDL Registers
#define LIS3MDL_WHO_AM_I          0x0F
#define LIS3MDL_CTRL_REG1         0x20
#define LIS3MDL_CTRL_REG2         0x21
#define LIS3MDL_CTRL_REG3         0x22
#define LIS3MDL_CTRL_REG4         0x23
#define LIS3MDL_CTRL_REG5         0x24
#define LIS3MDL_OUT_X_L           0x28
// ... up to Z_H = 0x2D

// LSM6DSL Sensor Hub Registers (slave config, etc.)
// Refer to LSM6DSL datasheet for exact offsets.
#define LSM6DSL_MASTER_CONFIG     0x1A
#define LSM6DSL_SLAVE0_CONFIG     0x04  // or 0x15 in some docs; be sure to check
#define LSM6DSL_SLAVE0_ADDRESS    0x02  // or 0x15... There are multiple versions
// etc. Possibly these are embedded function registers that you must write in a
// specific sequence. Always verify with the official datasheet or an ST example.

/*===========================================================================
  Enumerations for configuration options
===========================================================================*/

// Accelerometer Output Data Rate (ODR)
typedef enum {
  LSM6DSL_XL_ODR_POWER_DOWN = 0x00,
  LSM6DSL_XL_ODR_13Hz       = 0x01,
  LSM6DSL_XL_ODR_26Hz       = 0x02,
  LSM6DSL_XL_ODR_52Hz       = 0x03,
  LSM6DSL_XL_ODR_104Hz      = 0x04,
  LSM6DSL_XL_ODR_208Hz      = 0x05,
  LSM6DSL_XL_ODR_416Hz      = 0x06,
  LSM6DSL_XL_ODR_833Hz      = 0x07,
  LSM6DSL_XL_ODR_1_66kHz    = 0x08,
  LSM6DSL_XL_ODR_3_33kHz    = 0x09,
  LSM6DSL_XL_ODR_6_66kHz    = 0x0A
} lsm6dsl_xl_odr_t;

// Accelerometer full scale
typedef enum {
  LSM6DSL_XL_FS_2g  = 0x00,
  LSM6DSL_XL_FS_16g = 0x01,
  LSM6DSL_XL_FS_4g  = 0x02,
  LSM6DSL_XL_FS_8g  = 0x03
} lsm6dsl_xl_fs_t;

// Gyroscope Output Data Rate (ODR)
typedef enum {
  LSM6DSL_G_ODR_POWER_DOWN = 0x00,
  LSM6DSL_G_ODR_13Hz       = 0x01,
  LSM6DSL_G_ODR_26Hz       = 0x02,
  LSM6DSL_G_ODR_52Hz       = 0x03,
  LSM6DSL_G_ODR_104Hz      = 0x04,
  LSM6DSL_G_ODR_208Hz      = 0x05,
  LSM6DSL_G_ODR_416Hz      = 0x06,
  LSM6DSL_G_ODR_833Hz      = 0x07,
  LSM6DSL_G_ODR_1_66kHz    = 0x08,
  LSM6DSL_G_ODR_3_33kHz    = 0x09,
  LSM6DSL_G_ODR_6_66kHz    = 0x0A
} lsm6dsl_g_odr_t;

// Gyroscope full scale
typedef enum {
  LSM6DSL_G_FS_125dps  = 0x01, // (Check datasheet if this option is available/enabled)
  LSM6DSL_G_FS_245dps  = 0x00,
  LSM6DSL_G_FS_500dps  = 0x02,
  LSM6DSL_G_FS_1000dps = 0x04,
  LSM6DSL_G_FS_2000dps = 0x06
} lsm6dsl_g_fs_t;

// FIFO mode selection values according to the LSM6DSL datasheet (FIFO_CTRL2, bits 7:4):
typedef enum {
  LSM6DSL_FIFO_MODE_BYPASS            = 0x00, // Bypass mode: FIFO disabled.
  LSM6DSL_FIFO_MODE_FIFO              = 0x01, // FIFO mode: Data is stored until FIFO is full.
  LSM6DSL_FIFO_MODE_STREAM            = 0x06, // Stream (continuous) mode: Data continuously stored; oldest data overwritten.
  LSM6DSL_FIFO_MODE_BYPASS_TO_STREAM   = 0x07, // Bypass-to-Stream mode: Start in bypass, switch to streaming upon trigger.
  LSM6DSL_FIFO_MODE_STREAM_TO_BYPASS   = 0x08  // Stream-to-Bypass mode: Start streaming then revert to bypass.
} lsm6dsl_fifo_mode_t;

typedef enum {
	LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz 		 = 0x08,
	LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz 		 = 0x10,
	LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz 		 = 0x18,
	LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz 		 = 0x20,
	LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz 		 = 0x28,
	LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz 		 = 0x30,
	LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz 		 = 0x38,
	LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz 		 = 0x40,
	LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz 		 = 0x48,
	LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz 		 = 0x50,
	LSM6DS3_ACC_GYRO_ODR_FIFO_13300Hz 		 = 0x58,
} LSM6DS3_ACC_GYRO_ODR_FIFO_t;

// Example class
class LSM6DSL {
  public:
    // Constructor: pass in a TwoWire reference and the I2C address
    LSM6DSL(TwoWire &wirePort = Wire, uint8_t i2cAddress = LSM6DSL_DEFAULT_ADDRESS);

    // Initializes the sensor, returns true if successful
    bool begin();

    // Read the WHO_AM_I register (should return 0x6A or 0x6B for LSM6DSL/LSM6DS3 variant)
    uint8_t whoAmI();

    // Accelerometer config
    bool setAccelODR(lsm6dsl_xl_odr_t odr);
    bool setAccelFS(lsm6dsl_xl_fs_t fs);
    bool setAccelConfig(lsm6dsl_xl_odr_t odr, lsm6dsl_xl_fs_t fs);


    // Gyroscope config
    bool setGyroODR(lsm6dsl_g_odr_t odr);
    bool setGyroFS(lsm6dsl_g_fs_t fs);

    // Simple data read
    void readAccelerometer(int16_t &x, int16_t &y, int16_t &z);
    void readGyroscope(int16_t &x, int16_t &y, int16_t &z);
    int16_t readTemperature(); // If you’d like to read onboard temp

    bool initMagnetometer();
    bool readMagnetometer(int16_t &x, int16_t &y, int16_t &z);
    
    void readAccelerometer_g(float &x, float &y, float &z, lsm6dsl_xl_fs_t fs);
    void readGyroscope_dps(float &x, float &y, float &z, lsm6dsl_g_fs_t fs);
    void readMagnetometer_uT(float &x, float &y, float &z);
    bool enableAccelerometer(lsm6dsl_xl_odr_t odr);
    bool disableAccelerometer();

    bool enableGyroscope(lsm6dsl_g_odr_t odr);
    bool disableGyroscope();

    bool enableMagnetometer();
    bool disableMagnetometer();

    bool isAccelEnabled() const { return _accelEnabled; }
    bool isGyroEnabled()  const { return _gyroEnabled; }

    lsm6dsl_xl_fs_t getAccelFS() const { return _accelFS; }
    lsm6dsl_g_fs_t  getGyroFS()  const { return _gyroFS; }

    float convertRawAccelToG(int16_t raw, lsm6dsl_xl_fs_t fs);
    float convertRawGyroToDPS(int16_t raw, lsm6dsl_g_fs_t fs);
    float convertRawMagTouT(int16_t raw);

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);




  private:
    // Low-level read/write
    
    void readRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length);

    TwoWire *_wire;
    uint8_t _i2cAddress;

    bool _accelEnabled = false;
    bool _gyroEnabled = false;

    lsm6dsl_xl_odr_t _accelODR = LSM6DSL_XL_ODR_POWER_DOWN;
    lsm6dsl_xl_fs_t  _accelFS  = LSM6DSL_XL_FS_2g;

    lsm6dsl_g_odr_t _gyroODR = LSM6DSL_G_ODR_POWER_DOWN;
    lsm6dsl_g_fs_t  _gyroFS  = LSM6DSL_G_FS_245dps;

};

#endif // LSM6DSL_H
