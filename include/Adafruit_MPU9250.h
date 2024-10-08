/*!
 *  @file Adafruit_MPU9250.h
 *
 * 	I2C Driver for MPU9250 9-DoF Accelerometer, Gyro, and magnetometer
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_MPU9250_H
#define _ADAFRUIT_MPU9250_H

#include <Arduino.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MPU9250_I2CADDR_DEFAULT 0x68         ///< MPU9250 default i2c address
#define MPU9250_AK8963_I2CADDR 0x0c  ///< Magnetometer i2c address

#define MPU9250_DEVICE_ID 0x71 ///< The correct MPU9250_WHO_AM_I value

#define MPU9250_SELF_TEST_X_GYRO                                               \
  0x00 ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Y_GYRO                                               \
  0x01 ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Z_GYRO                                               \
  0x02 ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_X_ACCEL                                              \
  0x0D ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Y_ACCEL                                              \
  0x0E ///< Self test factory calibrated values register
#define MPU9250_SELF_TEST_Z_ACCEL                                              \
  0x0F ///< Self test factory calibrated values register
#define MPU9250_SMPLRT_DIV 0x19  ///< sample rate divisor register
#define MPU9250_CONFIG 0x1A      ///< General configuration register
#define MPU9250_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU9250_ACCEL_CONFIG                                                   \
  0x1C ///< Accelerometer specific configration register
#define MPU9250_ACCEL_CONFIG2                                                  \
  0x1D ///< Additional accelerometer specific configration register
#define MPU9250_MOT_THR 0x1F ///< Motion threshold
#define MPU9250_INT_PIN_CONFIG 0x37 ///< Interrupt pin configuration register
#define MPU9250_INT_ENABLE 0x38     ///< Interrupt enable configuration register
#define MPU9250_INT_STATUS 0x3A     ///< Interrupt status register
#define MPU9250_WHO_AM_I 0x75       ///< Divice ID register
#define MPU9250_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU9250_USER_CTRL 0x6A         ///< FIFO and I2C Master control register
#define MPU9250_PWR_MGMT_1 0x6B        ///< Primary power/sleep control register
#define MPU9250_PWR_MGMT_2 0x6C ///< Secondary power/sleep control register
#define MPU9250_TEMP_H 0x41     ///< Temperature data high byte register
#define MPU9250_TEMP_L 0x42     ///< Temperature data low byte register
#define MPU9250_ACCEL_OUT 0x3B  ///< base address for sensor data reads

//Magnetometer Registers
#define MPU9250_AK8963_DEVICE_ID 0x48 ///< The correct AK8963_WHO_AM_I value

#define MPU9250_AK8963_WHO_AM_I  0x00 ///< AK8963 Device ID register
#define MPU9250_AK8963_INFO      0x01
#define MPU9250_AK8963_ST1       0x02 ///< Status 1
#define MPU9250_AK8963_HXL       0x03 ///< data out (x-low, x-high, ...)
#define MPU9250_AK8963_HXH       0x04
#define MPU9250_AK8963_HYL       0x05
#define MPU9250_AK8963_HYH       0x06
#define MPU9250_AK8963_HZL       0x07
#define MPU9250_AK8963_HZH       0x08
#define MPU9250_AK8963_ST2       0x09  ///< Status 2 
#define MPU9250_AK8963_CNTL      0x0A  ///< Control register for mode
#define MPU9250_AK8963_CNTL2     0x0B  ///< Control register for reset
#define MPU9250_AK8963_ASTC      0x0C  ///< Self test
#define MPU9250_AK8963_I2CDIS    0x0F  ///< I2C disable
#define MPU9250_AK8963_ASAX      0x10  ///< Fuse ROM x-axis sensitivity adjustment value
#define MPU9250_AK8963_ASAY      0x11  ///< Fuse ROM y-axis sensitivity adjustment value
#define MPU9250_AK8963_ASAZ      0x12  ///< Fuse ROM z-axis sensitivity adjustment value


/**
 * @brief Magnetometer operating modes
 *
 */
typedef enum {
    MPU9250_AK8963_MODE_POWER_DOWN,
    MPU9250_AK8963_MODE_SINGLE,
    MPU9250_AK8963_MODE_8HZ,
    MPU9250_AK8963_MODE_EXT_TRIG = 0x4,
    MPU9250_AK8963_MODE_100HZ = 0x6,
    MPU9250_AK8963_MODE_SELF_TEST = 0x8,
    MPU9250_AK8963_MODE_FUSE_ROM = 0xf
} mpu9250_ak8963_mag_mode_t;

/**
 * @brief Magnetometer sensitivity
 *
 */
typedef enum {
    MPU9250_AK8963_SENSITIVITY_14,
    MPU9250_AK8963_SENSITIVITY_16
} mpu9250_ak8963_mag_sensitivity_t;


/**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
typedef enum fsync_out {
  MPU9250_FSYNC_OUT_DISABLED,
  MPU9250_FSYNC_OUT_TEMP,
  MPU9250_FSYNC_OUT_GYROX,
  MPU9250_FSYNC_OUT_GYROY,
  MPU9250_FSYNC_OUT_GYROZ,
  MPU9250_FSYNC_OUT_ACCELX,
  MPU9250_FSYNC_OUT_ACCELY,
  MPU9250_FSYNC_OUT_ACCELZ,
} mpu9250_fsync_out_t;

/**
 * @brief Clock source options
 *
 * Allowed values for `setClock`.
 */
typedef enum clock_select {
  MPU9250_INTERNAL_20MHz,
  MPU9250_PLL,
  MPU9250_STOP = 7,
} mpu9250_clock_select_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
  MPU9250_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
  MPU9250_RANGE_4_G = 0b01,  ///< +/- 4g
  MPU9250_RANGE_8_G = 0b10,  ///< +/- 8g
  MPU9250_RANGE_16_G = 0b11, ///< +/- 16g
} mpu9250_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum {
  MPU9250_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
  MPU9250_RANGE_500_DEG,  ///< +/- 500 deg/s
  MPU9250_RANGE_1000_DEG, ///< +/- 1000 deg/s
  MPU9250_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu9250_gyro_range_t;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setGyroFilterBandwidth`.
 * Includes fchoice_b from gyro config and dlpf_cfg from config
 */
typedef enum {
  MPU9250_GYRO_BAND_250_HZ,   ///< 250 Hz
  MPU9250_GYRO_BAND_184_HZ,   ///< 184 Hz
  MPU9250_GYRO_BAND_92_HZ,    ///< 92 Hz
  MPU9250_GYRO_BAND_41_HZ,    ///< 41 Hz
  MPU9250_GYRO_BAND_20_HZ,    ///< 20 Hz
  MPU9250_GYRO_BAND_10_HZ,    ///< 10 Hz
  MPU9250_GYRO_BAND_5_HZ,     ///< 5 Hz
  MPU9250_GYRO_BAND_3600_HZ,  ///< 3600 Hz
  MPU9250_GYRO_BAND_8800_HZ,  ///< 8800 Hz
} mpu9250_gyro_bandwidth_t;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setAccelFilterBandwidth`.
 * Includes accel_fchoice_b from accel config 2 register
 */
typedef enum {
  MPU9250_ACCEL_BAND_460_HZ,            ///< 460 Hz
  MPU9250_ACCEL_BAND_184_HZ,            ///< 184 Hz
  MPU9250_ACCEL_BAND_92_HZ,             ///< 94 Hz
  MPU9250_ACCEL_BAND_41_HZ,             ///< 44 Hz
  MPU9250_ACCEL_BAND_20_HZ,             ///< 21 Hz
  MPU9250_ACCEL_BAND_10_HZ,             ///< 10 Hz
  MPU9250_ACCEL_BAND_5_HZ,              ///< 5 Hz
  MPU9250_ACCEL_BAND_1130_HZ = 0b1000,  ///< 1130 Hz
} mpu9250_accel_bandwidth_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum {
  MPU9250_CYCLE_1_25_HZ, ///< 1.25 Hz
  MPU9250_CYCLE_5_HZ,    ///< 5 Hz
  MPU9250_CYCLE_20_HZ,   ///< 20 Hz
  MPU9250_CYCLE_40_HZ,   ///< 40 Hz
} mpu9250_cycle_rate_t;

class Adafruit_MPU9250;

/** Adafruit Unified Sensor interface for temperature component of MPU9250 */
class Adafruit_MPU9250_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the MPU9250 class */
  Adafruit_MPU9250_Temp(Adafruit_MPU9250 *parent) { _theMPU9250 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x650;
  Adafruit_MPU9250 *_theMPU9250 = NULL;
};

/** Adafruit Unified Sensor interface for accelerometer component of MPU9250 */
class Adafruit_MPU9250_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the MPU9250 class */
  Adafruit_MPU9250_Accelerometer(Adafruit_MPU9250 *parent) {
    _theMPU9250 = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x651;
  Adafruit_MPU9250 *_theMPU9250 = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of MPU9250 */
class Adafruit_MPU9250_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the MPU9250 class */
  Adafruit_MPU9250_Gyro(Adafruit_MPU9250 *parent) { _theMPU9250 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x652;
  Adafruit_MPU9250 *_theMPU9250 = NULL;
};

/** Adafruit Unified Sensor interface for magnetometer component of MPU9250 */
class Adafruit_MPU9250_Magnetometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the magnetometer
      @param parent A pointer to the MPU9250 class */
  Adafruit_MPU9250_Magnetometer(Adafruit_MPU9250 *parent) { _theMPU9250 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x653;
  Adafruit_MPU9250 *_theMPU9250 = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MPU9250 I2C Digital Potentiometer
 */
class Adafruit_MPU9250 final {
public:
  Adafruit_MPU9250();
  ~Adafruit_MPU9250();

  int begin(uint8_t i2c_addr = MPU9250_I2CADDR_DEFAULT,
            TwoWire *wire = &Wire,
            int32_t sensorID = 0);

  // Adafruit_Sensor API/Interface
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp);

  mpu9250_accel_range_t getAccelerometerRange(void);
  void setAccelerometerRange(mpu9250_accel_range_t);

  mpu9250_gyro_range_t getGyroRange(void);
  void setGyroRange(mpu9250_gyro_range_t);

  void setInterruptPinPolarity(bool active_low);
  void setInterruptPinLatch(bool held);
  void setFsyncSampleOutput(mpu9250_fsync_out_t fsync_output);

  void setMotionInterrupt(bool active);
  void setMotionDetectionThreshold(uint8_t thr);
  bool getMotionInterruptStatus(void);

  mpu9250_fsync_out_t getFsyncSampleOutput(void);
  void setI2CBypass(bool bypass);

  void setClock(mpu9250_clock_select_t);
  mpu9250_clock_select_t getClock(void);

  void setGyroFilterBandwidth(mpu9250_gyro_bandwidth_t bandwidth);
  mpu9250_gyro_bandwidth_t getGyroFilterBandwidth(void);

  void setAccelFilterBandwidth(mpu9250_accel_bandwidth_t bandwidth);
  mpu9250_accel_bandwidth_t getAccelFilterBandwidth(void);

  void setSampleRateDivisor(uint8_t);
  uint8_t getSampleRateDivisor(void);

  bool enableSleep(bool enable);
  bool enableCycle(bool enable);

  void setCycleRate(mpu9250_cycle_rate_t rate);
  mpu9250_cycle_rate_t getCycleRate(void);

  bool setGyroStandby(bool xAxisStandby, bool yAxisStandby, bool zAxisStandby);
  bool setAccelerometerStandby(bool xAxisStandby, bool yAxisStandby,
                               bool zAxisStandby);
  bool setTemperatureStandby(bool enable);

  mpu9250_ak8963_mag_mode_t getMagMode(void);
  const char *getMagModeString(mpu9250_ak8963_mag_mode_t mag_mode);
  void setMagMode(mpu9250_ak8963_mag_mode_t mag_mode);

  mpu9250_ak8963_mag_sensitivity_t getMagSensitivity(void);
  void setMagSensitivity(mpu9250_ak8963_mag_sensitivity_t mag_sensitivity);

  void reset(void);

  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);
  Adafruit_Sensor *getMagnetometerSensor(void);

private:
  void _getRawSensorData(void);
  void _scaleSensorData(void);

protected:
  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ,         ///< Last reading's gyro Z axis in rad/s
      magX,          ///< Last reading's magnetometer X axis in uT
      magY,          ///< Last reading's magnetometer Y axis in uT
      magZ;          ///< Last reading's magnetometer Z axis in uT

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_I2CDevice *ak8963_i2c_dev = NULL; ///< Pointer to I2C bus interface

  Adafruit_MPU9250_Temp *temp_sensor = NULL; ///< Temp sensor data object
  Adafruit_MPU9250_Accelerometer *accel_sensor =
      NULL;                                  ///< Accelerometer data object
  Adafruit_MPU9250_Gyro *gyro_sensor = NULL; ///< Gyro data object
  Adafruit_MPU9250_Magnetometer *mag_sensor = NULL; ///< Magnetometer data object

  float asax, asay, asaz;  ///< mag sensitivity adjustment data

  uint16_t _sensorid_accel, ///< ID number for accelerometer
      _sensorid_gyro,       ///< ID number for gyro
      _sensorid_temp,       ///< ID number for temperature
      _sensorid_mag;       ///< ID number for temperature

  void _read(void);
  virtual bool _init(int32_t sensor_id);

private:
  friend class Adafruit_MPU9250_Temp; ///< Gives access to private members to
                                      ///< Temp data object
  friend class Adafruit_MPU9250_Accelerometer; ///< Gives access to private
                                               ///< members to Accelerometer
                                               ///< data object
  friend class Adafruit_MPU9250_Gyro; ///< Gives access to private members to
                                      ///< Gyro data object

  friend class Adafruit_MPU9250_Magnetometer; ///< Gives access to private members to
                                              ///< Mag data object

  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ,
          rawMagX, rawMagY, rawMagZ;

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
  void fillMagEvent(sensors_event_t *mag, uint32_t timestamp);
};

#endif
