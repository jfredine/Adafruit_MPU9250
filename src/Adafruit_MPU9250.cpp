
/*!
 *  @file Adafruit_MPU9250.cpp
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  John Fredine
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include <Adafruit_MPU9250.h>

/*!
 *    @brief  Instantiates a new MPU9250 class
 */
Adafruit_MPU9250::Adafruit_MPU9250(void) {}

/*!
 *    @brief  Cleans up the MPU9250 class
 */
Adafruit_MPU9250::~Adafruit_MPU9250(void) {
  if (temp_sensor)
    delete temp_sensor;
  if (accel_sensor)
    delete accel_sensor;
  if (gyro_sensor)
    delete gyro_sensor;
  if (mag_sensor)
    delete mag_sensor;
  if (i2c_dev)
    delete i2c_dev;
  if (ak8963_i2c_dev)
    delete ak8963_i2c_dev;
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return 0 on success or a non-zero error code on failure
 */

int Adafruit_MPU9250::begin(uint8_t i2c_address, TwoWire *wire,
                             int32_t sensor_id) {
  delay(1000);
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  if (ak8963_i2c_dev) {
    delete ak8963_i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  // For boards with I2C bus power control, may need to delay to allow
  // MPU9250 to come up after initial power.
  bool mpu_found = false;
  for (uint8_t tries = 0; tries < 5; tries++) {
    mpu_found = i2c_dev->begin();
    if (mpu_found)
      break;
    delay(10);
  }
  if (!mpu_found)
    return 1;

  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_WHO_AM_I, 1);

  // make sure we're talking to the right chip
  if (chip_id.read() != MPU9250_DEVICE_ID) {
    return 2;
  }

  // Make the magnetometer visible on I2C bus by setting bypass mode
  setI2CBypass(true);

  ak8963_i2c_dev = new Adafruit_I2CDevice(MPU9250_AK8963_I2CADDR, wire);

  // Check that the magnetometer is now visible
  Adafruit_BusIO_Register ak8963_chip_id =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_WHO_AM_I, 1);

  // make sure we're talking to the right chip
  if (ak8963_chip_id.read() != MPU9250_AK8963_DEVICE_ID) {
    return 3;
  }

  return _init(sensor_id) ? 0 : 4;
}

/*!  @brief Initilizes the sensor
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_MPU9250::_init(int32_t sensor_id) {

  _sensorid_temp = sensor_id;
  _sensorid_accel = sensor_id + 1;
  _sensorid_gyro = sensor_id + 2;
  _sensorid_mag = sensor_id + 3;

  reset();

  setSampleRateDivisor(0);

  setGyroFilterBandwidth(MPU9250_GYRO_BAND_250_HZ);
  setAccelFilterBandwidth(MPU9250_ACCEL_BAND_460_HZ);

  setGyroRange(MPU9250_RANGE_500_DEG);

  setAccelerometerRange(MPU9250_RANGE_2_G); // already the default

  Adafruit_BusIO_Register power_mgmt_1 =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);
  power_mgmt_1.write(0x01); // set clock config to PLL with Gyro X reference

  // enter fuse access mode of mag which is required to read adjustment data
  setMagMode(MPU9250_AK8963_MODE_FUSE_ROM);
  
  // read three bytes of sensitivity adjustment data (one byte for each axis)
  uint8_t raw[3];
  Adafruit_BusIO_Register ak8963_asa =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_ASAX, 3);
  ak8963_asa.read(raw, 3);

  // store sensitivity adjustment per axis (see data sheet for calculation)
  asax = (static_cast<float>(raw[0]) - 128) / 256 + 1;
  asay = (static_cast<float>(raw[1]) - 128) / 256 + 1;
  asaz = (static_cast<float>(raw[2]) - 128) / 256 + 1;

  // Finish setting up magnetometer
  //setMagMode(MPU9250_AK8963_MODE_100HZ);
  setMagMode(MPU9250_AK8963_MODE_100HZ);
  setMagSensitivity(MPU9250_AK8963_SENSITIVITY_16);
  delay(100);

  // remove old reference
  if (temp_sensor)
    delete temp_sensor;
  if (accel_sensor)
    delete accel_sensor;
  if (gyro_sensor)
    delete gyro_sensor;
  if (mag_sensor)
    delete mag_sensor;

  temp_sensor = new Adafruit_MPU9250_Temp(this);
  accel_sensor = new Adafruit_MPU9250_Accelerometer(this);
  gyro_sensor = new Adafruit_MPU9250_Gyro(this);
  mag_sensor = new Adafruit_MPU9250_Magnetometer(this);

  return true;
}
/**************************************************************************/
/*!
    @brief Resets registers to their initial value and resets the sensors'
    analog and digital signal paths.
*/
/**************************************************************************/
void Adafruit_MPU9250::reset(void) {
  Adafruit_BusIO_Register power_mgmt_1 =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);
  Adafruit_BusIO_Register sig_path_reset =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_SIGNAL_PATH_RESET, 1);
  Adafruit_BusIO_RegisterBits device_reset =
      Adafruit_BusIO_RegisterBits(&power_mgmt_1, 1, 7);

  // see register map page 41
  device_reset.write(1);             // reset
  while (device_reset.read() == 1) { // check for the post reset value
    delay(1);
  }
  delay(100);

  sig_path_reset.write(0x7);

  delay(100);

  // Make the magnetometer visible on I2C bus by setting bypass mode
  setI2CBypass(true);

  // reset magnetometer
  Adafruit_BusIO_Register cntl_2 = 
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_CNTL2, 1);
  Adafruit_BusIO_RegisterBits ak8963_reset =
      Adafruit_BusIO_RegisterBits(&cntl_2, 1, 0);
  ak8963_reset.write(1);
  delay(100);
}

/**************************************************************************/
/*!
    @brief Gets the sample rate divisor.
    @return  The sample rate divisor
*/
/**************************************************************************/
uint8_t Adafruit_MPU9250::getSampleRateDivisor(void) {
  Adafruit_BusIO_Register sample_rate_div =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_SMPLRT_DIV, 1);
  return sample_rate_div.read();
}

/**************************************************************************/
/*!
    @brief  Sets the divisor used to divide the base clock rate into a
            measurement rate
    @param  divisor
            The new clock divisor
*/
/**************************************************************************/
void Adafruit_MPU9250::setSampleRateDivisor(uint8_t divisor) {
  Adafruit_BusIO_Register sample_rate_div =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_SMPLRT_DIV, 1);
  sample_rate_div.write(divisor);
}

/**************************************************************************/
/*!
    @brief Gets the acceleration measurement range.
    @return  The acceleration measurement range
*/
/**************************************************************************/
mpu9250_accel_range_t Adafruit_MPU9250::getAccelerometerRange(void) {
  Adafruit_BusIO_Register accel_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_ACCEL_CONFIG, 1);
  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config, 2, 3);

  return (mpu9250_accel_range_t)accel_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range
    @param  new_range
            The new range to set. Must be a `mpu9250_accel_range_t`
*/
/**************************************************************************/
void Adafruit_MPU9250::setAccelerometerRange(mpu9250_accel_range_t new_range) {
  Adafruit_BusIO_Register accel_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_ACCEL_CONFIG, 1);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config, 2, 3);
  accel_range.write(new_range);
}
/**************************************************************************/
/*!
    @brief Gets the gyroscope measurement range
    @return  The `mpu9250_gyro_range_t` gyroscope measurement range
*/
/**************************************************************************/
mpu9250_gyro_range_t Adafruit_MPU9250::getGyroRange(void) {
  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_GYRO_CONFIG, 1);
  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&gyro_config, 2, 3);

  return (mpu9250_gyro_range_t)gyro_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the gyroscope measurement range
    @param  new_range
            The new range to set. Must be a `mpu9250_gyro_range_t`
*/
/**************************************************************************/
void Adafruit_MPU9250::setGyroRange(mpu9250_gyro_range_t new_range) {
  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_GYRO_CONFIG, 1);
  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&gyro_config, 2, 3);

  gyro_range.write(new_range);
}

/**************************************************************************/
/*!
    @brief Gets the magnetometer mode
    @return  The `mpu9250_ak8963_mag_mode_t` magnetometer mode
*/
/**************************************************************************/
mpu9250_ak8963_mag_mode_t Adafruit_MPU9250::getMagMode(void) {
  Adafruit_BusIO_Register control =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_CNTL, 1);
  Adafruit_BusIO_RegisterBits mag_mode =
      Adafruit_BusIO_RegisterBits(&control, 4, 0);

  switch (mag_mode.read()) {
      case 0:
          return MPU9250_AK8963_MODE_POWER_DOWN;
      case 1:
          return MPU9250_AK8963_MODE_SINGLE;
      case 2:
          return MPU9250_AK8963_MODE_8HZ;
      case 4:
          return MPU9250_AK8963_MODE_EXT_TRIG;
      case 6:
          return MPU9250_AK8963_MODE_100HZ;
      case 8:
          return MPU9250_AK8963_MODE_SELF_TEST;
      case 15:
          return MPU9250_AK8963_MODE_FUSE_ROM;
      default:
          return MPU9250_AK8963_MODE_POWER_DOWN;
  }
}

/**************************************************************************/
/*!
    @brief Gets the magnetometer mode string
    @return  The `mpu9250_ak8963_mag_mode_t` magnetometer mode
*/
/**************************************************************************/
const char *Adafruit_MPU9250::getMagModeString(mpu9250_ak8963_mag_mode_t mode) {

  switch (mode) {
      case MPU9250_AK8963_MODE_SINGLE:
          return "MPU9250_AK8963_MODE_SINGLE";
      case MPU9250_AK8963_MODE_8HZ:
          return "MPU9250_AK8963_MODE_8HZ";
      case MPU9250_AK8963_MODE_EXT_TRIG:
          return "MPU9250_AK8963_MODE_EXT_TRIG";
      case MPU9250_AK8963_MODE_100HZ:
          return "MPU9250_AK8963_MODE_100HZ";
      case MPU9250_AK8963_MODE_SELF_TEST:
          return "MPU9250_AK8963_MODE_SELF_TEST";
      case MPU9250_AK8963_MODE_FUSE_ROM:
          return "MPU9250_AK8963_MODE_FUSE_ROM";
      case MPU9250_AK8963_MODE_POWER_DOWN:
          return "MPU9250_AK8963_MODE_POWER_DOWN";
      default:
          return "UNKNOWN";
  }
}

/**************************************************************************/
/*!
    @brief Sets the magnetometer mode
    @param  new_mode
            The new mode to set. Must be a `mpu9250_ak8963_mag_mode_t`
*/
/**************************************************************************/
void Adafruit_MPU9250::setMagMode(mpu9250_ak8963_mag_mode_t new_mode) {
  Adafruit_BusIO_Register control =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_CNTL, 1);
  Adafruit_BusIO_RegisterBits mag_mode =
      Adafruit_BusIO_RegisterBits(&control, 4, 0);

  mag_mode.write(MPU9250_AK8963_MODE_POWER_DOWN);
  delay(10);
  switch (new_mode) {
      case MPU9250_AK8963_MODE_SINGLE:
          mag_mode.write(MPU9250_AK8963_MODE_SINGLE);
          break;
      case MPU9250_AK8963_MODE_8HZ:
          mag_mode.write(MPU9250_AK8963_MODE_8HZ);
          break;
      case MPU9250_AK8963_MODE_EXT_TRIG:
          mag_mode.write(MPU9250_AK8963_MODE_EXT_TRIG);
          break;
      case MPU9250_AK8963_MODE_100HZ:
          mag_mode.write(MPU9250_AK8963_MODE_100HZ);
          break;
      case MPU9250_AK8963_MODE_SELF_TEST:
          mag_mode.write(MPU9250_AK8963_MODE_SELF_TEST);
          break;
      case MPU9250_AK8963_MODE_FUSE_ROM:
          mag_mode.write(MPU9250_AK8963_MODE_FUSE_ROM);
          break;
      default:
          mag_mode.write(MPU9250_AK8963_MODE_POWER_DOWN);
          break;
  }
  delay(10);
}

/**************************************************************************/
/*!
    @brief Gets the magnetometer sensitivity
    @return  The `mpu9250_ak8963_mag_sensitivity_t` magnetometer sensitivity
*/
/**************************************************************************/
mpu9250_ak8963_mag_sensitivity_t Adafruit_MPU9250::getMagSensitivity(void) {
  Adafruit_BusIO_Register control =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_CNTL, 1);
  Adafruit_BusIO_RegisterBits mag_sensitivity =
      Adafruit_BusIO_RegisterBits(&control, 1, 4);

  return mag_sensitivity.read() ? MPU9250_AK8963_SENSITIVITY_16
                                : MPU9250_AK8963_SENSITIVITY_14;
}

/**************************************************************************/
/*!
    @brief Sets the magnetometer sensitivity
    @param  new_sensitivity
            The new sensitivity to set. Must be a `mpu9250_ak8963_mag_sensitivity_t`
*/
/**************************************************************************/
void Adafruit_MPU9250::setMagSensitivity(mpu9250_ak8963_mag_sensitivity_t new_range) {
  Adafruit_BusIO_Register control =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_CNTL, 1);
  Adafruit_BusIO_RegisterBits mag_sensitivity =
      Adafruit_BusIO_RegisterBits(&control, 1, 4);

  mag_sensitivity.write(new_range);
}

/**************************************************************************/
/*!
    @brief Sets clock source.
    @param  new_clock
            The clock source to set. Must be a `mpu9250_clock_select_t`
*/
/**************************************************************************/
void Adafruit_MPU9250::setClock(mpu9250_clock_select_t new_clock) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits clock_select =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 3, 0);
  clock_select.write(new_clock);
}

/**************************************************************************/
/*!
    @brief Gets clock source.
    @return  The current `mpu9250_clock_select_t` clock source
*/
/**************************************************************************/
mpu9250_clock_select_t Adafruit_MPU9250::getClock(void) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits clock_select =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 3, 0);
  return (mpu9250_clock_select_t)clock_select.read();
}

/**************************************************************************/
/*!
 *     @brief  Sets the location that the FSYNC pin sample is stored
 *     @return fsync_output
 */
/**************************************************************************/
mpu9250_fsync_out_t Adafruit_MPU9250::getFsyncSampleOutput(void) {
  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_CONFIG, 1);
  Adafruit_BusIO_RegisterBits fsync_out =
      Adafruit_BusIO_RegisterBits(&config, 3, 3);
  return (mpu9250_fsync_out_t)fsync_out.read();
}

/**************************************************************************/
/*!
*     @brief  Sets the location that the FSYNC pin sample is stored
*     @param  fsync_output
              a `mpu9250_fsync_out_t` to specify the LSB of which data register
              should be used to store the state of the FSYNC pin
*/
/**************************************************************************/
void Adafruit_MPU9250::setFsyncSampleOutput(mpu9250_fsync_out_t fsync_output) {
  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_CONFIG, 1);
  Adafruit_BusIO_RegisterBits fsync_out =
      Adafruit_BusIO_RegisterBits(&config, 3, 3);
  fsync_out.write(fsync_output);
}

/**************************************************************************/
/*!
 *     @brief  Gets bandwidth of the Gyro/Temp Digital Low Pass Filter
 *     @return  The current `mpu9250_gyro_bandwidth_t` filter bandwidth
 */
/**************************************************************************/
mpu9250_gyro_bandwidth_t Adafruit_MPU9250::getGyroFilterBandwidth(void) {
  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_CONFIG, 1);

  Adafruit_BusIO_RegisterBits filter_config =
      Adafruit_BusIO_RegisterBits(&config, 3, 0);

  uint8_t u = filter_config.read();

  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_GYRO_CONFIG, 1);

  Adafruit_BusIO_RegisterBits fchoice_b =
      Adafruit_BusIO_RegisterBits(&gyro_config, 2, 0);

  uint8_t v = fchoice_b.read();

  if (v & 0x1) {
      return MPU9250_GYRO_BAND_8800_HZ;
  } else if (v == 0x2) {
      return MPU9250_GYRO_BAND_3600_HZ;
  } else {
      return (mpu9250_gyro_bandwidth_t)(u);
  }
}

/**************************************************************************/
/*!
 *    @brief Sets the bandwidth of the Gyro/Temp Digital Low-Pass Filter
 *    @param bandwidth the new `mpu9250_gyro_bandwidth_t` bandwidth
 */
/**************************************************************************/
void Adafruit_MPU9250::setGyroFilterBandwidth(mpu9250_gyro_bandwidth_t bandwidth) {
  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_GYRO_CONFIG, 1);

  Adafruit_BusIO_RegisterBits fchoice_b =
      Adafruit_BusIO_RegisterBits(&gyro_config, 2, 0);

  fchoice_b.write((bandwidth >> 3) & 0x3);

  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_CONFIG, 1);

  Adafruit_BusIO_RegisterBits filter_config =
      Adafruit_BusIO_RegisterBits(&config, 3, 0);

  filter_config.write(bandwidth & 0x7);
}

/**************************************************************************/
/*!
 *     @brief  Gets bandwidth of the accelerometer Digital Low Pass Filter
 *     @return  The current `mpu9250_accel_bandwidth_t` filter bandwidth
 */
/**************************************************************************/
mpu9250_accel_bandwidth_t Adafruit_MPU9250::getAccelFilterBandwidth(void) {
  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_ACCEL_CONFIG2, 1);

  Adafruit_BusIO_RegisterBits filter_config =
      Adafruit_BusIO_RegisterBits(&config, 4, 0);

  uint8_t u = filter_config.read();
  if (u & 0x8) {
    return  MPU9250_ACCEL_BAND_1130_HZ;
  } else if (u == 0x7) {
    return  MPU9250_ACCEL_BAND_460_HZ;
  } else {
    return (mpu9250_accel_bandwidth_t)u;
  }
}

/**************************************************************************/
/*!
 *    @brief Sets the bandwidth of the Accelerometer Digital Low-Pass Filter
 *    @param bandwidth the new `mpu9250_accel_bandwidth_t` bandwidth
 */
/**************************************************************************/
void Adafruit_MPU9250::setAccelFilterBandwidth(mpu9250_accel_bandwidth_t bandwidth) {
  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_ACCEL_CONFIG2, 1);

  Adafruit_BusIO_RegisterBits filter_config =
      Adafruit_BusIO_RegisterBits(&config, 4, 0);

  filter_config.write(bandwidth);
}

/**************************************************************************/
/*!
*     @brief  Sets the polarity of the INT pin when active
*     @param  active_low
              If `true` the pin will be low when an interrupt is active
              If `false` the pin will be high when an interrupt is active
*/
/**************************************************************************/
void Adafruit_MPU9250::setInterruptPinPolarity(bool active_low) {
  Adafruit_BusIO_Register int_pin_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_INT_PIN_CONFIG, 1);
  Adafruit_BusIO_RegisterBits int_level =
      Adafruit_BusIO_RegisterBits(&int_pin_config, 1, 7);
  int_level.write(active_low);
}

/**************************************************************************/
/*!
*     @brief  Sets the latch behavior of the INT pin when active
*     @param  held
              If `true` the pin will remain held until cleared
              If `false` the pin will reset after a 50us pulse
*/
/**************************************************************************/
void Adafruit_MPU9250::setInterruptPinLatch(bool held) {
  Adafruit_BusIO_Register int_pin_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_INT_PIN_CONFIG, 1);
  Adafruit_BusIO_RegisterBits int_latch =
      Adafruit_BusIO_RegisterBits(&int_pin_config, 1, 5);
  int_latch.write(held);
}

/**************************************************************************/
/*!
*     @brief  Sets the motion interrupt
*     @param  active
              If `true` motion interrupt will activate based on thr and dur
              If `false` motion interrupt will be disabled
*/
/**************************************************************************/
void Adafruit_MPU9250::setMotionInterrupt(bool active) {
  Adafruit_BusIO_Register int_enable =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_INT_ENABLE, 1);
  Adafruit_BusIO_RegisterBits int_motion =
      Adafruit_BusIO_RegisterBits(&int_enable, 1, 6);
  int_motion.write(active);
}

/**************************************************************************/
/*!
 *     @brief  Gets motion interrupt status
 *     @return  motion_interrupt
 */
/**************************************************************************/
bool Adafruit_MPU9250::getMotionInterruptStatus(void) {
  Adafruit_BusIO_Register status =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_INT_STATUS, 1);

  Adafruit_BusIO_RegisterBits motion =
      Adafruit_BusIO_RegisterBits(&status, 1, 6);
  return (bool)motion.read();
}

/**************************************************************************/
/*!
 *     @brief  Sets the motion detection threshold
 *     @param  thr
 */
/**************************************************************************/
void Adafruit_MPU9250::setMotionDetectionThreshold(uint8_t thr) {
  Adafruit_BusIO_Register threshold =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_MOT_THR, 1);
  threshold.write(thr);
}

/**************************************************************************/
/*!
*     @brief  Connects or disconects the I2C master pins to the main I2C pins
*     @param  bypass
              If `true` the I2C Master pins are connected to the main I2C pins,
              bypassing the I2C Master functions of the sensor
              If `false` the I2C Master pins are controlled by the I2C master
              functions of the sensor
*/
/**************************************************************************/
void Adafruit_MPU9250::setI2CBypass(bool bypass) {
  Adafruit_BusIO_Register int_pin_config =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_INT_PIN_CONFIG, 1);
  Adafruit_BusIO_RegisterBits i2c_bypass =
      Adafruit_BusIO_RegisterBits(&int_pin_config, 1, 1);

  Adafruit_BusIO_Register user_ctrl =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_USER_CTRL, 1);
  Adafruit_BusIO_RegisterBits i2c_master_enable =
      Adafruit_BusIO_RegisterBits(&user_ctrl, 1, 5);

  i2c_bypass.write(bypass);
  i2c_master_enable.write(!bypass);
}

/**************************************************************************/
/*!
*     @brief  Controls the sleep state of the sensor
*     @param  enable
              If `true` the sensor is put into a low power state
              and measurements are halted until sleep mode is deactivated
              Setting `false` wakes up the sensor from sleep mode,
              resuming measurements.
      @returns True or false on successful write
*/
/**************************************************************************/
bool Adafruit_MPU9250::enableSleep(bool enable) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits sleep =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 1, 6);
  return sleep.write(enable);
}

/**************************************************************************/
/*!
*     @brief  Controls sensor's 'Cycle' measurement mode
*     @param  enable
              If `true` the sensor will take measurements at the rate
              set by calling `setCycleRate`, sleeping between measurements.
              *Setting the sensor into 'Cycle' mode will have no effect
              if the sensor has been put into a sleep state with `enableSleep`
              Setting `false` returns the sensor to the normal
              measurement mode.
      @returns True or false on successful write
*/
/**************************************************************************/
bool Adafruit_MPU9250::enableCycle(bool enable) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits cycle =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 1, 5);
  return cycle.write(enable);
}

/**************************************************************************/
/*!
 *     @brief  Gets the frequencey of measurements in 'Cycle' mode
 *     @return  The current 'Cycle' measurement frequency
 */
/**************************************************************************/
mpu9250_cycle_rate_t Adafruit_MPU9250::getCycleRate(void) {
  Adafruit_BusIO_Register pwr_mgmt_2 =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_2, 1);

  Adafruit_BusIO_RegisterBits cycle_rate =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt_2, 2, 6);
  return (mpu9250_cycle_rate_t)cycle_rate.read();
}

/**************************************************************************/
/*!
 *     @brief  Sets the frequency of measurement in 'Cycle' mode
 *     @param  rate
 *              The `mpu9250_cycle_rate_t` specifying the desired
 *              measurement rate
 */
/**************************************************************************/
void Adafruit_MPU9250::setCycleRate(mpu9250_cycle_rate_t rate) {
  Adafruit_BusIO_Register pwr_mgmt_2 =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_2, 1);

  Adafruit_BusIO_RegisterBits cycle_rate =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt_2, 2, 6);
  cycle_rate.write(rate);
}

/**************************************************************************/
/*!
 *     @brief  Sets standbye mode for each of the gyroscope axes.
 *     @param  xAxisStandby
 *             If `true` the gyroscope stops sensing in the X-axis.
 *             Setting `false` resumes X-axis sensing.
 *     @param  yAxisStandby
 *             If `true` the gyroscope stops sensing in the Y-axis.
 *             Setting `false` resumes Y-axis sensing.
 *     @param  zAxisStandby
 *             If `true` the gyroscope stops sensing in the Z-axis.
 *             Setting `false` resumes Z-axis sensing.
 *     @return True if setting was successful, otherwise false.
 */
/**************************************************************************/
bool Adafruit_MPU9250::setGyroStandby(bool xAxisStandby, bool yAxisStandby,
                                      bool zAxisStandby) {
  Adafruit_BusIO_Register pwr_mgmt_2 =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_2, 1);

  Adafruit_BusIO_RegisterBits gyro_stdby =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt_2, 3, 0);
  return gyro_stdby.write(xAxisStandby << 2 | yAxisStandby << 1 | zAxisStandby);
}

/**************************************************************************/
/*!
 *     @brief  Sets standbye mode for each of the accelerometer axes.
 *     @param  xAxisStandby
 *             If `true` the accelerometer stops sensing in the X-axis.
 *             Setting `false` resumes X-axis sensing.
 *     @param  yAxisStandby
 *             If `true` the accelerometer stops sensing in the Y-axis.
 *             Setting `false` resumes Y-axis sensing.
 *     @param  zAxisStandby
 *             If `true` the accelerometer stops sensing in the Z-axis.
 *             Setting `false` resumes Z-axis sensing.
 *     @return True if setting was successful, otherwise false.
 */
/**************************************************************************/
bool Adafruit_MPU9250::setAccelerometerStandby(bool xAxisStandby,
                                               bool yAxisStandby,
                                               bool zAxisStandby) {
  Adafruit_BusIO_Register pwr_mgmt_2 =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_2, 1);

  Adafruit_BusIO_RegisterBits accel_stdby =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt_2, 3, 3);
  return accel_stdby.write(xAxisStandby << 2 | yAxisStandby << 1 |
                           zAxisStandby);
}

/**************************************************************************/
/*!
 *     @brief  Sets disable mode for thermometer sensor.
 *     @param  enable
 *             If `true` the temperature sensor will stop taking measurements.
 *             Setting `false` returns the temperature sensor to the normal
 *             measurement mode.
 *     @return True if setting was successful, otherwise false.
 */
/**************************************************************************/
bool Adafruit_MPU9250::setTemperatureStandby(bool enable) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits temp_stdby =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 1, 3);
  return temp_stdby.write(enable);
}

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void Adafruit_MPU9250::_read(void) {

  // get raw readings of temp, accel, and gyro
  Adafruit_BusIO_Register data_reg =
      Adafruit_BusIO_Register(i2c_dev, MPU9250_ACCEL_OUT, 14);

  uint8_t buffer[14];
  data_reg.read(buffer, 14);

  rawAccX = buffer[0] << 8 | buffer[1];
  rawAccY = buffer[2] << 8 | buffer[3];
  rawAccZ = buffer[4] << 8 | buffer[5];

  rawTemp = buffer[6] << 8 | buffer[7];

  rawGyroX = buffer[8] << 8 | buffer[9];
  rawGyroY = buffer[10] << 8 | buffer[11];
  rawGyroZ = buffer[12] << 8 | buffer[13];

  // adjust raw data for each sensor
  temperature = (rawTemp / 340.0) + 36.53;

  mpu9250_accel_range_t accel_range = getAccelerometerRange();

  float accel_scale = 1;
  if (accel_range == MPU9250_RANGE_16_G)
    accel_scale = 2048;
  if (accel_range == MPU9250_RANGE_8_G)
    accel_scale = 4096;
  if (accel_range == MPU9250_RANGE_4_G)
    accel_scale = 8192;
  if (accel_range == MPU9250_RANGE_2_G)
    accel_scale = 16384;

  accX = ((float)rawAccX) / accel_scale;
  accY = ((float)rawAccY) / accel_scale;
  accZ = ((float)rawAccZ) / accel_scale;

  mpu9250_gyro_range_t gyro_range = getGyroRange();

  float gyro_scale = 1;
  if (gyro_range == MPU9250_RANGE_250_DEG)
    gyro_scale = 131;
  if (gyro_range == MPU9250_RANGE_500_DEG)
    gyro_scale = 65.5;
  if (gyro_range == MPU9250_RANGE_1000_DEG)
    gyro_scale = 32.8;
  if (gyro_range == MPU9250_RANGE_2000_DEG)
    gyro_scale = 16.4;

  gyroX = ((float)rawGyroX) / gyro_scale;
  gyroY = ((float)rawGyroY) / gyro_scale;
  gyroZ = ((float)rawGyroZ) / gyro_scale;

  Adafruit_BusIO_Register mag_st1 =
      Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_ST1, 1);
  Adafruit_BusIO_RegisterBits drdy =
      Adafruit_BusIO_RegisterBits(&mag_st1, 1, 0);

  // only update sensor data if the magnetometer has a new data
  if (drdy.read()) {
      Adafruit_BusIO_Register mag_data_reg =
          Adafruit_BusIO_Register(ak8963_i2c_dev, MPU9250_AK8963_HXL, 7);
      mag_data_reg.read(buffer, 7);

      //  ignore new data if there was overflow
      uint8_t st2 = buffer[6];
      if ((st2 & 0x8) == 0) {
          rawMagX = buffer[1] << 8 | buffer[0];
          rawMagY = buffer[3] << 8 | buffer[2];
          rawMagZ = buffer[5] << 8 | buffer[4];

          mpu9250_ak8963_mag_sensitivity_t mag_sensitivity = getMagSensitivity();
          float mag_scale;
          if (mag_sensitivity == MPU9250_AK8963_SENSITIVITY_16)
            // 16b sensitivity
            mag_scale = 0.15;
          else
            // 14b sensitivity
            mag_scale = 0.6;
          
          // The magnetometer in the MPU9250 is not aligned the same way as
          // the accelerometer and gyro.  The x and y axes of the acceleromter
          // are swapped when aligned with the coordinate system of the
          // magnetometer and the z directions are reversed.  Adjust the
          // assignments of magX, magY, and magZ so the coodinate systems match
          magX = asay * rawMagY * mag_scale;
          magY = asax * rawMagX * mag_scale;
          magZ = -(asaz * rawMagZ * mag_scale);
#ifdef DEBUG_SERIAL
          char debug_buffer[1000];
          snprintf(debug_buffer, sizeof(debug_buffer),
                   "asax = %f, rawMagX = %d, mag_scale = %f, magX = %f", 
                   asax, rawMagX, mag_scale, magX);
          DEBUG_SERIAL.println(debug_buffer);
          snprintf(debug_buffer, sizeof(debug_buffer),
                   "asay = %f, rawMagY = %d, mag_scale = %f, magY = %f", 
                   asay, rawMagY, mag_scale, magY);
          DEBUG_SERIAL.println(debug_buffer);
          snprintf(debug_buffer, sizeof(debug_buffer),
                   "asaz = %f, rawMagZ = %d, mag_scale = %f, magZ = %f", 
                   asaz, rawMagZ, mag_scale, magZ);
          DEBUG_SERIAL.println(debug_buffer);
#endif
      }
  }
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  accel
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with acceleration event data.
    @param  gyro
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with gyroscope event data.
    @param  temp
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with temperature event data.
    @return True on successful read
*/
/**************************************************************************/
bool Adafruit_MPU9250::getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                                sensors_event_t *temp) {
  uint32_t timestamp = millis();
  _read();

  fillTempEvent(temp, timestamp);
  fillAccelEvent(accel, timestamp);
  fillGyroEvent(gyro, timestamp);

  return true;
}

void Adafruit_MPU9250::fillTempEvent(sensors_event_t *temp,
                                     uint32_t timestamp) {

  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = timestamp;
  temp->temperature = temperature;
}

void Adafruit_MPU9250::fillAccelEvent(sensors_event_t *accel,
                                      uint32_t timestamp) {

  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = timestamp;
  accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD;
  accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD;
  accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD;
}

void Adafruit_MPU9250::fillGyroEvent(sensors_event_t *gyro,
                                     uint32_t timestamp) {
  memset(gyro, 0, sizeof(sensors_event_t));
  gyro->version = 1;
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->timestamp = timestamp;
  gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
  gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
  gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}

void Adafruit_MPU9250::fillMagEvent(sensors_event_t *mag,
                                    uint32_t timestamp) {
  memset(mag, 0, sizeof(sensors_event_t));
  mag->version = 1;
  mag->sensor_id = _sensorid_mag;
  mag->type = SENSOR_TYPE_MAGNETIC_FIELD;
  mag->timestamp = timestamp;
  mag->magnetic.x = magX;
  mag->magnetic.y = magY;
  mag->magnetic.z  = magZ;
}

/*!
  @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
  @return Adafruit_Sensor pointer to temperature sensor
*/
Adafruit_Sensor *Adafruit_MPU9250::getTemperatureSensor(void) {
  return temp_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the accelerometer
    sensor component
    @return Adafruit_Sensor pointer to accelerometer sensor
 */
Adafruit_Sensor *Adafruit_MPU9250::getAccelerometerSensor(void) {
  return accel_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the gyro sensor component
    @return Adafruit_Sensor pointer to gyro sensor
 */
Adafruit_Sensor *Adafruit_MPU9250::getGyroSensor(void) { return gyro_sensor; }

/*!
    @brief  Gets an Adafruit Unified Sensor object for the gyro sensor component
    @return Adafruit_Sensor pointer to gyro sensor
 */
Adafruit_Sensor *Adafruit_MPU9250::getMagnetometerSensor(void) { return mag_sensor; }

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's gyroscope sensor
*/
/**************************************************************************/
void Adafruit_MPU9250_Gyro::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_G", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay = 0;
  sensor->min_value = -34.91; /* -000 dps -> rad/s (radians per second) */
  sensor->max_value = +34.91;
  sensor->resolution = 1.332e-4; /* 131.5 LSB/DPS */
}

/**************************************************************************/
/*!
    @brief  Gets the gyroscope as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_MPU9250_Gyro::getEvent(sensors_event_t *event) {
  _theMPU9250->_read();
  _theMPU9250->fillGyroEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's accelerometer
*/
/**************************************************************************/
void Adafruit_MPU9250_Accelerometer::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->min_value = -156.9064F; /*  -16g = 156.9064 m/s^2  */
  sensor->max_value = 156.9064F;  /* 16g = 156.9064 m/s^2  */
  sensor->resolution = 0.061;     /* 0.061 mg/LSB at +-2g */
}

/**************************************************************************/
/*!
    @brief  Gets the accelerometer as a standard sensor event
    @param  event Sensor event object that will be populatedx
    @returns True
*/
/**************************************************************************/
bool Adafruit_MPU9250_Accelerometer::getEvent(sensors_event_t *event) {
  _theMPU9250->_read();
  _theMPU9250->fillAccelEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's tenperature
*/
/**************************************************************************/
void Adafruit_MPU9250_Temp::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40;
  sensor->max_value = 105;
  sensor->resolution = 0.00294; /* 340 LSB/C => 1/340 C/LSB */
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_MPU9250_Temp::getEvent(sensors_event_t *event) {
  _theMPU9250->_read();
  _theMPU9250->fillTempEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the MPU9250's magnetometer 
*/
/**************************************************************************/
void Adafruit_MPU9250_Magnetometer::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU9250_M", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->min_value = -4900; 
  sensor->max_value = +4900;
  sensor->resolution = 0.15;
}

/**************************************************************************/
/*!
    @brief  Gets the magnetometer as a standard sensor event
    @param  event Sensor event object that will be populatedx
    @returns True
*/
/**************************************************************************/
bool Adafruit_MPU9250_Magnetometer::getEvent(sensors_event_t *event) {
  _theMPU9250->_read();
  _theMPU9250->fillMagEvent(event, millis());

  return true;
}
