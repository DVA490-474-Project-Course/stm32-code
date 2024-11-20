/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the nine axis BNO055 IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_SENSORINTERFACE_NINEAXISIMU_H
#define STM32CODE_SENSORINTERFACE_NINEAXISIMU_H

/* Project .h files */
#include "../common_types.h"
#include "../stm32h7xx_hal.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Class representing driver control of one BNO055 nine axis imu.
 *
 * Class representing driver control of one BNO055 nine axis imu.
 */
class NineAxisIMU
{
public:
  /*!
   * @brief Default constructor
   *
   * The default constructor.
   */
  NineAxisIMU();

  /*!
   * @brief Initializes the sensor.
   *
   * Configure the sensor to begin all measurements and enable internal sensor
   * fusion.
   *
   * @param[in] hi2c Pointer to the handle of the I2C peripheral.
   * 
   * @return Enum indicating wheter initialization was successful.
   *
   * @pre Sensor has been powered on for at least 0.5 seconds.
   *
   * @warning Testing has revealed that this sensor should be powered on for
   * roughly 0.5s before this method is called to ensure successful
   * initialization.
   */
  Status Init(I2C_HandleTypeDef* hi2c);

  /*!
   * @brief Get the measured temperature.
   *
   * Returns the measured temperature in degrees Celsius.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing temperature value and status enum indicating
   * whether sensor read was successful.
   */
  Scalar<uint8_t> GetTemperature();

  /*!
   * @brief Get the measured magnetic field vector.
   *
   * Returns the measured magnetic field vector in micro Tesla.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the magnetic field vector and status
   * enum indicating whether sensor read was successful.
   */
  Vector3d<float> GetMagneticField();

  /*!
   * @brief Get the measured rotational speed.
   *
   * Returns the measured rotational speed vector in degree/s from
   * the gyroscope.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the rotational speed and status
   * enum indicating whether sensor read was successful.
   */
  Vector3d<float> GetRotationalSpeed();

  /*!
   * @brief Get the measured acceleration.
   *
   * Returns the measured acceleration vector in m/s² from the accelerometer.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the acceleration and status enum indicating
   * whether sensor read was successful.
   * 
   * @note Unlike GetLinearAcceleration(), this method will return a
   * value where the sensor has not compensated for the acceleration
   * due to gravity.
   */
  Vector3d<float> GetAcceleration();

  /*!
   * @brief Get the measured acceleration.
   *
   * Returns the measured acceleration vector in m/s² from the accelerometer.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the acceleration and status enum indicating
   * whether sensor read was successful.
   * 
   * @note Unlike GetAcceleration(), this method will return a
   * value where the sensor has compensated for the acceleration
   * due to gravity.
   */
  Vector3d<float> GetLinearAcceleration();

  /*!
   * @brief Get the gravity vector.
   *
   * Returns the measured gravity vector vector in m/s².
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the gravity vector and status enum indicating
   * whether sensor read was successful.
   */
  Vector3d<float> GetGravity();

  /*!
   * @brief Get the orientation in euler degrees.
   *
   * Returns the measured orientation in euler degrees, relative to the
   * orientation of the sensor when it booted up from power on.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the orientation and status enum indicating
   * whether sensor read was successful.
   */
  Rotation<float> GetEulerOrientation();

  /*!
   * @brief Get the orientation in quarternion units.
   *
   * Returns the measured orientation in quarternion units, relative to the
   * orientation of the sensor when it booted up from power on.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the orientation and status enum indicating
   * whether sensor read was successful.
   */
  Quarternion<float> GetQuarternionOrientation();

private:
  /*!
   * @brief Returns a measurement of a 3D vector value.
   *
   * Returns a measurement of a 3D vector value, where measurement data
   * is read from a block of 6 registers on the sensor.
   *
   * @param[in] register_address Register address of the first byte to read.
   * @param[in] lsb_per_unit Number of least significant bits per output unit.
   *
   * @pre The data addresses are stored in 6 registers, ordered in sequence
   * of x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, starting at register_address.
   * lsb and msb stand for least and most significant bit respectively.
   */
  Vector3d<float> Get3dVector(const uint8_t register_address, float lsb_per_unit);

  /*!
   * @brief Join two 8 bit values into one 16 bit value.
   *
   * Take a 16 bit value where 8 least and 8 most significant bits are
   * separated and join them into one 16 bit variable.
   *
   * @param[in] 8 least significant bits.
   * @param[in] 8 most significant bits.
   *
   * @return Output 16 bit value.
   */
  uint16_t Join(uint8_t lsb, uint8_t msb);

  /*!
   * @brief Convert a 16 bit value in 2's complement form to signed int.
   *
   * Convert a 16 bit value in 2's complement form to signed int.
   *
   * @param[in] Value in 2's complement form.
   *
   * @return Value in signed int.
   */
  int16_t ConvertToSigned(uint16_t value);

  /*!
   * @brief Convert a 8 bit value in 2's complement form to signed int.
   *
   * Convert a 8 bit value in 2's complement form to signed int.
   *
   * @param[in] Value in 2's complement form.
   *
   * @return Value in signed int.
   */
  int8_t ConvertToSigned(uint8_t value);

  /*!
   * @brief Pointer to the handle of the I2C peripheral.
   */
  I2C_HandleTypeDef* hi2c;

  /*!
   * @brief Operating mode register.
   */
  static const uint8_t opr_mode = 0x3d;

  /*!
   * @brief "Nine Degrees Of Freedom" operation mode.
   * 
   * "Nine Degrees Of Freedom" operation mode. Writing this value to opr_mode
   * register enables all sensors and internal sensor fusion
   */
  static const uint8_t ndof = 0b1100;

  /*!
   * @brief I2C address of the IMU.
   */
  static const uint8_t i2c_address = 0x28 << 1;

  /*!
   * @brief Starting register address of the acceleration data.
   * 
   * Starting register address of the acceleration data. Acceleration data is
   * stored in this and the following 5 registers.
   */
  static const uint8_t acc_data = 0x08;

  /*!
   * @brief Starting register address of the magnetometer data.
   * 
   * Starting register address of the magnetometer data. Magnetometer data is
   * stored in this and the following 5 registers.
   */
  static const uint8_t mag_data = 0x0e;

  /*!
   * @brief Starting register address of the gyroscope data.
   * 
   * Starting register address of the gyroscope data. Gyroscope data is
   * stored in this and the following 5 registers.
   */
  static const uint8_t gyr_data = 0x14;

  /*!
   * @brief Starting register address of the Euler orientation data.
   * 
   * Starting register address of the Euler orientation data. Euler orientation
   * data is stored in this and the following 5 registers.
   */
  static const uint8_t eul_data = 0x1a;

  /*!
   * @brief Starting register address of the Quarternion orientation data.
   * 
   * Starting register address of the Quarternion orientation data. Quarternion
   * orientation data is stored in this and the following 7 registers.
   */
  static const uint8_t qua_data = 0x20;

  /*!
   * @brief Starting register address of the linear acceleration data.
   * 
   * Starting register address of the linear acceleration data. Linear
   * acceleration data is stored in this and the following 5 registers.
   */
  static const uint8_t lia_data = 0x28;

  /*!
   * @brief Starting register address of the gravity data.
   * 
   * Starting register address of the gravity data. Gravity data is
   * stored in this and the following 5 registers.
   */
  static const uint8_t grv_data = 0x2e;

  /*!
   * @brief Starting register address of the temperature data.
   */
  static const uint8_t temp_data = 0x34;

  /*!
   * @brief Least significant bits per m/s².
   */
  static constexpr float acc_lsb_per_unit = 100.0F;

  /*!
   * @brief Least significant bits per micro Tesla.
   */
  static constexpr float mag_lsb_per_unit = 16.0F;

  /*!
   * @brief Least significant bits per degrees/s.
   */
  static constexpr float gyr_lsb_per_unit = 16.0F;

  /*!
   * @brief Least significant bits per degree.
   */
  static constexpr float eul_lsb_per_unit = 16.0F;

  /*!
   * @brief Least significant bits per quarternion unit.
   */
  static constexpr float qua_lsb_per_unit = (2^24);

  /*!
   * @brief Least significant bits per m/s² for linear
   * acceleration.
   */
  static constexpr float lia_lsb_per_unit = 100.0F;

  /*!
   * @brief Least significant bits per m/s² for gravity
   * vector.
   */
  static constexpr float grv_lsb_per_unit = 100.0F;
};

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORINTERFACE_NINEAXISIMU_H */
