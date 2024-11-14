/* common_types.h
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-19
 * Last modified: 2024-10-15 by Emil Åberg
 * Description: Common types used by the individual robot behaviour program.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


#ifndef STM32CODE_COMMONTYPES_H
#define STM32CODE_COMMONTYPES_H


/* Related .h files */

/* C++ standard library headers */

/* Other .h files */

/* Project .h files */


namespace stm32_code
{

enum class Status
{
  kOk = 0,
  kNotOk = -1
};

template<typename T> struct Vector3d
{
  T x;
  T y;
  T z;
  Status status;
};

template<typename T> struct Quarternion
{
  T w;
  T x;
  T y;
  T z;
  Status status;
};

template<typename T> struct Rotation
{
  T heading;
  T roll;
  T pitch;
  Status status;
};

template<typename T> struct Scalar
{
  T value;
  Status status;
};

} /* namespace stm32_code */

#endif /* STM32CODE_COMMONTYPES_H */
