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

struct Vector_3d
{
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

struct Rotation
{
  uint16_t heading;
  uint16_t roll;
  uint16_t pitch;
};

struct Quarternion : Vector_3d
{
  uint16_t w;
};

} /* namespace stm32_code */

#endif /* STM32CODE_COMMONTYPES_H */
