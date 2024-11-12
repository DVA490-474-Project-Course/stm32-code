/* main_cpp.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Function that is called within main.c and serves as the main
 * entry point of the  user defined c++ code.
 * License: See LICENSE file for license details.
 *==============================================================================
 */
#ifndef MAINCPP_H_
#define MAINCPP_H_

#include <stdint.h>
#include <stdio.h>
#include "../stm32h7xx_hal.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* This function serves as the main entry point for the user defined c++ code */
void MainCpp();

#ifdef __cplusplus
}
#endif

#endif /* MAINCPP_H_ */
