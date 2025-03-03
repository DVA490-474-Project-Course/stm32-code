/* main_cpp.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-22 by Emil Åberg
 * Description: Function that is called within main.c and serves as the main
 * entry point of the  user defined c++ code.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_MAINCPP_H
#define STM32CODE_MAINCPP_H

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

/* Declare all necessary peripheral handles as extern */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;
extern I2C_HandleTypeDef hi2c5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @brief Main entry point for the user defined C++ code.
 *
 * This function serves as the main entry point for the user defined C++
 * code. It should be called at the end of the main function in the main.c
 * file, which is generated by STM32CubeIDE.
 */
void MainCpp();

#ifdef __cplusplus
}
#endif

#endif /* STM32CODE_MAINCPP_H */
