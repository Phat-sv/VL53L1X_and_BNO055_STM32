/**
 * @file       bno055_stm32_platform.h
 * @copyright
 * @license
 * @version    1.0.0
 * @date       2025-05-31
 * @author     Phat Nguyen Tan
 * @author
 *
 * @brief      <>
 *
 * @note
 * @example
 *
 * @example
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BNO055_STM32_PLATORM_H
#define __BNO055_STM32_PLATORM_H

/* Includes ----------------------------------------------------------- */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_uart.h"
#include <stdio.h>

/// update later
#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif

/* Public defines ----------------------------------------------------- */
#define BNO055_I2C_ADDR_HI 0x29
#define BNO055_I2C_ADDR_LO 0x28
#define BNO055_I2C_ADDR    BNO055_I2C_ADDR_LO  // modify here: 0x28 or 0x29

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  WRITE_SUCCESSED = 0,
  WRITE_ERROR     = 1,
  WRITE_TIMEOUT   = 2,
  WRITE_BUSY      = 3,
  WRITE_UNKNOWN   = 4,

} enum_write_command_status_t;

typedef enum
{
  READ_SUCCESSED = 0,
  READ_FAILED,
} enum_read_command_status_t;

/* Public macros ------------------------------------------------------ */

/* Public variables --------------------------------------------------- */

/* Public function prototypes ----------------------------------------- */
void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device);

void bno055_delay(uint16_t time);

enum_write_command_status_t bno055_write_data(uint8_t reg, uint8_t data);

enum_read_command_status_t bno055_read_data(uint8_t reg, uint8_t *data, uint8_t len);

#endif  // __BNO055_STM32_PLATORM_H

/* End of file -------------------------------------------------------- */
