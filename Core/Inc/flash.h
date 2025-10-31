// flash.h
#ifndef __FLASH_H
#define __FLASH_H

#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"

#define FLASH_SIZE_REG   ((uint16_t *)0x1FFFF7E0)  // STM32F1 series
#define FLASH_CALIB_ADDR 0x0800FC00                // Vi STM32F103 64KB

uint16_t check_flash_size(void);
void     write_calibration_to_flash(void *data, uint32_t size);
void     read_calibration_from_flash(void *data, uint32_t size);

#endif  // __FLASH_H
