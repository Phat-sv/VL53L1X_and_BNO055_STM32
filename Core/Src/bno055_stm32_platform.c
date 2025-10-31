/**
 * @file       bno055_stm32_platform.c
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

/* Includes ----------------------------------------------------------- */
#include "bno055_stm32_platform.h"

/* Private defines ---------------------------------------------------- */

/* Private enumerate/structure ---------------------------------------- */

/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
static I2C_HandleTypeDef *_bno055_i2c_port = NULL;

/* Private variables -------------------------------------------------- */

/* Private function prototypes ---------------------------------------- */

/* Function definitions ----------------------------------------------- */
void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device)
{
  _bno055_i2c_port = hi2c_device;
}

void bno055_delay(uint16_t time)
{
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

enum_write_command_status_t bno055_write_data(uint8_t reg, uint8_t data)
{
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), 800);
  if (status == HAL_OK)
  {
    return WRITE_SUCCESSED;
  }

  if (status == HAL_ERROR)
  {
    // printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
    return WRITE_ERROR;
  }
  else if (status == HAL_TIMEOUT)
  {
    // printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
    return WRITE_TIMEOUT;
  }
  else if (status == HAL_BUSY)
  {
    // printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
    return WRITE_BUSY;
  }
  else
  {
    // printf("Unknown status data %d", status);
    return WRITE_UNKNOWN;
  }
}

enum_read_command_status_t bno055_read_data(uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t tx_status, rx_status;

  tx_status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, &reg, 1, 800);
  if (tx_status != HAL_OK)
  {
    // printf("loi 1\n\r");
    return READ_FAILED;
  }

  rx_status = HAL_I2C_Master_Receive(_bno055_i2c_port, BNO055_I2C_ADDR << 1, data, len, 800);
  if (rx_status != HAL_OK)
  {
    // printf("loi 2\n\r");
    return READ_FAILED;
  }

  return READ_SUCCESSED;
}

/* Private definitions ----------------------------------------------- */

/* End of file -------------------------------------------------------- */
