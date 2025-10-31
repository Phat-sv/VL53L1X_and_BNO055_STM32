// flash.c
#include "flash.h"

uint16_t check_flash_size(void)
{
  uint16_t flash_size_kb = *FLASH_SIZE_REG;
  printf("Flash size: %d KB\r\n", flash_size_kb);
  return flash_size_kb;
}

void write_calibration_to_flash(void *data, uint32_t size)
{
  HAL_FLASH_Unlock();

  // Xóa trang (1KB trên STM32F1)
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t               pageError;

  eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = FLASH_CALIB_ADDR;
  eraseInit.NbPages     = 1;

  if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return;
  }

  // Tính số word (4 bytes) cần ghi, làm tròn lên
  uint32_t wordCount  = (size + 3) / 4;
  uint32_t paddedSize = wordCount * 4;

  // Tạo buffer tạm với kích thước đã lấp đầy
  uint8_t buffer[paddedSize];
  memset(buffer, 0x00, paddedSize);
  memcpy(buffer, data, size);

  uint32_t *pData = (uint32_t *)buffer;
  for (uint32_t i = 0; i < wordCount; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CALIB_ADDR + i * 4, pData[i]);
  }

  HAL_FLASH_Lock();
}

void read_calibration_from_flash(void *data, uint32_t size)
{
  memcpy(data, (void *)FLASH_CALIB_ADDR, size);
}
