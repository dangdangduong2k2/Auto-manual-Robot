#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"


void Flash_Delete(uint32_t pageAddress);
void Flash_Write(uint32_t Flash_Address,uint16_t Flash_Data);
uint32_t Flash_Read(uint32_t Flash_Address);

#ifdef __cplusplus
}
#endif

#endif
