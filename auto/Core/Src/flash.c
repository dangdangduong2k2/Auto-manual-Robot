#include "flash.h"

void Flash_Delete(uint32_t pageAddress) 
{ 
    HAL_FLASH_Unlock();
    uint32_t sector = FLASH_SECTOR_5; 
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = sector;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t pageError = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    HAL_FLASH_Lock();
}
void Flash_Write(uint32_t Flash_Address,uint16_t Flash_Data)
{ 
    Flash_Delete(Flash_Address);
    HAL_StatusTypeDef status = HAL_OK;
    HAL_FLASH_Unlock();
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Flash_Address, Flash_Data);
    HAL_FLASH_Lock();
}
uint32_t Flash_Read(uint32_t Flash_Address)
{
    uint32_t Flash_Data;
    Flash_Data = *(uint32_t*)Flash_Address;
    return Flash_Data;	
}