#ifndef __MODBUS_H
#define __MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
  
#include "string.h"

#define MODBUS_UART_RX_BUFFER_SIZE      15
#define MODBUS_UART_TX_TIMEOUT          100
#define MODBUS_UART_RX_TIMEOUT          100
extern UART_HandleTypeDef huart1;
////////////
typedef enum
{
    MB_OK = 0x00U,
    MB_NO_FRAME,
    MB_TIMEOUT,
    MB_ADDR_UNMATCH,
    MB_FRAME_ERROR,
    MB_CRC_ERROR,
    MB_RQST_OUT_OF_DATA,
    MB_RQST_FCN_ERROR,
} Modbus_Status_tdf;

//////////////
typedef struct
{
    uint8_t SlaveAddress;
    uint8_t Function;
    uint8_t StartingAddressHi;
    uint8_t StartingAddressLo;
    uint8_t QuantityOfRegistersHi;
    uint8_t QuantityOfRegistersLo;
    uint8_t ByteCount;
    uint8_t* DataPtr;
    uint8_t ErrorCheckLo;
    uint8_t ErrorCheckHi;
} ModbusMsg_tdf;

//////////////
typedef struct
{
    HAL_StatusTypeDef Status;
  
    UART_HandleTypeDef *UART_Handler;
    
    struct
    {
        struct
        {
            uint8_t Buffer[MODBUS_UART_RX_BUFFER_SIZE];
            uint16_t Length;
        } Data;
    } RX;
    
    uint8_t _xor;
    uint16_t crc;
    
} modbus;
typedef struct
{
  

    struct
    {
        // DI[0]: 00, 01, 02, 03, 04, 05, 06, 07
        // DI[1]: 10, 11, 12, 13, 14, 15, 16, 17
        // DI[2]: 20, 21, 22, 23, 24, 25, 26, 27
        uint16_t DI[3];

        // DO[0]: 00, 01, 02, 03, 04, 05, 06
        // DO[1]: 10, 11, 12, 13, 14, 15, 16
        uint16_t DO[2];
    } Serial;

    
} io_driver;

////////////////////////////////////////////
void Modbus_Init(UART_HandleTypeDef* huart);

//Function 03 (03hex) Read Holding Registers
HAL_StatusTypeDef Modbus03(ModbusMsg_tdf * mMsg);

//Function 06 (06hex) Write Single Register
HAL_StatusTypeDef Modbus06(ModbusMsg_tdf * mMsg);

//Function 16 (10hex) Write Multi Register
HAL_StatusTypeDef Modbus16(ModbusMsg_tdf * mMsg);


#ifdef __cplusplus
}
#endif

#endif