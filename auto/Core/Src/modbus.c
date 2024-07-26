#include "modbus.h"

modbus Modbus;

///////////////////////////
static void CRC_Reset(void)
{
    Modbus._xor = 0;
    Modbus.crc = 0xFFFF;
}

////////////////////////////////////////////////////
static uint16_t CRC_Calc(uint8_t *buf, uint16_t len)
{
    static const uint16_t table[256] =
    {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    while (len --)
    {
        Modbus._xor = (*buf++) ^ Modbus.crc;
        Modbus.crc >>= 8;
        Modbus.crc ^= table[Modbus._xor];
    }

    return Modbus.crc;
}

//////////////////////////////////////////////////////////////////
static HAL_StatusTypeDef CheckValidCRC(uint8_t *buf, uint16_t len)
{
    uint16_t crc_1, crc_2;
    crc_1 = ((buf[len - 1] << 8) & 0xFF00) | buf[len - 2];
    CRC_Reset();
    crc_2 = CRC_Calc(buf, len - 2);
    if (crc_1 == crc_2) return HAL_OK;
    else return HAL_ERROR;
}

///////////////////////////////////////////
void Modbus_Init(UART_HandleTypeDef* huart)
{
    Modbus.UART_Handler = huart;
}

//////////////////////////////////////////////////////////////
static HAL_StatusTypeDef Modbus_UART_Transmit(uint8_t *Buffer,
                                              uint8_t Length)
{
    HAL_StatusTypeDef Status;
      
#if (ROBOT_USING_MODBUS_RTU_UART_TX_IT)
    Status = HAL_UART_Transmit_IT(Modbus.UART_Handler,
                                  Buffer,
                                  Length);
#else
    Status = HAL_UART_Transmit(Modbus.UART_Handler,
                               Buffer,
                               Length,
                               MODBUS_UART_TX_TIMEOUT);
#endif
    
    return Status;
}

//////////////////////////////////////////////////
static HAL_StatusTypeDef Modbus_UART_Receive(void)
{
    HAL_StatusTypeDef Status;
  
    Status = HAL_UARTEx_ReceiveToIdle(Modbus.UART_Handler,
                                      Modbus.RX.Data.Buffer,
                                      MODBUS_UART_RX_BUFFER_SIZE,
                                      &Modbus.RX.Data.Length,
                                      MODBUS_UART_RX_TIMEOUT);
    return Status;
}

//Function 03 (03hex) Read Holding Registers
HAL_StatusTypeDef Modbus03(ModbusMsg_tdf *mMsg)
{
    HAL_StatusTypeDef Status = HAL_OK;
    if (mMsg->Function != 0x03)
    {
        Status = HAL_ERROR;
    }
    else
    {
        CRC_Reset();
        CRC_Calc((uint8_t*)mMsg, 6);
        mMsg->ErrorCheckLo = Modbus.crc & 0xFF;
        mMsg->ErrorCheckHi = (Modbus.crc >> 8) & 0xFF;
        
        Status = Modbus_UART_Transmit((uint8_t *)mMsg, 6);
        if (Status != HAL_OK)
        {
            return (Status);
        }
        
        Status = Modbus_UART_Transmit(&(mMsg->ErrorCheckLo), 2);
        if (Status != HAL_OK)
        {
            return (Status);
        }
        
        Status = Modbus_UART_Receive();
        if (Status != HAL_OK)
        {
            return (Status);
        }
        else
        {
            if (mMsg->SlaveAddress != Modbus.RX.Data.Buffer[0])
            {
                Status = HAL_ERROR;
            }
            else
            {
                if (CheckValidCRC(Modbus.RX.Data.Buffer, Modbus.RX.Data.Length) != HAL_OK)
                {
                    Status = HAL_ERROR;
                }
                else
                {
                    if ((mMsg->Function != Modbus.RX.Data.Buffer[1]) &&
                        (mMsg->ByteCount != Modbus.RX.Data.Buffer[2]))
                    {
                        Status = HAL_ERROR;
                    }
                    else
                    {
                        //mMsg->DataPtr = &Modbus.RX.Data.Buffer[3];
                        memcpy((uint8_t*)mMsg->DataPtr, (uint8_t*)&Modbus.RX.Data.Buffer[3], mMsg->ByteCount);
                    }
                }
            }
        }
    }
    
    return Status;
}

//Function 06 (06hex) Write Single Register
HAL_StatusTypeDef Modbus06(ModbusMsg_tdf * mMsg)
{
    HAL_StatusTypeDef Status = HAL_OK;
    if (mMsg->Function != 0x06)
    {
        Status = HAL_ERROR;
    }
    else
    {
        CRC_Reset();
        CRC_Calc((uint8_t*)mMsg, 4);
        CRC_Calc(mMsg->DataPtr, 2);
        mMsg->ErrorCheckLo = Modbus.crc & 0xFF;
        mMsg->ErrorCheckHi = (Modbus.crc >> 8) & 0xFF;
        
        Status = Modbus_UART_Transmit((uint8_t *)mMsg, 4);
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        Status = Modbus_UART_Transmit(mMsg->DataPtr, 2);
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        Status = Modbus_UART_Transmit(&(mMsg->ErrorCheckLo), 2);
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        Status = Modbus_UART_Receive();
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        if (Modbus.RX.Data.Length != 8)
        {
            Status = HAL_ERROR;
        }
        else
        {
            if ((memcmp(Modbus.RX.Data.Buffer, mMsg, 4) != 0) ||
                (memcmp(Modbus.RX.Data.Buffer + 4, mMsg->DataPtr, 2) != 0) ||
                (memcmp(Modbus.RX.Data.Buffer + 2, &(mMsg->ErrorCheckLo), 2) != 0))
            {
                Status = HAL_ERROR;
            }
        }
    }
    
    return Status;
}

//Function 16 (10hex) Write Multi Register
HAL_StatusTypeDef Modbus16(ModbusMsg_tdf * mMsg)
{
    HAL_StatusTypeDef Status = HAL_OK;
    if (mMsg->Function != 0x10)
    {
        Status = HAL_ERROR;
    }
    else
    {
        CRC_Reset();
        CRC_Calc((uint8_t*)mMsg, 7);
        CRC_Calc(mMsg->DataPtr, mMsg->ByteCount);
        mMsg->ErrorCheckLo = Modbus.crc & 0xFF;
        mMsg->ErrorCheckHi = (Modbus.crc >> 8) & 0xFF;
        
        Status = Modbus_UART_Transmit((uint8_t *)mMsg, 7);
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        Status = Modbus_UART_Transmit(mMsg->DataPtr, mMsg->ByteCount);
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        Status = Modbus_UART_Transmit(&(mMsg->ErrorCheckLo), 2);
        if (Status != HAL_OK)
        {
            return Status;
        }

        Status = Modbus_UART_Receive();
        if (Status != HAL_OK)
        {
            return Status;
        }
        
        if (Modbus.RX.Data.Length != 8)
        {
            Status = HAL_ERROR;
        }
        else
        {
            if (CheckValidCRC(Modbus.RX.Data.Buffer, Modbus.RX.Data.Length) != HAL_OK)
            {
                Status = HAL_ERROR;
            }
            else
            {
                if ((memcmp(Modbus.RX.Data.Buffer, mMsg, 6) != 0))
                {
                    Status = HAL_ERROR;
                }
            }
        }
    }
    
    Modbus.Status = Status;
    
    return Status;
}


