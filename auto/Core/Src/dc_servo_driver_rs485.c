#include "dc_servo_driver_rs485.h"

ModbusMsg_tdf mMsgr1;
ModbusMsg_tdf mMsgr2;
DeviceReg_tdf DRBD;
io_driver IO_Driver;
//////////////////////////////////////////////////


HAL_StatusTypeDef Modbus_ReadDI(uint8_t DeviceAddress,
                                uint16_t regAddr,
                                uint16_t* pdata,
                                uint16_t lenData)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x03;
    mMsgr1.StartingAddressHi = (uint8_t)(regAddr >> 8);
    mMsgr1.StartingAddressLo = (uint8_t)(regAddr);
    mMsgr1.QuantityOfRegistersHi = (uint8_t)(lenData >> 8);
    mMsgr1.QuantityOfRegistersLo = (uint8_t)(lenData);
    mMsgr1.ByteCount = 2 * lenData;
    Status = Modbus03(&mMsgr1);
    if (Status == HAL_OK)
    {
        memcpy(pdata, mMsgr1.DataPtr, mMsgr1.ByteCount);
    }
    
    return Status;
}

///////////////////////////////////////////////////////
HAL_StatusTypeDef Modbus_WriteDO(uint8_t DeviceAddress,
                                 uint16_t regAddr,
                                 uint16_t* pdata,
                                 uint16_t lenData)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    mMsgr2.SlaveAddress = DeviceAddress;
    mMsgr2.Function = 0x10;
    mMsgr2.StartingAddressHi = (uint8_t)(regAddr >> 8);
    mMsgr2.StartingAddressLo = (uint8_t)(regAddr);
    mMsgr2.QuantityOfRegistersHi = (uint8_t)(lenData >> 8);
    mMsgr2.QuantityOfRegistersLo = (uint8_t)(lenData);
    mMsgr2.ByteCount = 2 * lenData;
    mMsgr2.DataPtr = (uint8_t*)pdata;
    Status = Modbus16(&mMsgr2);
    
    return Status;
}

HAL_StatusTypeDef DRBD_Init(uint8_t DeviceAddress,
                            Parameter_tdf * params)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x10;
    mMsgr1.StartingAddressHi = 0x02;
    mMsgr1.StartingAddressLo = 0x00;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x0E;
    mMsgr1.ByteCount = 0x1C;
    mMsgr1.DataPtr = (uint8_t *)(&(DRBD.Parameter));
    Status = Modbus16(&mMsgr1);
    
    return Status;
}

//////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_ReadInfo(uint8_t DeviceAddress,
                                int32_t *Buffer)
{
    HAL_StatusTypeDef Status = HAL_OK;
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x03;
    mMsgr1.StartingAddressHi = 0x00;
    mMsgr1.StartingAddressLo = 0x00;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x05;
    mMsgr1.ByteCount = 0x0A;
    mMsgr1.DataPtr = (uint8_t *)(&(DRBD.Status));
    Status = Modbus03(&mMsgr1);
    if (Status == HAL_OK)
    {
        Buffer[0] = DRBD.Status.General;
        Buffer[1] = DRBD.Status.Actual_Position;
        Buffer[2] = DRBD.Status.Actual_Speed;
        Buffer[3] = DRBD.Status.Motor_Current;
        Buffer[4] = DRBD.Status.Motor_Voltage;
        Buffer[5] = DRBD.Status.Vsup;
        Buffer[6] = DRBD.Status.Fet_Temperature;
    }
    
    return Status;
}

////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand(uint8_t DeviceAddress,
                              DeviceReg_tdf *Demand)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x10;
    mMsgr1.StartingAddressHi = 0x01;
    mMsgr1.StartingAddressLo = 0x00;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x01;
    mMsgr1.ByteCount = 0x02;
    mMsgr1.DataPtr = (uint8_t *)Demand;
    Status = Modbus16(&mMsgr1);
    
    return Status;
}

///////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_General(uint8_t DeviceAddress,
                                      uint16_t General)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    DRBD.Demands.General = General;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x10;
    mMsgr1.StartingAddressHi = 0x01;
    mMsgr1.StartingAddressLo = 0x00;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x01;
    mMsgr1.ByteCount = 0x02;
    mMsgr1.DataPtr = (uint8_t *)(&(DRBD.Demands.General));
    Status = Modbus16(&mMsgr1);
    
    return Status;
}

////////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Disable(uint8_t DeviceAddress)
{
    HAL_StatusTypeDef Status = HAL_OK;
    Status = DRBD_Demand_General(DeviceAddress, 0x0000);
    return Status;
}

///////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Enable(uint8_t DeviceAddress)
{
    HAL_StatusTypeDef Status = HAL_OK;
    Status = DRBD_Demand_General(DeviceAddress, 0x0001);
    return Status;
}

//////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Speed(uint8_t DeviceAddress,
                                    int16_t MaxSpeed)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    DRBD.Demands.Speed = MaxSpeed;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x10;
    mMsgr1.StartingAddressHi = 0x01;
    mMsgr1.StartingAddressLo = 0x01;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x03;
    mMsgr1.ByteCount = 0x06;
    mMsgr1.DataPtr = (uint8_t *)(&(DRBD.Demands.Speed));
    Status = Modbus16(&mMsgr1);
    
    return Status;
}

/////////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Position(uint8_t DeviceAddress,
                                       int32_t TargetPosition,
                                       int16_t MaxSpeed)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    DRBD.Demands.Position = TargetPosition;
    DRBD.Demands.Speed = MaxSpeed;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x10;
    mMsgr1.StartingAddressHi = 0x01;
    mMsgr1.StartingAddressLo = 0x01;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x03;
    mMsgr1.ByteCount = 0x06;
    mMsgr1.DataPtr = (uint8_t *)(&(DRBD.Demands.Position));
    Status = Modbus16(&mMsgr1);
    
    return Status;
}

///////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_PWM(uint8_t DeviceAddress,
                                  int16_t PWM)
{
    HAL_StatusTypeDef Status = HAL_OK;
    
    DRBD.Demands.PWM = PWM;
    
    mMsgr1.SlaveAddress = DeviceAddress;
    mMsgr1.Function = 0x10;
    mMsgr1.StartingAddressHi = 0x01;
    mMsgr1.StartingAddressLo = 0x04;
    mMsgr1.QuantityOfRegistersHi = 0x00;
    mMsgr1.QuantityOfRegistersLo = 0x01;
    mMsgr1.ByteCount = 0x02;
    mMsgr1.DataPtr = (uint8_t *)(&(DRBD.Demands.PWM));
    Status = Modbus16(&mMsgr1);
    
    return Status;
}

///////////////////////////////////////////
//static void DRBD_Demand_PWM_Mode_Init(void)
//{
//    DRBD.Parameter.General = 0x01; // PWM mode
//    
//    HAL_StatusTypeDef Status = HAL_TIMEOUT;
//    uint32_t pretick = HAL_GetTick();
//    while (Status != HAL_OK)
//    {
//        //Status = DRBD_Init(0xFF, &(DRBD.Parameter));
//        
//        Status = DRBD_Demand_Enable(0xFF);
//        
//        HAL_Delay(1);
//        if (2000 < HAL_GetTick() - pretick)
//        {
//            return;
//        }
//    }
//}

/////////////////////////////////////////////////
static void DRBD_Demand_PID_Speed_Mode_Init(void)
{
    DRBD.Parameter.General = 0x09;
    
    DRBD.Parameter.Vsup_High_Threshold = 27;
    DRBD.Parameter.Vsup_Low_Threshold = 15;
    DRBD.Parameter.Motor_Rated_Voltage = 24;
    DRBD.Parameter.Motor_Rated_Current = 6;
    
#if (1) // Old Motor Info
    DRBD.Parameter.KpS = 150;
    DRBD.Parameter.KiS = 150;
    DRBD.Parameter.Encoder_Resolution_PPR = 400;
    DRBD.Parameter.Motor_Maximun_RPM = 7000;
    DRBD.Parameter.Ramp_Up = 64000;
    
    DRBD.Parameter.Ramp_Down = 64000;
#else // New Motor Info
    DRBD.Parameter.KpS = 50;
    DRBD.Parameter.KiS = 50;
    DRBD.Parameter.Encoder_Resolution_PPR = 2000;
    DRBD.Parameter.Motor_Maximun_RPM = 3000;
    DRBD.Parameter.Ramp_Up = 64000;
    DRBD.Parameter.Ramp_Down = 64000;
#endif
    
    HAL_StatusTypeDef Status = HAL_TIMEOUT;
    uint32_t pretick = HAL_GetTick();
    while (Status != HAL_OK)
    {
        Status = DRBD_Demand_Enable(1);
        Status = DRBD_Demand_Enable(5);
        Status = DRBD_Demand_Enable(6);

        HAL_Delay(1);
        if (2000 < HAL_GetTick() - pretick)
        {
            return;
        }
    }
}

////////////////////////////////////////////////////
static void DRBD_Demand_PID_Position_Mode_Init(void)
{
    DRBD.Parameter.General = 0x11;
    
    DRBD.Parameter.KpP = 50;
    DRBD.Parameter.KiP = 50;
    DRBD.Parameter.KdP = 50;
    
    DRBD.Parameter.KpS = 50;
    DRBD.Parameter.KiS = 50;
    
    DRBD.Parameter.Vsup_High_Threshold = 27;
    DRBD.Parameter.Vsup_Low_Threshold = 15;
    DRBD.Parameter.Motor_Rated_Voltage = 24;
    DRBD.Parameter.Motor_Rated_Current = 3;
    
    DRBD.Parameter.Encoder_Resolution_PPR = 400;
    DRBD.Parameter.Motor_Maximun_RPM = 3000;
    DRBD.Parameter.Ramp_Up = 64000;
    DRBD.Parameter.Ramp_Down = 64000;
    
    HAL_StatusTypeDef Status = HAL_TIMEOUT;
    uint32_t pretick = HAL_GetTick();
    while (Status != HAL_OK)
    {
//                Status = DRBD_Init(1, &(DRBD.Parameter));
//                Status = DRBD_Init(5, &(DRBD.Parameter));
//                Status = DRBD_Init(6, &(DRBD.Parameter));
//        
//        Status = DRBD_Demand_Enable(1);
//        Status = DRBD_Demand_Enable(5);
//        Status = DRBD_Demand_Enable(6);

        
      
      HAL_Delay(1);
        if (2000 < HAL_GetTick() - pretick)
        {
            return;
        }
    }
}

////////////////////////////////////
void DC_Servo_Driver_UART_Init(void)
{
    DRBD_Demand_PID_Speed_Mode_Init();
    DRBD_Demand_PID_Position_Mode_Init();
    //DRBD_Demand_PWM_Mode_Init();
}

//////////////////////////////////////////////
void DC_Servo_Driver_UART_ReadInfo(uint8_t ID,
                                   int32_t *Buffer)
{
    DRBD_ReadInfo(ID, Buffer);
}

///////////////////////////////////////////////////
void DC_Servo_Driver_UART_Speed_Control(uint8_t ID,
                                        int16_t Speed)
{
    if (Speed < -DC_SERVO_DRIVER_UART_MAX_SPEED)
    {
        Speed = -DC_SERVO_DRIVER_UART_MAX_SPEED;
    }
    if (DC_SERVO_DRIVER_UART_MAX_SPEED < Speed)
    {
        Speed = DC_SERVO_DRIVER_UART_MAX_SPEED;
    }
    
    DRBD_Demand_Position(ID,3, Speed);
    
}

//////////////////////////////////////////////////////
void DC_Servo_Driver_UART_Position_Control(uint8_t ID,
                                           int32_t Position,
                                           int16_t Speed)
{
    if (Speed < -DC_SERVO_DRIVER_UART_MAX_SPEED)
    {
        Speed = -DC_SERVO_DRIVER_UART_MAX_SPEED;
    }
    if (DC_SERVO_DRIVER_UART_MAX_SPEED < Speed)
    {
        Speed = DC_SERVO_DRIVER_UART_MAX_SPEED;
    }
  
    DRBD_Demand_Position(ID, Position, Speed);
}

