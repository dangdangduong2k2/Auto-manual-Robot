#ifndef __DC_SERVO_DRIVER_UART_H
#define __DC_SERVO_DRIVER_UART_H
/****************/
#ifdef __cplusplus
 extern "C" {
#endif


#define DC_SERVO_DRIVER_UART_MAX_SPEED  1023


#include "modbus.h"

typedef struct
{
    uint16_t General;
    uint16_t KpP;
    uint16_t KiP;
    uint16_t KdP;
    uint16_t KpS;
    uint16_t KiS;
    uint16_t Vsup_High_Threshold;
    uint16_t Vsup_Low_Threshold;
    uint16_t Motor_Rated_Voltage;
    uint16_t Motor_Rated_Current;
    uint16_t Encoder_Resolution_PPR;
    uint16_t Motor_Maximun_RPM;
    uint16_t Ramp_Up;
    uint16_t Ramp_Down;
} Parameter_tdf;

typedef struct
{
    uint16_t General;
    int32_t Position;
    int16_t Speed;
    int16_t PWM;
} Demands_tdf;

typedef struct
{
    uint16_t General;
    int32_t Actual_Position;
    int16_t Actual_Speed;
    uint16_t Motor_Current;
    int16_t Motor_Voltage;
    uint16_t Vsup;
    uint16_t Fet_Temperature;
} Status_tdf;

typedef struct
{
    Status_tdf Status;
    Demands_tdf Demands;
    Parameter_tdf Parameter;
} DeviceReg_tdf;

//////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Init(uint8_t DeviceAddress,
                            Parameter_tdf *params);

//////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_ReadInfo(uint8_t DeviceAddress,
                                int32_t *Buffer);

////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand(uint8_t DeviceAddress,
                              DeviceReg_tdf *Demand);

////////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_General(uint8_t DeviceAddress,
                                      uint16_t General);

/////////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Disable(uint8_t DeviceAddress);

////////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Enable(uint8_t DeviceAddress);

//////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Speed(uint8_t DeviceAddress,
                                    int16_t MaxSpeed);

/////////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_Position(uint8_t DeviceAddress,
                                       int32_t TargetPosition,
                                       int16_t MaxSpeed);

////////////////////////////////////////////////////////
HAL_StatusTypeDef DRBD_Demand_PWM(uint8_t DeviceAddress,
                                  int16_t PWM);



/////////////////////////////////////
void DC_Servo_Driver_UART_Init(void);

///////////////////////////////////////////////////
void DC_Servo_Driver_UART_Speed_Control(uint8_t ID,
                                        int16_t Speed);

//////////////////////////////////////////////////////
void DC_Servo_Driver_UART_Position_Control(uint8_t ID,
                                           int32_t Position,
                                           int16_t Speed);

//////////////////////////////////////////////
void DC_Servo_Driver_UART_ReadInfo(uint8_t ID,
                                   int32_t *Buffer);
HAL_StatusTypeDef Modbus_ReadDI(uint8_t DeviceAddress,
                                uint16_t regAddr,
                                uint16_t* pdata,
                                uint16_t lenData);
HAL_StatusTypeDef Modbus_WriteDO(uint8_t DeviceAddress,
                                 uint16_t regAddr,
                                 uint16_t* pdata,
                                 uint16_t lenData);
extern io_driver IO_Driver;
/****************/
#ifdef __cplusplus
}
#endif
#endif
