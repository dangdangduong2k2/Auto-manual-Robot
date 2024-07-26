#ifndef __DC_SEND_H
#define __DC_SEND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdio.h"
#include <string.h>
  
extern UART_HandleTypeDef huart2;
void DC_Servo_Driver_UART_MSD_Restart_Driver(uint8_t id);

void DC_Servo_Driver_UART_MSD_Moving_Set(uint8_t id,
                                         uint32_t acceleration,
                                         uint32_t velocity,
                                         int position);
void DC_Servo_Driver_UART_MSD_Moving_Stop(uint8_t id);
void DC_Servo_Driver_UART_MSD_Moving_Start(uint8_t id);
void Dc_servo_driver_msb_init(UART_HandleTypeDef* huart);
void DC_Servo_Driver_UART_MSD_Set_0(uint8_t id);
#ifdef __cplusplus
}
#endif

#endif 
