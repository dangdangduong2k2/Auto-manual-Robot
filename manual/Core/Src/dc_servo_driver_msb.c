#include "dc_servo_driver_msb.h"

UART_HandleTypeDef *UART_Handler;

void Dc_servo_driver_msb_init(UART_HandleTypeDef* huart)
{
  UART_Handler=huart;
}

static void DC_Servo_Driver_UART_MSD_Send_Command(const uint8_t *pData,
                                                  uint16_t Size)
{
    HAL_UART_Transmit(UART_Handler,
                      pData,
                      Size,
                      1000);
}

//////////////////////////////////////////////////
void DC_Servo_Driver_UART_MSD_Restart_Driver(uint8_t id)
{
    uint8_t buffer[10];
  
    if(id==1)
    {
      memcpy(buffer, "N1 O R101\n", 10);
    }
    
    else if(id==2)
    {
      memcpy(buffer, "N2 O R101\n", 10);
    }
    
    DC_Servo_Driver_UART_MSD_Send_Command(buffer,
                                          10);
}

////////////////////////////////////////////////////////////////
void DC_Servo_Driver_UART_MSD_Moving_Set(uint8_t id,
                                         uint32_t acceleration,
                                         uint32_t velocity,
                                         int32_t position)
{
    uint8_t buffer[30];
    uint8_t length = 0;
    
    sprintf((char *)buffer,
            "N%u p%d v%u a%u\n",
            id,
            position,
            velocity,
            acceleration);
    length = strlen((const char *)buffer);
    
    DC_Servo_Driver_UART_MSD_Send_Command(buffer,
                                          length);
}

void DC_Servo_Driver_UART_MSD_Moving_Stop(uint8_t id)
{
    uint8_t buffer[7];
  
    if(id==1)
    {
      memcpy(buffer, "N1 O L\n", 7);
    }
    else if(id==2)
    {
      memcpy(buffer, "N2 O L\n", 7);
    }
    DC_Servo_Driver_UART_MSD_Send_Command(buffer,
                                          7);
}
void DC_Servo_Driver_UART_MSD_Moving_Start(uint8_t id)
{
    uint8_t buffer[7];
  
    if(id==1)
    {
      memcpy(buffer, "N1 O U\n", 7);
    } 
    else if(id==2)
    {
      memcpy(buffer, "N2 O U\n", 7);
    }
    
    
    DC_Servo_Driver_UART_MSD_Send_Command(buffer,
                                          7);
}
void DC_Servo_Driver_UART_MSD_Set_0(uint8_t id)
{
    uint8_t buffer[7];
  
    if(id==1)
    {
      memcpy(buffer, "N1 O r\n", 7);
    }
    else if(id==2)
    {
      memcpy(buffer, "N2 O r\n", 7);
    }
    DC_Servo_Driver_UART_MSD_Send_Command(buffer,
                                          7);
}
