#ifndef __SCREEN_H
#define __SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "imu.h"
#include "string.h"
#include "robot_moving.h"
#include "dc_servo_driver_msb.h"
#include "read_encoder_data.h"
#include "lazer.h"
extern UART_HandleTypeDef huart6;  

extern DMA_HandleTypeDef hdma_usart6_rx;

typedef struct 
{
  struct 
  {
    uint8_t calib;
    uint8_t reset;
  } imu;
  struct 
  {
    uint8_t up;
    uint8_t down;
    uint8_t right;
    uint8_t left;
  } direction;
  struct 
  {
    uint8_t up;
    uint8_t down;
    uint8_t right;
    uint8_t left;
  } rotary;
  
  struct 
  {
    struct
    {
      uint8_t test;
      uint8_t home;
      struct
      {
        uint8_t test;
        uint8_t home;
      } state;
    } cannon;
    struct
    {
      uint8_t test;
      uint8_t home;
      struct
      {
        uint8_t test;
        uint8_t home;
      } state;
    } loader;
    struct
    {
      uint8_t test;
    } gripper;
    struct
    {
      uint8_t test;
    } gripper_pull;

    
  } actuators;
  uint8_t data[9];
} screen;

extern screen Screen;

void Screen_Return(void);
void Screen_Init(void);
void Screen_Processing(void);
void Display_16bits(uint16_t add, int16_t data);
void Display_32bits(uint16_t add, int32_t data);
#ifdef __cplusplus
}
#endif

#endif 
