#ifndef __LAZER_H
#define __LAZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
  
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3; 

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
typedef struct 
{

  uint8_t buff1[20];
  uint8_t buff2[20];
  int16_t Distance_1, Distance_2;
} lazer;

extern lazer Lazer;

int16_t convert(const char* chuoi);
void Lazer_Init(void);
void Lazer_Return_1(void);
void Lazer_Return_2(void);
#ifdef __cplusplus
}
#endif

#endif 
