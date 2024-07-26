#include "read_encoder_data.h"

#include <stdlib.h>
#include "stdio.h"
#include "main.h"
                                          ///////////////
                                        //////////////////
//////////////////////////////////////////////////////////
//slot1///slot2///slot3///slot4///slot5///slot6////slot7//
//tim5////tim2////tim4////tim3////tim1////uart1////uart2//
//////////////////////////////////////////////////////////

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;


encoder Encoder;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&htim5,TIM_CHANNEL_ALL);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM1)
  {
    if(TIM1->CR1 & TIM_COUNTERMODE_DOWN)
    {
      Encoder.data[0]--;
    }
    else
    {
      Encoder.data[0]++;
    }
  }
  if(htim->Instance==TIM2)
  {
    if(TIM2->CR1 & TIM_COUNTERMODE_DOWN)
    {
      Encoder.data[1]--;
    }
    else
    {
      Encoder.data[1]++;
    }
  }
  if(htim->Instance==TIM3)
  {
    if(TIM3->CR1 & TIM_COUNTERMODE_DOWN)
    {
      Encoder.data[2]--;
    }
    else
    {
      Encoder.data[2]++;
    }
  }
  if(htim->Instance==TIM4)
  {
    if(TIM4->CR1 & TIM_COUNTERMODE_DOWN)
    {
      Encoder.data[3]--;
    }
    else
    {
      Encoder.data[3]++;
    }
  }
  if(htim->Instance==TIM5)
  {
    if(TIM5->CR1 & TIM_COUNTERMODE_DOWN)
    {
      Encoder.data[4]--;
    }
    else
    {
      Encoder.data[4]++;
    }
  }
}