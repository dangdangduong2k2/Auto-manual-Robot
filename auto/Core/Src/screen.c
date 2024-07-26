#include "screen.h"

screen Screen;

void Screen_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Screen.data,sizeof(Screen.data));
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  
}
void Screen_Return(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Screen.data,sizeof(Screen.data));
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  
 
}
    
