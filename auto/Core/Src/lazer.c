#include "lazer.h"
lazer Lazer;

int16_t convert(const char* chuoi) {
    int so = 0;
    int i = 0;
    
    if(chuoi[0]==0x44 && (chuoi[9] == 0x0A || chuoi[10]==0x0A))
    {
      if(chuoi[10]==0x44)
      {
        memset((void*)chuoi, '\0', sizeof(chuoi));
        return -1;
      }
      else
      {
        while (chuoi[i] != '\0') 
        {
          if (chuoi[i] >= '0' && chuoi[i] <= '9') {
              so = so * 10 + (chuoi[i] - '0');
          }
          i++;
        }
        return so;
      }
      
    }
    
    else
    {
      return -1;
    }
}
void Lazer_Init(void)
{
  uint8_t transbuff[15] = "iFACM: 2\n";
  HAL_UART_Transmit(&huart5,transbuff,strlen((const char *)transbuff),100);
  HAL_UART_Transmit(&huart3,transbuff,strlen((const char *)transbuff),100);
    
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Lazer.buff1,sizeof(Lazer.buff1));
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
     
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Lazer.buff2,sizeof(Lazer.buff2)); 
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void Lazer_Return_1(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Lazer.buff1,sizeof(Lazer.buff1));
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
  Lazer.Distance_1=convert((const char*)Lazer.buff1);
  
}

void Lazer_Return_2(void)
{ 
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Lazer.buff2,sizeof(Lazer.buff2)); 
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  Lazer.Distance_2=convert((const char*)Lazer.buff2);
  
}
