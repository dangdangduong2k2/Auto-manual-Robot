#ifndef __READ_ENCODER_DATA_H
#define __READ_ENCODER_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

  
#include "stm32f4xx_hal.h"


typedef struct 
{
    int32_t data[5];
    int32_t overflow[5];  
} encoder;

extern encoder Encoder;

void Encoder_Init(void);  
void Encoder_read(uint8_t i);

#ifdef __cplusplus
}
#endif

#endif
