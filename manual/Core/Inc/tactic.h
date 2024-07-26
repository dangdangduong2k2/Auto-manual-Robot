#ifndef __TACTIC_H
#define __TACTIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "screen.h"

extern uint8_t skip;
extern uint8_t stop;
 
  
  
void E_Stop(void);
void running_process(void);
void home(void);
void actuators_running(void);


#ifdef __cplusplus
}
#endif

#endif