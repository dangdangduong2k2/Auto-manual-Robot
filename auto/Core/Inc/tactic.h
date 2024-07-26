#ifndef __TACTIC_H
#define __TACTIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "screen.h"
  
  
  
extern int tick;
extern int set_pulses;
void running_process(void);
void home(void);
void actuators_running(void);
void stable(void);
void getbal(uint8_t num);
#ifdef __cplusplus
}
#endif

#endif