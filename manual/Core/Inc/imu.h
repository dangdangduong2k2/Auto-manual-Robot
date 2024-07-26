#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
  
#define true 1
#define false 0	

#ifndef NULL
#define NULL 0
#endif

#define OK 6U  
#define dt 0.001978 //(s)
#define gyro_ratio 13
          //131 
           //(with full scale range 250 degre/s)
          // 00    0    250    131
          // 08    1    500    65.5
          // 10    2    1000    32.8
          // 18    3    2000    16.4




void imu_init(void);
void imu_calib(void);
void imu_reset(void);
int16_t imu_return(void);

#ifdef __cplusplus
}
#endif

#endif 
