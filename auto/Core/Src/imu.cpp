#include "imu.h"
#include "flash.h"
#include "main.h"
#include "MPU6050.h"
#include "stdio.h"

#define true 1
#define false 0	

#ifndef NULL
#define NULL 0
#endif

#define OK 6U
class IMU {
public:
    // Constructor
    IMU() : tick(0), gz(0), read_offset(0), sample_count(0), sample_count_max(0), offset_value(0), popular_gz_value(0), integral(0.0f) {}
    
    // Member variables
    uint16_t tick;
    int16_t gz;
    int16_t read_offset;
    //calib offset 
    uint16_t sample_count;
    uint16_t sample_count_max;
    int16_t offset_value;
    int32_t popular_gz_value;
    //calib offset 
    float integral;
};
MPU6050 mpu;
I2Cdev i2c;
IMU imu;
#define page_64_startAdress 0x08030000

void imu_init(void)
{
  mpu.reset();
  HAL_Delay(30);
  mpu.initialize();
  HAL_Delay(30);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  mpu.setStandbyXAccelEnabled(false);
  mpu.setStandbyYAccelEnabled(false);
  mpu.setStandbyXGyroEnabled(false);
  mpu.setStandbyYGyroEnabled(false);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  imu.read_offset =   (int)Flash_Read(page_64_startAdress);
  imu.sample_count_max=500;
}
void imu_calib(void)
{ 
  mpu.setZGyroOffset(0);
  imu.read_offset = 0;
  while(1)
  {
    HAL_Delay(30);
    imu.popular_gz_value += imu.gz;
    imu.sample_count++;
    
    if(imu.sample_count > imu.sample_count_max)
    {
      imu.offset_value = imu.popular_gz_value/imu.sample_count_max;
      imu.sample_count = 0;
      imu.popular_gz_value =0;
      imu.offset_value = (int16_t)(imu.offset_value);
      Flash_Write(page_64_startAdress,(uint64_t)imu.offset_value); 
      imu.read_offset =   (int)Flash_Read(page_64_startAdress);
      imu.integral=0;
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
      break;
    }
  }
}
void imu_reset(void)
{
  imu.integral=0;
}
int16_t imu_return(void)
{
  imu.gz=mpu.getRotationZ() - imu.read_offset;
  if(abs(imu.gz) > 1) imu.integral += (imu.gz/gyro_ratio)*dt;
  return (int16_t)imu.integral;
}
        
