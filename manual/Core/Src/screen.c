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
    
  ///////imu//////
  if (Screen.data[4]==0x38)
  {
    if (Screen.data[8]==0x02) Screen.imu.calib=1;
    else Screen.imu.calib=0;
  }
  if (Screen.data[4]==0x39)
  {
    if (Screen.data[8]==0x02) Screen.imu.reset=1;
    else Screen.imu.reset=0;
  }
  ///////direction//////
  if (Screen.data[4]==0x30)
  {
    if (Screen.data[8]==0x02) Screen.direction.up=1;
    else Screen.direction.up=0;
  }
  if (Screen.data[4]==0x31)
  {
    if (Screen.data[8]==0x02) Screen.direction.right=1;
    else Screen.direction.right=0;
  }
  if (Screen.data[4]==0x32)
  {
    if (Screen.data[8]==0x02) Screen.direction.down=1;
    else Screen.direction.down=0;
  }
  if (Screen.data[4]==0x33)
  {
    if (Screen.data[8]==0x02) Screen.direction.left=1;
    else Screen.direction.left=0;
  }
  ///////rotary//////
  if (Screen.data[4]==0x34)
  {
    if (Screen.data[8]==0x02) Screen.rotary.up=1;
    else Screen.rotary.up=0;
  }
  if (Screen.data[4]==0x35)
  {
    if (Screen.data[8]==0x02) Screen.rotary.right=1;
    else Screen.rotary.right=0;
  }
  if (Screen.data[4]==0x36)
  {
    if (Screen.data[8]==0x02) Screen.rotary.down=1;
    else Screen.rotary.down=0;
  }
  if (Screen.data[4]==0x37)
  {
    if (Screen.data[8]==0x02) Screen.rotary.left=1;
    else Screen.rotary.left=0;
  }
  
  if (Screen.data[4]==0x24)
  {
    if (Screen.data[8]==0x02) Screen.actuators.cannon.home=1;
    else Screen.actuators.cannon.home=0;
  }
  
  if (Screen.data[4]==0x25)
  {
    if (Screen.data[8]==0x02) Screen.actuators.cannon.test=1;
    else Screen.actuators.cannon.test=0;
  }
  if (Screen.data[4]==0x27)
  {
    if (Screen.data[8]==0x02) Screen.actuators.loader.home=1;
    else Screen.actuators.loader.home=0;
  }
  if (Screen.data[4]==0x26)
  {
    if (Screen.data[8]==0x02) Screen.actuators.loader.test=1;
    else Screen.actuators.loader.test=0;
  }
  if (Screen.data[4]==0x28)
  {
    if (Screen.data[8]==0x02) Screen.actuators.gripper_pull.test++;
    if (Screen.actuators.gripper_pull.test>=2)
    {
      Screen.actuators.gripper_pull.test=0;
    }
  }
  if (Screen.data[4]==0x29)
  {
    if (Screen.data[8]==0x02) Screen.actuators.gripper.test++;
    if (Screen.actuators.gripper.test>=2)
    {
      Screen.actuators.gripper.test=0;
    }
  }
}

void Screen_Processing(void)
{
if(Robot.isHomeCompleted==1 && Robot.isRunning==0)
{  
  robotMovingDirectionSpeedSet(0);
  robotMovingRotaryAnalyticsEnable();
  robotMovingRotarySet(0);
  
  ///////imu//////
  if (Screen.imu.calib) imu_calib();
  else if (Screen.imu.reset) imu_reset();
  ///////direction//////
  else if (Screen.direction.up)
  {
    robotMovingDirectionSet(0);
    robotMovingDirectionSpeedSet(100);
  }
  else if (Screen.direction.right)
  {
    robotMovingDirectionSet(90);
    robotMovingDirectionSpeedSet(100);
  }
  else if (Screen.direction.down)
  {
    robotMovingDirectionSet(180);
    robotMovingDirectionSpeedSet(100);
  }
  else if (Screen.direction.left)
  {
    robotMovingDirectionSet(270);
    robotMovingDirectionSpeedSet(100);
  }
  ///////rotary//////

  else if (Screen.rotary.right) 
  {
    robotMovingRotaryAnalyticsDisable();
    robotMovingRotarySet(-50);
  }
  else if (Screen.rotary.left)
  {
    robotMovingRotaryAnalyticsDisable();
    robotMovingRotarySet(50);
  }
  else if (Screen.actuators.cannon.home==1&& Screen.actuators.cannon.state.home==0) 
  {
    DC_Servo_Driver_UART_MSD_Moving_Set(1,9999,10,-20); 
    Screen.actuators.cannon.state.home=2;
    
  }
  else if((IO_Driver.Serial.DI[0]&(1<<0))==(1<<0) && Screen.actuators.cannon.state.home==2)
  {
    DC_Servo_Driver_UART_MSD_Moving_Stop(1);
    DC_Servo_Driver_UART_MSD_Set_0(1);
    Screen.actuators.cannon.state.home=1;
    DC_Servo_Driver_UART_MSD_Moving_Set(1,9999,10,16); 
    Screen.actuators.cannon.state.test=1;  //1=ready to shoot
  }
  else if (Screen.actuators.cannon.test==1 && Screen.actuators.cannon.state.test==1) 
  {
    if((IO_Driver.Serial.DI[0]&(1<<1))!=(1<<1))
    {
      DC_Servo_Driver_UART_MSD_Moving_Set(1,9999,8888,2); 
      Screen.actuators.cannon.state.home=0;
      Screen.actuators.cannon.state.test=0; 
    }
  }
  else if (Screen.actuators.loader.home==1 && Screen.actuators.loader.state.home==1)
  {
    IO_Driver.Serial.DO[0] |= (1<<6);   
    Screen.actuators.loader.state.home=2;
  }
  else if (Screen.actuators.loader.state.home==2)
  {
    HAL_Delay(1000);
    DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,-15);    
    Screen.actuators.loader.state.home=0;
  }
  else if (Screen.actuators.loader.test==1 && Screen.actuators.cannon.state.home==1 && Screen.actuators.loader.state.home==0) 
  {
    DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,70); 
    IO_Driver.Serial.DO[0] &= ~(1<<6);
    Screen.actuators.loader.state.home=1;
  }
  if(Screen.actuators.gripper_pull.test==1)
  {
     IO_Driver.Serial.DO[0] &= ~(1<<5);  
  }
  if(Screen.actuators.gripper_pull.test==0)
  {
    IO_Driver.Serial.DO[0] |= (1<<5);  
  }
  if(Screen.actuators.gripper.test==1)
  {
    IO_Driver.Serial.DO[0] |= (1<<4); IO_Driver.Serial.DO[0] &= ~(1<<0); 
    IO_Driver.Serial.DO[0] |= (1<<1); IO_Driver.Serial.DO[0] &= ~(1<<2); 
  }
  if(Screen.actuators.gripper.test==0)
  {
    IO_Driver.Serial.DO[0] |= (1<<0); IO_Driver.Serial.DO[0] &= ~(1<<4); 
    IO_Driver.Serial.DO[0] |= (1<<2); IO_Driver.Serial.DO[0] &= ~(1<<1); 
  }
  
  else{}
}
    
}
void Display_16bits(uint16_t add, int16_t data)
{
  uint8_t data_screen[8];
  data_screen[0]=0x5a;
  data_screen[1]=0xa5;
  data_screen[2]=0x05;
  data_screen[3]=0x82;
  data_screen[4]=add>>8;
  data_screen[5]=add&0xff;
  data_screen[6]=data>>8;
  data_screen[7]=data&0xff;
  
  HAL_UART_Transmit(&huart6,data_screen,8,100);
}
void Display_32bits(uint16_t add, int32_t data)
{
  uint8_t data_screen[10];
  data_screen[0]=0x5a;
  data_screen[1]=0xa5;
  data_screen[2]=0x07;
  data_screen[3]=0x82;
  data_screen[4]=add>>8;
  data_screen[5]=add&0xff;
  data_screen[6]=data>>24;
  data_screen[7]=data>>16;
  data_screen[8]=data>>8;
  data_screen[9]=data&0xff;
  
  HAL_UART_Transmit(&huart6,data_screen,10,100);
}