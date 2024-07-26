#include "tactic.h"
extern TIM_HandleTypeDef htim9;
uint8_t i;
uint8_t safe_var;
uint8_t loader_scan;
uint8_t loader_process;
uint8_t cannon_process;

int tick;
int set_pulses;
uint8_t stepper;
int32_t arr_ball_position[16]={9850,10000,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

static void robot_Delay(uint16_t time)
{
  osDelay(time);
}


void stable(void)
{
  if(Robot.isHomeCompleted==2)
  {
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && stepper==0)
    {
      set_pulses=-100000;
      stepper=1;
    }
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0)
    {
      set_pulses=0;
      tick=0;
      Robot.isHomeCompleted=1;
      stepper=0;
    }
   
  }
  
}

void home(void)
{
   if(Robot.isHomeCompleted==0)
   {
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==1 && stepper==0)
      { 
          DC_Servo_Driver_UART_MSD_Moving_Set(1,100,50,-1000);
          stepper=1;
         
      }
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==0 && stepper <2 )
      {
          DC_Servo_Driver_UART_MSD_Moving_Stop(1);
          DC_Servo_Driver_UART_MSD_Set_0(1);
          DC_Servo_Driver_UART_MSD_Moving_Set(2,100,70,1000);
          stepper=2;
          
      }
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0 && stepper==2)
      {
          DC_Servo_Driver_UART_MSD_Moving_Stop(2);
          DC_Servo_Driver_UART_MSD_Set_0(2);
          set_pulses=-100000;
          stepper=3;
      }
      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0 && stepper==3)
      {
          set_pulses=0;
          tick=0;
          DC_Servo_Driver_UART_MSD_Moving_Set(2,100,70,-63);
          DC_Servo_Driver_UART_MSD_Moving_Set(1,100,70,105);
          Robot.isHomeCompleted=1;
          stepper=0;
      }
   }
}


void getbal(uint8_t num)
{
  if(0<num && num <17 && num!=99)
  {
      set_pulses=arr_ball_position[num-1];
      if(set_pulses<10000)
      {
          robot_Delay(5000);
      }
      else
      {
          robot_Delay(8000);
      }
      DC_Servo_Driver_UART_MSD_Moving_Set(2,100,100,-95);
      DC_Servo_Driver_UART_MSD_Moving_Set(1,100,40,130);
      robot_Delay(2000);
      htim9.Instance->CCR1=110;
      robot_Delay(2000);
      DC_Servo_Driver_UART_MSD_Moving_Set(1,100,30,43);
      robot_Delay(2000);
      DC_Servo_Driver_UART_MSD_Moving_Set(2,200,40,5);  
      robot_Delay(4000);
      Robot.isHomeCompleted=2;
      Robot.ball_num=99;
  }
}


void running_process(void)
{
 
    
    
/////////////////////////////////////////////////////   
/////////////////////////////////////////////////////   
/////////////////////////////////////////////////////   
/////////////////////////////////////////////////////       
      
     
      Robot.isRunning=1;
      robot_Delay(2000);
      //phase 1:
      robotMovingDirectionSet(0);
      robotMovingDirectionSpeedSet(200);
      robotMovingRotarySet(470);
      robotMovingRotarySpeedSet(300);
      while(Robot.Moving.Rotary.Current<=465) robotMovingRotarySpeedSet(300);
      robotMovingRotarySpeedSet(50);
      robotMovingDirectionSpeedSet(500);
      while(-18500<Encoder.data[2])robotMovingRotarySpeedSet(50);
      robotMovingDirectionSpeedSet(400);
      while(-18800<Encoder.data[2])robotMovingRotarySpeedSet(50);
      robotMovingDirectionSpeedSet(300);
      while(-19000<Encoder.data[2])robotMovingRotarySpeedSet(50);
      robotMovingDirectionSpeedSet(200);
      while(1100<Lazer.Distance_1) robotMovingDirectionSet(0);
      robotMovingRotarySet(1360);
      robotMovingRotarySpeedSet(250);
      while(Robot.Moving.Rotary.Current<=1355) robotMovingRotarySpeedSet(250);
      robotMovingDirectionSpeedSet(400);
      robotMovingRotarySpeedSet(50);
      robot_Delay(1000);
      while(2400<Lazer.Distance_1) robotMovingDirectionSet(0);
      robotMovingDirectionSpeedSet(300);
      while(2200<Lazer.Distance_1) robotMovingDirectionSet(0);
      robotMovingRotarySet(460);
      robotMovingDirectionSpeedSet(200);
      robotMovingRotarySpeedSet(300);
      while(470<=Robot.Moving.Rotary.Current) robotMovingRotarySpeedSet(300);
      robotMovingRotarySpeedSet(50);
      robotMovingDirectionSpeedSet(300);
      robot_Delay(1500);
      robotMovingDirectionSpeedSet(200);
      while(2800<Lazer.Distance_1) robotMovingDirectionSpeedSet(300);
      robotMovingDirectionSpeedSet(150);
      robotMovingRotarySet(-460);
      robotMovingRotarySpeedSet(300);
      while(-450<=Robot.Moving.Rotary.Current) robotMovingRotarySpeedSet(300);
      robotMovingRotarySpeedSet(30);
      robot_Delay(4800);
      robotMovingDirectionSpeedSet(50);
      robotMovingRotarySpeedSet(100);
      robotMovingDirectionSet(260);
      while(550 < Lazer.Distance_1) robotMovingDirectionSet(260);
      while(Lazer.Distance_2 < 1840) robotMovingDirectionSet(260);
      while(1860< Lazer.Distance_2) robotMovingDirectionSet(90);
      robotMovingDirectionSet(0);
      robotMovingRotarySet(-430);
      robotMovingRotarySpeedSet(100);
      robot_Delay(1000);
      while(50<Lazer.Distance_1)robotMovingRotarySpeedSet(100);
      robotMovingDirectionSpeedSet(0);
      robot_Delay(2000);
      Robot.isHomeCompleted=2;
      stable();
      while(Robot.isHomeCompleted==2) robotMovingRotarySpeedSet(100);
      robotBallNum(1);
      //////////////////////lui
      
      while(Robot.ball_num==1) robotMovingDirectionSpeedSet(0);
      stable();
      while(Robot.isHomeCompleted==2) robotMovingDirectionSpeedSet(0);
      robotMovingDirectionSet(200);
      robotMovingDirectionSpeedSet(200);
      
      Encoder.data[2]=0;
      while(Encoder.data[2]<11000)robotMovingDirectionSpeedSet(200);
      robotMovingDirectionSet(200);
      robotMovingDirectionSpeedSet(200);
      while(Encoder.data[2]<13000)robotMovingDirectionSpeedSet(200);
      robotMovingRotarySpeedSet(100);
      robotMovingDirectionSet(80);
      while(2000<Lazer.Distance_2) robotMovingRotarySpeedSet(100);
      robotMovingDirectionSet(180);
      robotMovingDirectionSpeedSet(50);
      robot_Delay(3500);
      robotMovingRotarySpeedSet(0);
      DC_Servo_Driver_UART_MSD_Moving_Set(2,100,10,-10); 
      htim9.Instance->CCR1=50;
      robotMovingDirectionSpeedSet(0);
      
      
      
      while(1);      
  
}




