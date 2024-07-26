#include "tactic.h"

uint8_t i;
uint8_t safe_var;
uint8_t loader_scan;
uint8_t loader_process;
uint8_t cannon_process;
uint8_t skip;
uint8_t stop;


static void robot_Delay(uint16_t time)
{
  osDelay(time);
}

void home(void)
{
  if(Robot.isHomeCompleted==0)
  {
    ////////home cannon/////////////////////////
    if (Screen.actuators.cannon.state.home==0 && safe_var==1) 
    {
      DC_Servo_Driver_UART_MSD_Moving_Set(1,9999,10,-20); 
      
    }
    if(IO_Driver.Serial.DI[0] & (1<<0)==(1<<0) && Screen.actuators.cannon.state.home==0 && safe_var==1)
    {
      DC_Servo_Driver_UART_MSD_Moving_Stop(1);
      DC_Servo_Driver_UART_MSD_Set_0(1);
      Screen.actuators.cannon.state.home=1;
      DC_Servo_Driver_UART_MSD_Moving_Set(1,9999,10,16); 
      Screen.actuators.cannon.state.test=1;  //1=ready to shoot
      
    }
    ////////home loader/////////////////////////
    if (Screen.actuators.loader.state.home==0 && Screen.actuators.cannon.state.home==0 && safe_var==0) 
    {
      DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,-100); 
      if(((IO_Driver.Serial.DI[0] & (1<<1))!=(1<<1)))
      {
        DC_Servo_Driver_UART_MSD_Moving_Stop(2);
        DC_Servo_Driver_UART_MSD_Set_0(2);
        safe_var=1;
      }
    }
    if (Screen.actuators.loader.state.home==0 && Screen.actuators.cannon.state.home==1 && ((IO_Driver.Serial.DI[0]>>1)&0x0001==1)  && i==0) 
    {
      DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,-100);      
    }
    if(((IO_Driver.Serial.DI[0] & (1<<1))!=(1<<1)) && Screen.actuators.loader.state.home==0 &&Screen.actuators.cannon.state.home==1 )
    {    
      if(i==0)
      {
          DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,100);      
          i=1;
      }
      
    }
    if(i==1)
    {
      if((IO_Driver.Serial.DI[0]>>1)&0x0001==1)
      {
        DC_Servo_Driver_UART_MSD_Moving_Stop(2);
        DC_Servo_Driver_UART_MSD_Set_0(2);       
        DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,-100);      
        i=2;
      }
    }  
    if(i==2)
    {
      if(((IO_Driver.Serial.DI[0] & (1<<1))!=(1<<1)))
      {
        DC_Servo_Driver_UART_MSD_Moving_Stop(2);
        DC_Servo_Driver_UART_MSD_Set_0(2);      
        DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,30,70); 
        Screen.actuators.loader.state.home=1;
        Screen.actuators.loader.state.test=1;  
        i=0;
      }
    }  
    
    if(Screen.actuators.loader.state.home==1 && Screen.actuators.cannon.state.home==1)
    {
      IO_Driver.Serial.DO[0] |= (1<<6);
      IO_Driver.Serial.DO[0] |= (1<<5);
      IO_Driver.Serial.DO[0] |= (1<<4); IO_Driver.Serial.DO[0] &= ~(1<<0); 
      IO_Driver.Serial.DO[0] |= (1<<2); IO_Driver.Serial.DO[0] &= ~(1<<1); 
      Robot.isHomeCompleted=1;
    }
  }
  
}
void E_Stop(void)
{
    robotMovingDirectionSpeedSet(0);
    robotMovingRotarySpeedSet(0);
    robotMovingRotarySet(0);
    DC_Servo_Driver_UART_MSD_Moving_Stop(1);
    DC_Servo_Driver_UART_MSD_Moving_Stop(2);    
}

void running_process(void)
{
  if(Robot.isHomeCompleted==1)
  {
    
    
/////////////////////////////////////////////////////   
/////////////////////////////////////////////////////   
/////////////////////////////////////////////////////   
/////////////////////////////////////////////////////       
      
      while(150<Lazer.Distance_1 | Lazer.Distance_1 == -1)robotMovingDirectionSet(270);
      Robot.isRunning=1;
      
      //phase 1:
      robotMovingDirectionSet(270);
      robotMovingDirectionSpeedSet(200);
      robotMovingRotarySet(455);
      robotMovingRotarySpeedSet(150);
      while(Robot.Moving.Rotary.Current<=440 && skip==0) robotMovingDirectionSpeedSet(200);
      while(3350<Lazer.Distance_2 && skip==0) robotMovingDirectionSpeedSet(200);
      robotMovingDirectionSpeedSet(100);
      IO_Driver.Serial.DO[0] &= ~(1<<5);
      
      while(2990<Lazer.Distance_2 && skip==0) robotMovingDirectionSpeedSet(120);
      robotMovingRotarySpeedSet(0);
      robotMovingDirectionSet(180);
      robot_Delay(1000);
      
      
      robotMovingDirectionSpeedSet(0);
      IO_Driver.Serial.DO[0] |= (1<<0); IO_Driver.Serial.DO[0] &= ~(1<<4); 
      IO_Driver.Serial.DO[0] |= (1<<1); IO_Driver.Serial.DO[0] &= ~(1<<2); 
      robot_Delay(500); 
      IO_Driver.Serial.DO[0] |= (1<<5);
      robot_Delay(2000); 
      robotMovingDirectionSet(10);
      robotMovingDirectionSpeedSet(200);
      while(Lazer.Distance_2<3040 && skip==0) robotMovingRotarySpeedSet(150);
      robotMovingDirectionSet(0);         
      while(2220<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(200); 
      robotMovingDirectionSpeedSet(100); 
      while(1370<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(100); 
      robotMovingDirectionSpeedSet(0); 
      IO_Driver.Serial.DO[0] &= ~(1<<5);
      robot_Delay(1600);
      IO_Driver.Serial.DO[0] |= (1<<2); IO_Driver.Serial.DO[0] &= ~(1<<1); 
      robot_Delay(500);
      IO_Driver.Serial.DO[0] |= (1<<5);
      robot_Delay(1000);
      robotMovingDirectionSet(0);
      while(1120<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(100);
      robotMovingDirectionSet(300);
      while(2840<Lazer.Distance_2 && skip==0) robotMovingDirectionSpeedSet(80);
      robotMovingDirectionSet(0);
      while(850<Lazer.Distance_1 && skip==0) robotMovingDirectionSet(0);
      
      robotMovingDirectionSpeedSet(0);
      IO_Driver.Serial.DO[0] &= ~(1<<5);
      robot_Delay(1000);
      IO_Driver.Serial.DO[0] |= (1<<4); IO_Driver.Serial.DO[0] &= ~(1<<0);   
      robot_Delay(1000);     
      //phase 2:
      
      
      robotMovingDirectionSet(0);
      robotMovingDirectionSpeedSet(100);
      while(520<Lazer.Distance_1 && skip==0) robotMovingDirectionSet(0);
      robotMovingDirectionSet(250);
      robotMovingDirectionSpeedSet(200);
      while(2200<Lazer.Distance_2 && skip==0) robotMovingDirectionSet(250);
      robotMovingDirectionSet(175);
      robotMovingDirectionSpeedSet(350);
      while(Lazer.Distance_1<1620 && skip==0) robotMovingDirectionSet(175);
      robotMovingDirectionSpeedSet(200);
      while(Lazer.Distance_1<2720 && skip==0) robotMovingDirectionSet(175);
      robotMovingDirectionSpeedSet(100);
      while(Lazer.Distance_1<3220 && skip==0) robotMovingDirectionSet(175);
      robotMovingDirectionSpeedSet(80);
      while(1970<Lazer.Distance_2 && skip==0) robotMovingDirectionSet(270);
      robotMovingDirectionSet(180);
      robotMovingRotarySpeedSet(0);
      robotMovingDirectionSpeedSet(120);
      robot_Delay(1000);
      
      
      robotMovingDirectionSpeedSet(0);
      IO_Driver.Serial.DO[0] |= (1<<0); IO_Driver.Serial.DO[0] &= ~(1<<4); 
      IO_Driver.Serial.DO[0] |= (1<<1); IO_Driver.Serial.DO[0] &= ~(1<<2); 
      robot_Delay(500);
      IO_Driver.Serial.DO[0] |= (1<<5);
      robot_Delay(2000);  
      
      robotMovingDirectionSet(10);
      robotMovingDirectionSpeedSet(200);
      
      while(Lazer.Distance_2<2040 && skip==0) robotMovingRotarySpeedSet(150);
      robotMovingDirectionSet(0);         
      while(2220<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(200); 
      robotMovingDirectionSpeedSet(100); 
      while(1370<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(100); 
      robotMovingDirectionSpeedSet(0); 
      IO_Driver.Serial.DO[0] &= ~(1<<5);
      robot_Delay(1600);
      IO_Driver.Serial.DO[0] |= (1<<2); IO_Driver.Serial.DO[0] &= ~(1<<1); 
      robot_Delay(500);
      IO_Driver.Serial.DO[0] |= (1<<5);
      robot_Delay(1000);
      robotMovingDirectionSet(0);
      while(1120<Lazer.Distance_1  && skip==0) robotMovingDirectionSpeedSet(100);
      robotMovingDirectionSet(300);
      while(1830<Lazer.Distance_2 && skip==0) robotMovingDirectionSpeedSet(80);
      robotMovingDirectionSet(0);
      while(850<Lazer.Distance_1 && skip==0) robotMovingDirectionSet(0);
      
      robotMovingDirectionSpeedSet(0);
      IO_Driver.Serial.DO[0] &= ~(1<<5);
      robot_Delay(1000);
      IO_Driver.Serial.DO[0] |= (1<<4); IO_Driver.Serial.DO[0] &= ~(1<<0);   
      robot_Delay(1000); 
      robotMovingDirectionSet(0);
      while(670<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(150);
      robotMovingDirectionSet(90);
      robotMovingDirectionSpeedSet(200);
      while(Lazer.Distance_2<2900 && skip==0) robotMovingDirectionSpeedSet(300);     
      robotMovingRotarySet(-455);
      while(-440<=Robot.Moving.Rotary.Current && skip==0) robotMovingDirectionSpeedSet(300);     
      while(Lazer.Distance_2<1000 && skip==0) robotMovingDirectionSpeedSet(300);     
      robotMovingRotarySet(455);
      while(Robot.Moving.Rotary.Current<=440 && skip==0) robotMovingDirectionSpeedSet(300);     
      while(Lazer.Distance_2<4800 && skip==0) robotMovingDirectionSpeedSet(300);     
      robotMovingDirectionSet(0);
      robotMovingDirectionSpeedSet(400);
      //tang 2:
      while(Robot.Moving.Rotary.Current<=440 && skip==0) robotMovingDirectionSpeedSet(300);     
      robot_Delay(1600);
      
      //////////////////////////////////
      robotMovingDirectionSet(0);
      robotMovingRotarySet(445);
      robotMovingDirectionSpeedSet(300);     
      //////////////////////////////////
      
      while(2620<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(300);     
      robotMovingDirectionSpeedSet(200);
      robotMovingDirectionSet(265);
      while(3400<Lazer.Distance_2 && skip==0) robotMovingDirectionSpeedSet(200);
      robotMovingDirectionSpeedSet(100);
      while(3250<Lazer.Distance_2 && skip==0) robotMovingDirectionSpeedSet(100);
      robotMovingDirectionSet(0);
      robotMovingDirectionSpeedSet(0);
      DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,40,-10);
      IO_Driver.Serial.DO[0] |= (1<<6);
      robot_Delay(1200);
      robotMovingDirectionSpeedSet(50);
      while((IO_Driver.Serial.DI[0]>>3)&0x0001==1 && skip==0) robotMovingDirectionSpeedSet(50);
      robotMovingDirectionSpeedSet(0);
      robot_Delay(1000);
      IO_Driver.Serial.DO[0] &= ~(1<<6);
      DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,40,70);
      robotMovingDirectionSet(90);
      robotMovingDirectionSpeedSet(200);
      while(Lazer.Distance_2<4050 && skip==0) robotMovingDirectionSpeedSet(200);
      robotMovingDirectionSet(0);
      while(400<Lazer.Distance_1 && skip==0) robotMovingDirectionSpeedSet(200);
      robotMovingDirectionSpeedSet(0);
      IO_Driver.Serial.DO[0] |= (1<<6);
      DC_Servo_Driver_UART_MSD_Moving_Set(2,9999,40,20);
      robot_Delay(2500);
      DC_Servo_Driver_UART_MSD_Moving_Set(1,9999,9999,2); 
      robot_Delay(2000);
      
      while(1);      
  }
}




