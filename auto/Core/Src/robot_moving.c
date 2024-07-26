#include "robot_moving.h"

robot Robot;


///////////////////////////////////////////
void robotHome(void)
{
  Robot.isHomeCompleted=0;
}
void robotStable(void)
{
  Robot.isHomeCompleted=2;
}
void robotBallNum(uint8_t num)
{
  Robot.ball_num=num;
}
void robotMovingRotaryRadiusSet(double A,
                                double B)
{
    Robot.Moving.Mecanum.Radius.A = A;
    Robot.Moving.Mecanum.Radius.B = B;
    Robot.Moving.Mecanum.Radius.Ratio = Robot.Moving.Mecanum.Radius.A / Robot.Moving.Mecanum.Radius.B;
}

///////////////////////////////////////////
void robotMovingSpeedRangeSet(uint16_t Min,
                              uint16_t Max)
{
    Robot.Moving.Direction.Speed.Min = Min;
    Robot.Moving.Direction.Speed.Max = Max;
}

/////////////////////////////////////////////
void robotMovingDirectionRangeSet(int16_t Min,
                                  int16_t Max)
{
    Robot.Moving.Direction.Min = Min;
    Robot.Moving.Direction.Max = Max;
}

//////////////////////////////////////////
void robotMovingRotaryRangeSet(double Min,
                               double Max)
{
    Robot.Moving.Rotary.Min = Min;
    Robot.Moving.Rotary.Max = Max;
}

///////////////////////////////////////


void robotMovingRotaryPIDSet(double Kp,
                             double Ki,
                             double Kd,
                             double Min,
                             double Max)
{
    PID_Init(&Robot.Moving.Rotary.Speed.PID,
             DT_ANALYTICS,
             Kp,
             Ki,
             Kd,
             Min,
             Max);
}
void robotMovingRotarySpeedSet(double Max)
{
    Robot.Moving.Rotary.Speed.PID.Output.Max=Max;
}
///////////////////////////////////////
void robotMovingDirectionSpeedSet(int16_t Speed)
{
    Robot.Moving.Direction.Speed.Run = Speed;
}

///////////////////////////////////////////////
void robotMovingDirectionSet(int16_t Direction)
{
    Robot.Moving.Direction.Run = Direction;
}

////////////////////////////////////////
void robotMovingRotarySet(int16_t Rotary)
{
    Robot.Moving.Rotary.Run = Rotary;
}

///////////////////////////////////////////////
static void robotMovingDirectionAnalytics(void)
{
    // Thiet lap cac gia tri dau vao tam thoi
    double tempSpeedInput = Robot.Moving.Direction.Speed.Run;
    double tempDirectionInput = Robot.Moving.Direction.Run;
    
    // Thiet lap cac gia tri tinh toan theo huong di chuyen
    double sinDirectionInput = sin(tempDirectionInput * DEG_TO_RAD);
    double cosDirectionInput = cos(tempDirectionInput * DEG_TO_RAD);
    
    // Thiet lap cac gia tri tam thoi de tinh toan toc do tong hop theo huong di chuyen
    double tempSpeedDirectionA = 0;
    double tempSpeedDirectionB1 = 0;
    double tempSpeedDirectionB2 = 0;
    
    tempSpeedDirectionA = sinDirectionInput + cosDirectionInput;
    tempSpeedDirectionB1 = sinDirectionInput - cosDirectionInput;
    tempSpeedDirectionB2 = sinDirectionInput - cosDirectionInput;
    
    tempSpeedDirectionA *= tempSpeedInput;
    tempSpeedDirectionB1 *= tempSpeedInput;
    tempSpeedDirectionB2 *= tempSpeedInput;

    Robot.Moving.Mecanum.Speed.Direction.A = tempSpeedDirectionA;
    Robot.Moving.Mecanum.Speed.Direction.B1 = tempSpeedDirectionB1;
    Robot.Moving.Mecanum.Speed.Direction.B2 = tempSpeedDirectionB2;
}

///////////////////////////////////////////
void robotMovingRotaryAnalyticsEnable(void)
{
    Robot.Moving.Rotary.Speed.Analytics = 1;
}

////////////////////////////////////////////
void robotMovingRotaryAnalyticsDisable(void)
{
    Robot.Moving.Rotary.Speed.Analytics = 0;
}

////////////////////////////////////////////
static void robotMovingRotaryAnalytics(void)
{
    double tempSpeedRotary = 0;
    
    // Lay gia tri ngo ra PID lam toc do chay
    // Khi chay toi goc thiet lap
    // -> Current ~ 0 -> Robot se tu dong dung lai
    if (Robot.Moving.Rotary.Speed.Analytics)
    {
        PID_Process_Basic(&Robot.Moving.Rotary.Speed.PID,
                    Robot.Moving.Rotary.Run,
                    Robot.Moving.Rotary.Current);
        
        tempSpeedRotary = Robot.Moving.Rotary.Speed.PID.Output.Current;
    }
    
    // Lay gia tri toc do cai dat lam gia tri chay ngay lap tuc
    else
    {
        PID_Reset(&Robot.Moving.Rotary.Speed.PID);
        
        tempSpeedRotary = Robot.Moving.Rotary.Run;
    }

    Robot.Moving.Mecanum.Speed.Rotary.A = -tempSpeedRotary * Robot.Moving.Mecanum.Radius.Ratio;    
    Robot.Moving.Mecanum.Speed.Rotary.B1 = tempSpeedRotary;
    Robot.Moving.Mecanum.Speed.Rotary.B2 = tempSpeedRotary;
}

////////////////////////////////////////////
static void robotMovingCalculatorMotor(void)
{
    double tempMotorA = Robot.Moving.Mecanum.Speed.Direction.A + Robot.Moving.Mecanum.Speed.Rotary.A;
    double tempMotorB1 = Robot.Moving.Mecanum.Speed.Direction.B1 + Robot.Moving.Mecanum.Speed.Rotary.B1;
    double tempMotorB2 = Robot.Moving.Mecanum.Speed.Direction.B2 - Robot.Moving.Mecanum.Speed.Rotary.B2;
    
    tempMotorB2 *= (-1);
    
    Robot.Moving.Mecanum.Speed.Total.A = tempMotorA;
    Robot.Moving.Mecanum.Speed.Total.B1 = tempMotorB1;
    Robot.Moving.Mecanum.Speed.Total.B2 = tempMotorB2;
}

//////////////////////////////////////////////////
void robotMovingSetAccelerationValue(uint8_t Value)
{
    Robot.Moving.Direction.Speed.Acceleration = Value;
}

///////////////////////////////////////////////////
void robotMovingSetDecelerationValue(uint8_t Value)
{
    Robot.Moving.Direction.Speed.Deceleration = Value;
}
void Robot_Moving_Init(void)
{
  robotMovingRotaryRadiusSet(ROBOT_ROTARY_RADIUS_A, ROBOT_ROTARY_RADIUS_B);
  robotMovingSpeedRangeSet(ROBOT_MOVING_SPEED_OUTPUT_MIN, ROBOT_MOVING_SPEED_OUTPUT_MAX);
  robotMovingDirectionRangeSet(0, 360);
  
  
  robotMovingRotaryPIDSet(1.65, 15, 0.005 , 0, 300);  
  
  robotMovingRotaryAnalyticsEnable();
  
  robotMovingDirectionSet(0);
  robotMovingRotarySet(0);
  robotMovingDirectionSpeedSet(0);
}
///////////////////////////////
static void robotMovingAnalytics(void)
{
    if (Robot.Moving.Direction.Speed.Run < Robot.Moving.Direction.Speed.Min) Robot.Moving.Direction.Speed.Run = Robot.Moving.Direction.Speed.Min;
    if (Robot.Moving.Direction.Speed.Max < Robot.Moving.Direction.Speed.Run) Robot.Moving.Direction.Speed.Run = Robot.Moving.Direction.Speed.Max;
    
    // Khi hoat dong o che do tu dong
    if (1)
    {
        int delta = Robot.Moving.Direction.Speed.Run - Robot.Moving.Direction.Speed.Current;
        
        // Lenh tang toc do robot
        if (0 < delta)
        {
            Robot.Moving.Direction.Speed.Current += Robot.Moving.Direction.Speed.Acceleration;
        }
        
        // Lenh giam toc do robot
        else if (delta < 0)
        {
            Robot.Moving.Direction.Speed.Current -= Robot.Moving.Direction.Speed.Deceleration;
        }
        
        // Lenh dung robot
        else
        {
            Robot.Moving.Direction.Speed.Current = Robot.Moving.Direction.Speed.Run;
        }
    }
    
    // Khi hoat dong o che do bang tay
   
    
    /* Tinh toan toc do theo phuong trinh di chuyen tinh tien */
    robotMovingDirectionAnalytics();
    
    /* Tinh toan toc do theo phuong trinh di chuyen xoay */
    robotMovingRotaryAnalytics();
 
    /* Tinh toan tong hop toc do phuong trinh di chuyen tinh tien + xoay */
    robotMovingCalculatorMotor();
}
void robotMovingHandleOutput(void)
{
    robotMovingAnalytics();
    DC_Servo_Driver_UART_Speed_Control(6,(int16_t)-Robot.Moving.Mecanum.Speed.Total.A);
    DC_Servo_Driver_UART_Speed_Control(5,(int16_t)Robot.Moving.Mecanum.Speed.Total.B1);
    DC_Servo_Driver_UART_Speed_Control(1,(int16_t)Robot.Moving.Mecanum.Speed.Total.B2);    
}