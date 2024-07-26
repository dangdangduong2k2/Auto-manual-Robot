#ifndef __ROBOT_MOVING_H
#define __ROBOT_MOVING_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "pid_function.h"
#include "math.h"
#include "dc_servo_driver_rs485.h"



#define DT_ANALYTICS                                            (double)0.001//10 Hz

#define IMU_DT_OFFSET                                           (double)0.00095

#define ROBOT_LAZER_MAX_RANGE                                   (double)10000.0
#define ROBOT_LAZER_OFFSET                                      (int16_t)130

#define ROBOT_ROTARY_RADIUS_A                                   (double)250.0
#define ROBOT_ROTARY_RADIUS_B                                   (double)325.0

#define ROBOT_ROTARY_RADIUS_RATIO                               (ROBOT_ROTARY_RADIUS_A / ROBOT_ROTARY_RADIUS_B)

////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ROBOT_MOVING_SPEED_OUTPUT_MIN                           0
#define ROBOT_MOVING_SPEED_OUTPUT_MAX                           900  
   
#define PI                              ((double)3.1415926535897932384626433832795)
#define HALF_PI                         ((double)1.5707963267948966192313216916398)
#define TWO_PI                          ((double)6.283185307179586476925286766559)
#define DEG_TO_RAD                      ((double)0.017453292519943295769236907684886)
#define RAD_TO_DEG                      ((double)57.295779513082320876798154814105)
   
   
                                   /*180*/
   
                        //////////////////////////////   
                        // 2                       3//    
                        //                          //
             /*90*/      //                      //         /*270*/
                            //        1         //
                              /////////////////
   
                                   /*0*/
   
   
   
typedef struct
{

    // Thong so toc do va huong di chuyen tinh tien
    struct
    {
        int16_t Min, Max;
        int16_t Run, Current;
        struct
        {
            uint16_t Min, Max;
            int16_t Run, Current;
            uint16_t Acceleration, Deceleration, Saturation; 
            uint16_t SetSpeed, SaturationSpeed;
        } Speed;
    } Direction;
    
    // Thong so toc do va huong di chuyen xoay
    struct
    {
        double Min, Max;
        int16_t Run, Current;
        
        struct
        {
            // Cho phep tinh toan dua theo thong so IMU
            uint8_t Analytics;
            // Thong so toc do xoay tinh theo PID
            pid PID;
            double Max;
        } Speed;
    } Rotary;
    
    struct 
    {
        struct
        {
          int Run,Current;
        } X;
        // Toc do tong hop tu huong di chuyen va huong xoay
        struct
        {
          int Run,Current; 
        } Y;
    } Position;
    
    
    // He di chuyen banh Mecanum
    struct
    {   uint8_t Auto;
        struct
        {
            int32_t X,Y;
            float Angle;
            uint8_t State;
        } Point;
        
  
        pid PIDX;
        pid PIDY;
        struct
        {
            double A;
            double B;
            double Ratio;
        } Radius;
        
        struct
        {
            int32_t Buffer[7];
            struct
            {
                uint16_t General;
                int32_t Actual_Position;
                int16_t Actual_Speed;
                uint16_t Motor_Current;
                int16_t Motor_Voltage;
                uint16_t Vsup;
                uint16_t Fet_Temperature;
            } Status;
        } Info[3]; // 0: A, 1: B1, 2: B2
        
        struct
        {
            // Toc do tinh theo huong di chuyen
            struct
            {
                double A, B1, B2;
            } Direction;
            // Toc do tinh theo huong xoay (IMU)
            struct
            {
                double A, B1, B2;
            } Rotary;
            // Toc do tong hop tu huong di chuyen va huong xoay
            struct
            {
                double A, B1, B2;
            } Total;
        } Speed;
    } Mecanum;
    
} moving;


/************/
typedef struct
{
    uint8_t isHomeCompleted;
    uint8_t isRunning;
    uint8_t isHoming;
    uint32_t iTick;
    uint8_t isAnalogCompleted : 1; 
    moving Moving;
    
} robot;


extern robot Robot;
/*****************/


/*************************************************************************/

void robotMovingReverseCheck(void);
   
void robotMovingSpeedRangeSet(uint16_t Min, uint16_t Max);
void robotMovingDirectionSpeedSet(int16_t Speed);

void robotMovingDirectionRangeSet(int16_t Min, int16_t Max);
void robotMovingDirectionSet(int16_t Direction);

void robotMovingRotaryRadiusSet(double A, double B);
void robotMovingRotaryRangeSet(double Min, double Max);
void robotMovingRotarySet(int16_t Rotary);
void robotMovingRotaryPIDSet(double Kp,
                             double Ki,
                             double Kd,
                             double Min,
                             double Max);

void robotMovingRotarySpeedSet(double Max);
////////////////////////////////////////////////////
void robotMovingSetAccelerationValue(uint8_t Value);
void robotMovingSetDecelerationValue(uint8_t Value);

////////////////////////////////////////////
void robotMovingRotaryAnalyticsEnable(void);
void robotMovingRotaryAnalyticsDisable(void);

////////////////////////////////


///////////////////////////////////////
void robotMovingHandleOutput(void);


///////////////////////////////////////
void robotMovingMecanumEncoderSet(void);
int32_t robotMovingMecanumEncoderCalculate(void);
void Robot_Moving_Init(void);


/****************/
#ifdef __cplusplus
}
#endif
#endif
