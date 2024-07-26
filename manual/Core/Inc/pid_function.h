#ifndef __PID_FUNCTION_H__
#define __PID_FUNCTION_H__

/****************/
#ifdef __cplusplus
 extern "C" {
#endif

/************/
typedef struct
{
    double Kp, Ki, Kd;
    double Alpha, Beta, Gama;
    double T;
    double E, E1, E2;
    struct
    {
        double Max, Min;
        double Current, Last;
    } Output;
} pid;

/*********************/
void PID_Init(pid *PID,
              double T,
              double Kp,
              double Ki,
              double Kd,
              double OutMin,
              double OutMax);

/***********************/
void PID_Process(pid *PID,
                 double Setpoint,
                 double CurrentPoint);

/******************************/
void PID_Process_Basic(pid *PID,
                        double Setpoint,
                        double CurrentPoint);

/***********************/
void PID_Reset(pid *PID);

/****************/
#ifdef __cplusplus
}
#endif
#endif
