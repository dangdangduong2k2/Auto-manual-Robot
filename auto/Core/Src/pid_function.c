#include "pid_function.h"

/*********************/
void PID_Init(pid *PID,
              double T,
              double Kp,
              double Ki,
              double Kd,
              double OutMin,
              double OutMax)
{
    PID->T = T;

    PID->Output.Min = OutMin;
    PID->Output.Max = OutMax;

    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    
    PID->Alpha = (2 * PID->Kp * PID->T) + (PID->Ki * PID->T * PID->T) + (2 * PID->Kd);
    PID->Beta = - (2 * PID->Kp * PID->T) + (PID->Ki * PID->T * PID->T) - (4 * PID->Kd);
    PID->Gama = (2 * PID->Kd);
}

/************************/
void PID_Process(pid *PID,
                 double Setpoint,
                 double CurrentPoint)
{
    PID->E = Setpoint - CurrentPoint;
    PID->Output.Current = (PID->Alpha * PID->E + PID->Beta * PID->E1 + PID->Gama * PID->E2 + 2 * PID->T * PID->Output.Last) / (2 * PID->T);
    PID->Output.Last = PID->Output.Current;
    PID->E2 = PID->E1;
    PID->E1 = PID->E;

    if (PID->Output.Min != 0)
    {
        if (0 < PID->Output.Current && PID->Output.Current < PID->Output.Min)
        {
            PID->Output.Current = PID->Output.Min;
        }
        else if (-PID->Output.Min < PID->Output.Current && PID->Output.Current < 0)
        {
            PID->Output.Current = -PID->Output.Min;
        }
    }

    if (PID->Output.Max != 0)
    {
        if (PID->Output.Max < PID->Output.Current)
        {
            PID->Output.Current = PID->Output.Max;
            PID->Output.Last = PID->Output.Max;
        }
        else if (PID->Output.Current < (-PID->Output.Max))
        {
            PID->Output.Current = -PID->Output.Max;
            PID->Output.Last = -PID->Output.Max;
        }
    }
}

/******************************/
void PID_Process_Basic(pid *PID,
                       double Setpoint,
                       double CurrentPoint)
{
    PID->E = Setpoint - CurrentPoint;
    
    PID->Beta = PID->E * PID->Ki * PID->T;
    
    if (PID->Beta >= PID->Output.Max)
    {PID->Beta = PID->Output.Max;}
    else if (PID->Beta <= -PID->Output.Max)
    {PID->Beta = -PID->Output.Max;}

    PID->Gama = (PID->E - PID->E1) * PID->Kd / PID->T; 
    PID->E1 = PID->E;
    PID->Output.Current = PID->E * PID->Kp + PID->Gama + PID->Beta;
       
    PID->Output.Last = PID->Output.Current;
    PID->E2 = PID->E1;
    PID->E1 = PID->E;

    if (PID->Output.Min != 0)
    {
        if (0 < PID->Output.Current && PID->Output.Current < PID->Output.Min)
        {
            PID->Output.Current = PID->Output.Min;
        }
        else if (-PID->Output.Min < PID->Output.Current && PID->Output.Current < 0)
        {
            PID->Output.Current = -PID->Output.Min;
        }
    }

    if (PID->Output.Max != 0)
    {
        if (PID->Output.Max < PID->Output.Current)
        {
            PID->Output.Current = PID->Output.Max;
            
        }
        else if (PID->Output.Current < (-PID->Output.Max))
        {
            PID->Output.Current = -PID->Output.Max;
            
        }
    }
    //PID->Output.Current = -PID->Output.Current;
}

/**********************/
void PID_Reset(pid *PID)
{
    PID->E = 0;
    PID->Output.Current = 0;
    PID->Output.Last = 0;
    PID->E2 = 0;
    PID->E1 = 0;
}
