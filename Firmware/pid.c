//Adapted from Atmel AVR221 - Josh Blake
#include "pid.h"
#define PID_SCALING_FACTOR 1

void pid_Init(float p_factor, float i_factor, float d_factor, struct PID_DATA *pid)
{
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_LONG / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

unsigned long pid_Controller(float setPoint, float processValue, struct PID_DATA *pid_st)
{
  float error, p_term, d_term;
  float i_term, ret, temp;

  error = setPoint - processValue;

  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){
    p_term = MAX_LONG;
  }
  else if (error < -pid_st->maxError){
    p_term = -MAX_LONG;
  }
  else{
    p_term = pid_st->P_Factor * error;
  }

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  }
  else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  }
  else{
    pid_st->sumError = temp;
    i_term = pid_st->I_Factor * pid_st->sumError;
  }

  // Calculate Dterm
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);

  pid_st->lastProcessValue = processValue;

  ret = (p_term + i_term + d_term) / PID_SCALING_FACTOR;
  //Bound Check the PID Value
  if(ret > MAX_LONG){
    ret = MAX_LONG;
  }
  else if(ret < 0){
    ret = 0;
  }
  return ret;
}

void pid_Reset_Integrator(pidData_t *pid_st)
{
  pid_st->sumError = 0;
}
