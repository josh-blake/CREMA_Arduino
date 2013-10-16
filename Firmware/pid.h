//Adapted from Atmel AVR221 - Josh Blake
#ifndef PID_H
#define PID_H

typedef struct PID_DATA{
  //! Last process value, used to find derivative of process value.
  float lastProcessValue;
  //! Summation of errors, used for integrate calculations
  float sumError;
  //! The Proportional tuning constant, multiplied with PID_SCALING_FACTOR
  float P_Factor;
  //! The Integral tuning constant, multiplied with PID_SCALING_FACTOR
  float I_Factor;
  //! The Derivative tuning constant, multiplied with PID_SCALING_FACTOR
  float D_Factor;
  //! Maximum allowed error, avoid overflow
  float maxError;
  //! Maximum allowed sumerror, avoid overflow
  float maxSumError;
} pidData_t;

//#define MAX_INT         0x7FFF
#define MAX_LONG        0x7FFF
#define MAX_I_TERM      (MAX_LONG / 2)

void pid_Init(float p_factor, float i_factor, float d_factor, struct PID_DATA *pid);
unsigned long pid_Controller(float setPoint, float processValue, struct PID_DATA *pid_st);
void pid_Reset_Integrator(pidData_t *pid_st);

#endif
