/**
 * @author  Burak Yueksel
 * @date    02 January 2023
 * @brief   Control libraries
 * @addtogroup CONTROLS
 **/

#include "controls.h"

// Initialize PID controller
void initPID(PIDController *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->error = 0;
  pid->integral = 0;
  pid->derivative = 0;
  pid->prev_error = 0;
}

// Update PID controller
float updatePID(PIDController *pid, float error, float dt)
{
  pid->error = error;
  pid->integral += error * dt;
  pid->derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;
  float output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
  return output;
}