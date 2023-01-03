#ifndef CONTROLS_H_
#define CONTROLS_H_

/**
 * @author  Burak Yueksel
 * @date    02 January 2023
 * @brief   Control libraries
 * @addtogroup CONTROLS
 **/

// Structure to store PID controller data
typedef struct {
  float kp;  // Proportional gain
  float ki;  // Integral gain
  float kd;  // Derivative gain
  float error;  // Current error
  float integral;  // Current integral
  float derivative;  // Current derivative
  float prev_error;  // Previous error
} PIDController;

//void ctrlInit(PIDController* pid);
void initPID(PIDController* pid, float kp, float ki, float kd);
float updatePID(PIDController *pid, float error, float dt);

#endif // CONTROLS_H_