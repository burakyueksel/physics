#ifndef CONTROLS_H_
#define CONTROLS_H_

/**
 * @author  Burak Yueksel
 * @date    02 January 2023
 * @brief   Control libraries
 * @addtogroup CONTROLS
 **/

#include "matrix.h"

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

typedef struct
{
  matrix* kx;
  matrix* kv;
  matrix* kR;
  matrix* kOmega;
  matrix* e3;
} SE3Controller;


//void ctrlInit(PIDController* pid);
void initPID(PIDController* pid, float kp, float ki, float kd);
float updatePID(PIDController *pid, float error, float dt);

void initSE3Ctrl(SE3Controller *se3);
void updateSE3Ctrl(SE3Controller *se3,
                    matrix* des_pos, matrix* des_vel, matrix* des_acc, matrix* des_jerk,
                    matrix* pos, matrix* vel, matrix* acc, matrix* jerk,
                    matrix* R, matrix* rotVel,
                    float yawRef, float yawRefDot, float yawRefdDot);

#endif // CONTROLS_H_