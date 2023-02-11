#ifndef CONTROLS_H_
#define CONTROLS_H_

/**
 * @author  Burak Yueksel
 * @date    02 January 2023
 * @brief   Control libraries
 * @addtogroup CONTROLS
 **/

#include "matrix.h"
#include "vector.h"

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
  float mass_kg;
  matrix* J_kgm2;
  matrix* ctrlThrust_N;
  matrix* ctrlMoments_Nm;
} SE3Controller;

typedef struct
{
  matrix* kd;
  float kp_xy;
  float kp_z;
  matrix* J_kgm2;
  matrix* ctrlMoments_Nm;
} TiltPrioCtrl;

typedef struct
{
  matrix* KV;
  matrix* KD_eta;
  matrix* KP_des_eta;
  float mass_kg;
  float mass_des_kg;
  matrix* J_kgm2;
  matrix* J_des_kgm2;
  matrix* J_des_inv_kgm2;
  matrix* ctrlThrust_N;
  matrix* ctrlMoments_Nm;
} IDAPBCCtrl;



//void ctrlInit(PIDController* pid);
void initPID(PIDController* pid, float kp, float ki, float kd);
void initSE3Ctrl(SE3Controller *se3);
void initTiltPrioCtrl(TiltPrioCtrl *tltCtl);
void initIDAPBCCtrl(IDAPBCCtrl * idapbcCtrl);

float updatePID(PIDController *pid, float error, float dt);

void updateSE3Ctrl(SE3Controller *se3,
                    matrix* des_pos, matrix* des_vel, matrix* des_acc, matrix* des_jerk,
                    matrix* pos, matrix* vel, matrix* acc, matrix* jerk, matrix* des_snap,
                    matrix* R, matrix* rotVel,
                    float yawRef, float yawRefDot, float yawRefdDot);
void updateTiltPrioCtrl(TiltPrioCtrl *tltCtl, matrix* rotVel, quaternion q,
                                             matrix* rotVelDes, quaternion qDes,
                                             matrix* rotVelDotEst);
void updateIDAPBCCtrl(IDAPBCCtrl *idapbc, matrix* vel,
                      matrix* rotMat, matrix* eul_rad,
                      matrix* omega_rps, matrix* eul_des_rad,
                      matrix* wrench_ext);
#endif // CONTROLS_H_