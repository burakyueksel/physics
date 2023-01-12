/**
 * @author  Burak Yueksel
 * @date    02 January 2023
 * @brief   Control libraries
 * @addtogroup CONTROLS
 **/

#include "controls.h"
#include "parameters.h"
#include <math.h> // for cos, sin etc

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


void initSE3Ctrl(SE3Controller *se3)
{
  se3->e3 = newMatrix(3,1); // [0;0;1] unit vector (gravity direction in NED)
  setMatrixElement(se3->e3,  1,  1,  0.0f);
  setMatrixElement(se3->e3,  2,  1,  0.0f);
  setMatrixElement(se3->e3,  3,  1,  1.0f);

  se3->kx = newMatrix(3,3); // P gain matrix for position errors
  setMatrixElement(se3->kx,  1,  1,  4.0 * POINT_MASS_KG);
  setMatrixElement(se3->kx,  2,  2,  4.0 * POINT_MASS_KG);
  setMatrixElement(se3->kx,  3,  3,  4.0 * POINT_MASS_KG);

  se3->kv = newMatrix(3,3); // D gain matrix for velocity errors
  setMatrixElement(se3->kv,  1,  1,  5.6 * POINT_MASS_KG);
  setMatrixElement(se3->kv,  2,  2,  5.6 * POINT_MASS_KG);
  setMatrixElement(se3->kv,  3,  3,  5.6 * POINT_MASS_KG);

  se3->kR = newMatrix(3,3); // Propotional gain multiplying rotation matrix errors
  setMatrixElement(se3->kR,  1,  1,  8.81);
  setMatrixElement(se3->kR,  2,  2,  8.81);
  setMatrixElement(se3->kR,  3,  3,  8.81);

  se3->kOmega = newMatrix(3,3); // Propotional gain multiplying rotational velocity errors
  setMatrixElement(se3->kOmega,  1,  1,  2.54);
  setMatrixElement(se3->kOmega,  2,  2,  2.54);
  setMatrixElement(se3->kOmega,  3,  3,  2.54);
}

void updateSE3Ctrl(SE3Controller *se3,
                    matrix* des_pos, matrix* des_vel, matrix* des_acc, matrix* des_jerk,
                    matrix* pos, matrix* vel, matrix* acc, matrix* jerk,
                    matrix* R, matrix* rotVel,
                    float yawRef, float yawRefDot, float yawRefdDot)
{

  // compute translational errors
  matrix* error_pos = newMatrix(3,1);
  matrix* error_vel = newMatrix(3,1);
  matrix* error_acc = newMatrix(3,1);
  matrix* error_jerk = newMatrix(3,1);

  subtractMatrix(pos, des_pos, error_pos);
  subtractMatrix(vel, des_vel, error_vel);
  subtractMatrix(acc, des_acc, error_acc);
  subtractMatrix(jerk, des_jerk, error_jerk);

  // control thrust and its magnitude

  //A = -kx*error_x - kv*error_v - m*g*e3 + m*ad;
  matrix* A = newMatrix(3,1);
  matrix* A1 = newMatrix(3,1);
  matrix* A2 = newMatrix(3,1);
  matrix* A3 = newMatrix(3,1);
  matrix* A4 = newMatrix(3,1);
  matrix* A12 = newMatrix(3,1);
  matrix* A34 = newMatrix(3,1);
  productMatrix(se3->kx, error_pos, A1);
  productMatrix(se3->kv, error_pos, A2);
  productScalarMatrix(POINT_MASS_KG*ENV_GRAVITY_MPS2, se3->e3, A3);
  productScalarMatrix(POINT_MASS_KG, des_acc, A4);
  sumMatrix(A1,A2,A12);
  subtractMatrix(A4,A3,A34);
  subtractMatrix(A34,A12,A);
  //f = vec_dot(-A, R*e3); // equal to -transpose(A)*R*e3
  matrix* f     = newMatrix(3,1);
  matrix* AT    = newMatrix(3,1);
  matrix* negAT = newMatrix(3,1);
  matrix* Re3   = newMatrix(3,1);
  transposeMatrix(A, AT);
  productScalarMatrix(-1.0,AT,negAT);
  productMatrix(R,se3->e3,Re3);
  productMatrix(negAT,Re3,f);
  // desired heading as function of yaw and its derivatives
  matrix* b1_d          = newMatrix(3,1);
  matrix* b1_d_dot      = newMatrix(3,1);
  matrix* b1_d_ddot     = newMatrix(3,1);
  float cs_yref = cos(yawRef);
  float sn_yref = sin(yawRef);
  float yawRefDot_square = yawRefDot*yawRefDot;

  setMatrixElement(b1_d,  1,  1,  cs_yref);
  setMatrixElement(b1_d,  2,  1,  sn_yref);
  setMatrixElement(b1_d,  3,  1,  0.0f);

  setMatrixElement(b1_d_dot,  1,  1,  -sn_yref*yawRefDot);
  setMatrixElement(b1_d_dot,  2,  1,  cs_yref*yawRefDot);
  setMatrixElement(b1_d_dot,  3,  1,  0.0f);

  setMatrixElement(b1_d_ddot,  1,  1,  (-sn_yref*yawRefdDot -cs_yref*yawRefDot_square));
  setMatrixElement(b1_d_ddot,  2,  1,  (cs_yref*yawRefdDot - sn_yref*yawRefDot_square));
  setMatrixElement(b1_d_ddot,  3,  1,  0.0f);

  // TODO: to be cont.
}

/*
void ctrlInit(PIDController* pid)
{
  float kp = POINT_MASS_KG * CTRL_PID_OMEGA_RPS*CTRL_PID_OMEGA_RPS/ENV_GRAVITY_MPS2;
  float kd = 2.0 * CTRL_PID_OMEGA_RPS * CTRL_PID_DAMPING/ENV_GRAVITY_MPS2;
  float ki = kd;

  initPID(&pid, kp, kd, ki);
}
*/