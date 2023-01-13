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

  //A = -kx*error_x - kv*error_v - m*g*e3 + m*ad
  matrix* A = newMatrix(3,1);
  matrix* kxex = newMatrix(3,1);
  matrix* kvev = newMatrix(3,1);
  matrix* mge3 = newMatrix(3,1);
  matrix* mad = newMatrix(3,1);
  matrix* xv = newMatrix(3,1);
  matrix* mad_mge3 = newMatrix(3,1);
  // kx*error_x
  productMatrix(se3->kx, error_pos, kxex);
  // kv*error_v
  productMatrix(se3->kv, error_pos, kvev);
  // m*g*e3
  productScalarMatrix(POINT_MASS_KG*ENV_GRAVITY_MPS2, se3->e3, mge3);
  // m*ad
  productScalarMatrix(POINT_MASS_KG, des_acc, mad);
  // + kx*error_x + kv*error_v
  sumMatrix(kxex,kvev,xv);
  // m*ad - m*g*e3
  subtractMatrix(mad,mge3,mad_mge3);
  // m*ad - m*g*e3 - kx*error_x - kv*error_v
  subtractMatrix(mad_mge3,xv,A);
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

  /*Now we are going to construct the desired/control rotation matrix
    direction of the thrust vector (desired/control)
    aka body z direction
  */
  float normA = normL2Vec(A);
  matrix* b3_c = newMatrix(3,1);
  productScalarMatrix(-1/normA, A, b3_c);
  /*vector orthogonal to thrust direction and desired heading.
    this is directing to y axis
  */
  matrix* C = newMatrix(3,1);
  crossProduct3DVec(b3_c, b1_d, C);
  /*orthonormal desired body y direction
  */
  matrix* b2_c = newMatrix(3,1);
  float normC = normL2Vec(C);
  productScalarMatrix(1/normC, C, b2_c);
  /*orthonormal desired body x direction
  */
  matrix* b1_c = newMatrix(3,1);
  crossProduct3DVec(b2_c, b3_c, b1_c);
  /*construct the rotation matrix to be tracked: R_c = [b1_c b2_c b3_c]
  */
  matrix* R_c = newMatrix(3,3);
  matrix* R_c_23 = newMatrix(3,2);
  matrixConcatenation(b2_c, b3_c, R_c_23);
  matrixConcatenation(b1_c, R_c_23, R_c);

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