/**
 * @author  Burak Yueksel
 * @date    02 January 2023
 * @brief   Control libraries
 * @addtogroup CONTROLS
 **/

#include "controls.h"
#include "parameters.h"
#include "mathematics.h"
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

  se3->ctrlForces_N = newMatrix(3,1);   // Fx, Fy, Fz
  se3->ctrlMoments_Nm = newMatrix(3,1); // Mx, My, Mz
}

void initTiltPrioCtrl(TiltPrioCtrl *tiltCtrl)
{
  // source:https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
  float timeConst_xy = 0.3;
  float timeConst_z  = 0.3;
  float damping_xy   = 0.707;
  float damping_z    = 0.707;

  tiltCtrl->kp_xy = 2*POINT_I_XX_KGM2/(timeConst_xy*timeConst_xy); // see eq. 37
  tiltCtrl->kp_z = 2*POINT_I_ZZ_KGM2/(timeConst_z*timeConst_z); // see eq. 37

  float kd_xy = 2*POINT_I_XX_KGM2*damping_xy/timeConst_xy; // see eq. 38
  float kd_z  = 2*POINT_I_ZZ_KGM2*damping_z/timeConst_z; // see eq.38

  tiltCtrl->kd = newMatrix(3,1);
  setMatrixElement(tiltCtrl->kd,  1,  1,  kd_xy);
  setMatrixElement(tiltCtrl->kd,  2,  2,  kd_xy);
  setMatrixElement(tiltCtrl->kd,  3,  3,  kd_z);

  tiltCtrl->J_kgm2 = newMatrix(3,3);
  /*set the moment of inertia terms*/
  setMatrixElement(tiltCtrl->J_kgm2, 1, 1, POINT_I_XX_KGM2);//Ixx
  setMatrixElement(tiltCtrl->J_kgm2, 2, 2, POINT_I_YY_KGM2);//IYY
  setMatrixElement(tiltCtrl->J_kgm2, 3, 3, POINT_I_ZZ_KGM2);//IZZ
  setMatrixElement(tiltCtrl->J_kgm2, 1, 2, POINT_I_XY_KGM2);//IXY
  setMatrixElement(tiltCtrl->J_kgm2, 2, 1, POINT_I_XY_KGM2);//IYX=IXY
  setMatrixElement(tiltCtrl->J_kgm2, 1, 3, POINT_I_XZ_KGM2);//IXZ
  setMatrixElement(tiltCtrl->J_kgm2, 3, 1, POINT_I_XZ_KGM2);//IZX=IXZ
  setMatrixElement(tiltCtrl->J_kgm2, 2, 3, POINT_I_YZ_KGM2);//IYZ
  setMatrixElement(tiltCtrl->J_kgm2, 3, 2, POINT_I_YZ_KGM2);//IZY=IYZ

  tiltCtrl->ctrlMoments_Nm = newMatrix(3,1);
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

void updateSE3Ctrl(SE3Controller *se3,
                    matrix* des_pos, matrix* des_vel, matrix* des_acc, matrix* des_jerk, matrix* des_snap,
                    matrix* pos, matrix* vel, matrix* acc, matrix* jerk,
                    matrix* R, matrix* rotVel,
                    float yawRef, float yawRefDot, float yawRefdDot)
{
  // source: https://mathweb.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf
  // source: https://arxiv.org/abs/1003.2005v1
  // notice that there are 4 versions of this paper in arxiv.
  // explicit computations of omega_c, omega_dot_c and R_c, R_c_dot, R_c_ddot are provided here (v2), section F:
  // [1] https://arxiv.org/pdf/1003.2005v2.pdf

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
  // see eq.42 and the equations/descriptions after eq. 97 of [1]
  //A = -kx*error_x - kv*error_v - m*g*e3 + m*ad
  matrix* A = newMatrix(3,1);
  sum4Matrix(returnNegMatrix(returnProductMatrix(se3->kx, error_pos)),
             returnNegMatrix(returnProductMatrix(se3->kv, error_vel)),
             returnNegMatrix(returnProductScalarMatrix(POINT_MASS_KG*ENV_GRAVITY_MPS2, se3->e3)),
             returnProductScalarMatrix(POINT_MASS_KG, des_acc), A);
  //f = vec_dot(-A, R*e3); // equal to -transpose(A)*R*e3
  matrix* f     = newMatrix(3,1);
  matrix* AT    = newMatrix(3,1);// we will use AT later. Hence compute it once and store it.
  transposeMatrix(A, AT);
  productMatrix(returnNegMatrix(AT),returnProductMatrix(R,se3->e3),f);
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
  float normA = normL2Vec(A); // another repeatedely used value. Hence compute once and store.
  matrix* b3_c = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  productScalarMatrix(-1/normA, A, b3_c);
  /*vector orthogonal to thrust direction and desired heading.
    this is directing to y axis
  */
  // see sec F, equations/descriptions after eq. 97 of [1]
  // C = vec_cross(b3_c, b1_d);
  matrix* C = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  crossProduct3DVec(b3_c, b1_d, C);
  /*orthonormal desired body y direction
  */
  matrix* b2_c = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  float normC = normL2Vec(C); // another repeatedely used value. Hence compute once and store.
  productScalarMatrix(1/normC, C, b2_c);
  /*orthonormal desired body x direction
  */
  matrix* b1_c = newMatrix(3,1);
  crossProduct3DVec(b2_c, b3_c, b1_c);
  deleteMatrix(b1_c); // remove the temp matrix
  /*construct the rotation matrix to be tracked: R_c = [b1_c b2_c b3_c]
  */
  matrix* R_c = newMatrix(3,3);
  matrix* R_c_23 = newMatrix(3,2);
  matrixConcatenation(b2_c, b3_c, R_c_23);
  matrixConcatenation(b1_c, R_c_23, R_c);
  deleteMatrix(R_c_23); // remove the temp matrix

  /*time derivatives of body axes for excessive attitude tracking performances
  A_dot   = -kx*error_v - kv*error_a + m*jd;
  */
  matrix* A_dot = newMatrix(3,1);
  sum3Matrix(returnNegMatrix(returnProductMatrix(se3->kx, error_vel)),
             returnNegMatrix(returnProductMatrix(se3->kv, error_acc)),
             returnProductScalarMatrix(POINT_MASS_KG,des_jerk),A_dot);

  /*time derivative of body z*
  b3_c_dot = -A_dot/norm(A) + (vec_dot(A,A_dot)/norm(A)^3)*A;
  // remember vec_dot(A,A_dot) = A^T * A_dot
  */
  float normACube = normA*normA*normA; // another repeatedely used value. Hence compute once and store.
  matrix* ATAdot = newMatrix(1,1); // another repeatedely used value. Hence compute once and store.
  matrix* b3_c_dot = newMatrix(3,1);
  productMatrix(AT,A_dot,ATAdot);
  sumMatrix(returnProductScalarMatrix(-1/normA, A_dot),
             returnProductMatrix(returnProductScalarMatrix(1/normACube, ATAdot),A),
             b3_c_dot);  /*
  C_dot   = vec_cross(b3_c_dot, b1_d) + vec_cross(b3_c, b1_d_dot);
  */
  matrix* C_dot = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  sumMatrix(returnCrossProduct3DVec(b3_c_dot, b1_d),returnCrossProduct3DVec(b3_c, b1_d_dot),C_dot);

  /*time derivative of body y
    b2_c_dot = C/norm(C) - (vec_dot(C,C_dot)/norm(C)^3)*C;
  */
 float normCCube = normC*normC*normC; // another repeatedely used value. Hence compute once and store.
  matrix* CT     = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  matrix* b2_c_dot = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.

  transposeMatrix(C, CT);
  sumMatrix(returnProductScalarMatrix(1/normC, C),
            returnProductMatrix(returnProductScalarMatrix(-1/normCCube,returnProductMatrix(CT,C_dot)), C), b2_c_dot);

  /*time derivative of body x
    b1_c_dot = vec_cross(b2_c_dot, b3_c) + vec_cross(b2_c, b3_c_dot);
  */
  matrix* b1_c_dot = newMatrix(3,1);
  sumMatrix(returnCrossProduct3DVec(b2_c_dot, b3_c),returnCrossProduct3DVec(b2_c, b3_c_dot), b1_c_dot);

  /* second time derivatives of body axes
  A_ddot   = -kx*error_a - kv*error_j + m*sd;
  */
  matrix* A_ddot = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  sum3Matrix(returnNegMatrix(returnProductMatrix(se3->kx,error_acc)),
             returnNegMatrix(returnProductMatrix(se3->kv,error_jerk)),
             returnProductScalarMatrix(POINT_MASS_KG, des_snap),A_ddot);

  /* second time derivative of body z
  b3_c_ddot = -A_ddot/norm(A) + (2/norm(A)^3)*vec_dot(A,A_dot)*A_dot ...
         + ((norm(A_dot)^2 + vec_dot(A,A_ddot))/norm(A)^3)*A       ...
         - (3/norm(A)^5)*(vec_dot(A,A_dot)^2)*A;
  */
 float normASquared = normA*normA; // another repeatedely used value. Hence compute once and store.
 float normA5       = normASquared*normACube; // another repeatedely used value. Hence compute once and store.
 float normA_dot    = normL2Vec(A_dot);
 matrix* normA_dotSquared  = newMatrix(1,1); // norm of A_dot will go in there in matrix form
 matrix* ATAdotSquared = returnProductMatrix(ATAdot,ATAdot);
 matrix* b3_c_ddot = newMatrix(3,1);
 setMatrixElement(normA_dotSquared,1,1,normA_dot*normA_dot);

 sum5Matrix(returnProductScalarMatrix(-1/normA, A_ddot),
            returnProductScalarMatrix(2/normACube, returnProductMatrix(ATAdot,A_dot)),
            returnProductMatrix(normA_dotSquared,A),
            returnProductMatrix(returnProductScalarMatrix(1/normACube,returnProductMatrix(AT,A_ddot)),A),
            returnProductScalarMatrix(-3/normA5,returnProductMatrix(ATAdotSquared,A)),b3_c_ddot);
  // TODO: to be cont.
  // delete all created matrices at the end
}

void updateTiltPrioCtrl(TiltPrioCtrl *tiltCtrl, matrix* rotVel, quaternion q,
                                             matrix* rotVelDes, quaternion qDes,
                                             matrix* rotVelDotEst)
{
  // eq. 13
  quaternion q_inv;
  quaternion q_error;
  quaternionInverse(q, &q_inv);
  quaternionProduct(qDes, q_inv, &q_error);
  // eq. 14
  matrix* rotVell_error = newMatrix(3,1);
  subtractMatrix(rotVelDes, rotVel, rotVell_error);
  // eq. 18
  quaternion q_err_red;
  float one_over_q_err_red_norm = quaternionZAlignNorm(q_error);
  q_err_red.w = one_over_q_err_red_norm * (q_error.w*q_error.w + q_error.z*q_error.z);
  q_err_red.x = one_over_q_err_red_norm * (q_error.w*q_error.x - q_error.y*q_error.z);
  q_err_red.y = one_over_q_err_red_norm * (q_error.w*q_error.y + q_error.x*q_error.z);
  q_err_red.z = 0.0f;
  // eq. 20
  quaternion q_err_yaw;
  q_err_yaw.w = one_over_q_err_red_norm * q_error.w;
  q_err_yaw.x = 0.0f;
  q_err_yaw.y = 0.0f;
  q_err_yaw.z = one_over_q_err_red_norm * q_error.z;
  // eq. 23
  matrix* tau_ff = newMatrix(3,1);
  matrix* JomegaXomega = newMatrix(3,1);
  crossProduct3DVec(returnProductMatrix(tiltCtrl->J_kgm2, rotVel), rotVel, JomegaXomega);
  sumMatrix(returnProductMatrix(tiltCtrl->J_kgm2, rotVelDotEst),returnNegMatrix(JomegaXomega),tau_ff);
  // eq. 21
  matrix* qv_e_red = newMatrix(3,1);
  matrix* qv_e_yaw = newMatrix(3,1);
  float signqw = signumf(q_error.w);
  getQuaternionVectorPart(q_err_red, qv_e_red);
  getQuaternionVectorPart(q_err_yaw, qv_e_yaw);
  sum4Matrix(returnProductScalarMatrix(tiltCtrl->kp_xy, qv_e_red),
             returnProductScalarMatrix(signqw*tiltCtrl->kp_z, qv_e_yaw),
             returnProductMatrix(tiltCtrl->kd, rotVell_error),
             tau_ff,
             tiltCtrl->ctrlMoments_Nm);
  // release memory
  deleteMatrix(rotVell_error);
  deleteMatrix(tau_ff);
  deleteMatrix(JomegaXomega);
  deleteMatrix(qv_e_red);
  deleteMatrix(qv_e_yaw);
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