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

// init full se3 controller
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

  /*set the mass*/
  se3->mass_kg = POINT_MASS_KG;

  se3->J_kgm2 = newMatrix(3,3);
  /*set the moment of inertia terms*/
  setMatrixElement(se3->J_kgm2, 1, 1, POINT_I_XX_KGM2);//Ixx
  setMatrixElement(se3->J_kgm2, 2, 2, POINT_I_YY_KGM2);//IYY
  setMatrixElement(se3->J_kgm2, 3, 3, POINT_I_ZZ_KGM2);//IZZ
  setMatrixElement(se3->J_kgm2, 1, 2, POINT_I_XY_KGM2);//IXY
  setMatrixElement(se3->J_kgm2, 2, 1, POINT_I_XY_KGM2);//IYX=IXY
  setMatrixElement(se3->J_kgm2, 1, 3, POINT_I_XZ_KGM2);//IXZ
  setMatrixElement(se3->J_kgm2, 3, 1, POINT_I_XZ_KGM2);//IZX=IXZ
  setMatrixElement(se3->J_kgm2, 2, 3, POINT_I_YZ_KGM2);//IYZ
  setMatrixElement(se3->J_kgm2, 3, 2, POINT_I_YZ_KGM2);//IZY=IYZ

  se3->ctrlThrust_N = newMatrix(1,1);   // Fz
  se3->ctrlMoments_Nm = newMatrix(3,1); // Mx, My, Mz
}

// init tilt prioritizing attitude controller
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

void initIDAPBCCtrl(IDAPBCCtrl *idapbc)
{
  // source 1: Journal paper: https://journals.sagepub.com/doi/full/10.1177/0278364919835605
  // source 2: Ch 4 of thesis https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf
  // source 3: Conference paper: https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf

  /*precompensation controller gains*/
  idapbc->KD_eta = newMatrix(3,3);
  idapbc->KP_des_eta = newMatrix(3,3);
  float omega = 0.3;
  float damping = 1.0;
  float kp_eta = omega*omega; // used for the desired rotational potential energy
  float kd_eta = 2*omega*damping; // used for the precomensation control input

  setMatrixElement(idapbc->KD_eta,  1,  1,  kd_eta);
  setMatrixElement(idapbc->KD_eta,  2,  2,  kd_eta);
  setMatrixElement(idapbc->KD_eta,  3,  3,  kd_eta);

  setMatrixElement(idapbc->KP_des_eta,  1,  1,  kp_eta);
  setMatrixElement(idapbc->KP_des_eta,  2,  2,  kp_eta);
  setMatrixElement(idapbc->KP_des_eta,  3,  3,  kp_eta);

  /*set the real mass*/
  idapbc->mass_kg = POINT_MASS_KG;
  /*set the desired mass*/
  idapbc->mass_des_kg = 1.5*POINT_MASS_KG;

  /*set the moment of inertia terms*/
  idapbc->J_kgm2 = newMatrix(3,3);
  setMatrixElement(idapbc->J_kgm2, 1, 1, POINT_I_XX_KGM2);//Ixx
  setMatrixElement(idapbc->J_kgm2, 2, 2, POINT_I_YY_KGM2);//IYY
  setMatrixElement(idapbc->J_kgm2, 3, 3, POINT_I_ZZ_KGM2);//IZZ
  setMatrixElement(idapbc->J_kgm2, 1, 2, POINT_I_XY_KGM2);//IXY
  setMatrixElement(idapbc->J_kgm2, 2, 1, POINT_I_XY_KGM2);//IYX=IXY
  setMatrixElement(idapbc->J_kgm2, 1, 3, POINT_I_XZ_KGM2);//IXZ
  setMatrixElement(idapbc->J_kgm2, 3, 1, POINT_I_XZ_KGM2);//IZX=IXZ
  setMatrixElement(idapbc->J_kgm2, 2, 3, POINT_I_YZ_KGM2);//IYZ
  setMatrixElement(idapbc->J_kgm2, 3, 2, POINT_I_YZ_KGM2);//IZY=IYZ

  /*set the desired moment of inertia terms*/
  idapbc->J_des_kgm2 = copyMatrix(idapbc->J_kgm2);

  /* inverse of the desired moment of inertia*/
  invertDiagonalMatrix(idapbc->J_des_kgm2, idapbc->J_des_inv_kgm2);

  /*damping injection gains*/
  // eq. 26 of [1]
  idapbc->KV = newMatrix(4,4);
  // KT
  float kT_bar = 1.0f;
  float kT = kT_bar*(idapbc->mass_kg*idapbc->mass_kg)/(idapbc->mass_des_kg*idapbc->mass_des_kg);
  // KR
  matrix* KR = newMatrix(3,3);
  matrix* KR_bar = newMatrix(3,3);
  productScalarMatrix(2, idapbc->KD_eta, KR_bar); // choose KR as 2*kd
  matrix* KR_barMinusKDN = newMatrix(3,3);
  subtractMatrix(KR_bar, returnProductMatrix(idapbc->KD_eta,idapbc->J_des_kgm2) , KR_barMinusKDN);
  productMatrix(idapbc->J_des_inv_kgm2, returnProductMatrix(KR_barMinusKDN, idapbc->J_des_inv_kgm2), KR);
  // set KV
  setMatrixElement(idapbc->KV,1,1,kT);
  setMatrixElement(idapbc->KV,2,2,ELEM(KR,1,1));
  setMatrixElement(idapbc->KV,3,3,ELEM(KR,2,3));
  setMatrixElement(idapbc->KV,4,4,ELEM(KR,3,3));

  // ctrl outputs
  idapbc->ctrlThrust_N = newMatrix(1,1);   // Fz
  idapbc->ctrlMoments_Nm = newMatrix(3,1); // Mx, My, Mz
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
             returnNegMatrix(returnProductScalarMatrix(se3->mass_kg*ENV_GRAVITY_MPS2, se3->e3)),
             returnProductScalarMatrix(se3->mass_kg, des_acc), A);
  //f = vec_dot(-A, R*e3); // equal to -transpose(A)*R*e3
  // f = se3->ctrlThrust_N
  matrix* AT    = newMatrix(1,3);// we will use AT later. Hence compute it once and store it.
  transposeMatrix(A, AT);
  productMatrix(returnNegMatrix(AT),returnProductMatrix(R,se3->e3),se3->ctrlThrust_N);
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
             returnProductScalarMatrix(se3->mass_kg,des_jerk),A_dot);

  /*time derivative of body z*
  b3_c_dot = -A_dot/norm(A) + (vec_dot(A,A_dot)/norm(A)^3)*A;
  // remember vec_dot(A,A_dot) = A^T * A_dot
  */
  float normASquared = normA*normA; // another repeatedely used value. Hence compute once and store.
  float normACube = normASquared*normA; // another repeatedely used value. Hence compute once and store.
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
  matrix* CTCdot = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.
  matrix* b2_c_dot = newMatrix(3,1); // another repeatedely used value. Hence compute once and store.

  transposeMatrix(C, CT);
  productMatrix(CT,C_dot,CTCdot);
  sumMatrix(returnProductScalarMatrix(1/normC, C),
            returnProductMatrix(returnProductScalarMatrix(-1/normCCube,CTCdot), C), b2_c_dot);

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
             returnProductScalarMatrix(se3->mass_kg, des_snap),A_ddot);

  /* second time derivative of body z
  b3_c_ddot = -A_ddot/norm(A) + (2/norm(A)^3)*vec_dot(A,A_dot)*A_dot ...
         + ((norm(A_dot)^2 + vec_dot(A,A_ddot))/norm(A)^3)*A       ...
         - (3/norm(A)^5)*(vec_dot(A,A_dot)^2)*A;
  */
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
            returnProductScalarMatrix(-3/normA5,returnProductMatrix(ATAdotSquared,A)),
            b3_c_ddot);
  /*
  time derivative of body y
  C_ddot   = vec_cross(b3_c_ddot, b1_d) + vec_cross(b3_c, b1_d_ddot)          ...
         + 2*vec_cross(b3_c_dot, b1_d_dot);
  b2_c_ddot = C_ddot/norm(C) - (2/norm(C)^3)*vec_dot(C,C_dot)*C_dot  ...
         - ((norm(C_ddot)^2 + vec_dot(C,C_ddot))/norm(C)^3)*C       ...
         + (3/norm(C)^5)*(vec_dot(C,C_dot)^2)*C;
  */
  matrix* C_ddot = newMatrix(3,1);
  sum3Matrix(returnCrossProduct3DVec(b3_c_ddot,b1_d),
             returnCrossProduct3DVec(b3_c, b1_d_ddot),
             returnProductScalarMatrix(2,returnCrossProduct3DVec(b3_c_dot,b1_d_dot)),
             C_ddot);
  matrix* b2_c_ddot = newMatrix(3,1);
  float normCSquared = normC*normC; // another repeatedely used value. Hence compute once and store.
  float normC5    = normCSquared*normCCube; // another repeatedely used value. Hence compute once and store.
  matrix* CTCdotSquared = returnProductMatrix(CTCdot,CTCdot);
  float normC_dot    = normL2Vec(C_dot);
  matrix* negNormC_dotSquared  = newMatrix(1,1); // norm of C_dot will go in there in matrix form
  setMatrixElement(negNormC_dotSquared,1,1,-normC_dot*normC_dot);

  sum5Matrix(returnProductScalarMatrix(1/normC, C_ddot),
             returnProductScalarMatrix(-2/normCCube, returnProductMatrix(CTCdot,C_dot)),
             returnProductMatrix(negNormC_dotSquared,C),
             returnProductMatrix(returnProductScalarMatrix(-1/normCCube,returnProductMatrix(CT,C_ddot)),C),
             returnProductScalarMatrix(3/normC5,returnProductMatrix(CTCdotSquared,C)),
             b2_c_ddot);
  /*
  time derivative of body x
  b1_c_ddot = vec_cross(b2_c_ddot, b3_c) + vec_cross(b2_c, b3_c_ddot)          ...
         + 2*vec_cross(b2_c_dot, b3_c_dot);
  */
  matrix* b1_c_ddot = newMatrix(3,1);
  sum3Matrix(returnCrossProduct3DVec(b2_c_ddot, b3_c),
             returnCrossProduct3DVec(b2_c, b3_c_ddot),
             returnProductScalarMatrix(2,returnCrossProduct3DVec(b2_c_dot,b3_c_dot)),
             b1_c_ddot);

  /*construct the time derivative of the rotation matrix to be tracked: R_c_dot = [b1_c_dot b2_c_dot b3_c_dot]
  */
  matrix* R_c_dot = newMatrix(3,3);
  matrix* R_c_dot_23 = newMatrix(3,2);
  matrixConcatenation(b2_c_dot, b3_c_dot, R_c_dot_23);
  matrixConcatenation(b1_c_dot, R_c_dot_23, R_c_dot);
  deleteMatrix(R_c_dot_23); // remove the temp matrix
  deleteMatrix(b1_c_dot); // remove temp matrix
  deleteMatrix(b2_c_dot); // remove temp matrix
  deleteMatrix(b3_c_dot); // remove temp matrix

  /*construct the twice time derivative of the rotation matrix to be tracked: R_c_ddot = [b1_c_ddot b2_c_ddot b3_c_ddot]
  */
  matrix* R_c_ddot = newMatrix(3,3);
  matrix* R_c_ddot_23 = newMatrix(3,2);
  matrixConcatenation(b2_c_ddot, b3_c_ddot, R_c_ddot_23);
  matrixConcatenation(b1_c_ddot, R_c_ddot_23, R_c_ddot);
  deleteMatrix(R_c_ddot_23); // remove the temp matrix
  deleteMatrix(b1_c_ddot); // remove temp matrix
  deleteMatrix(b2_c_ddot); // remove temp matrix
  deleteMatrix(b3_c_ddot); // remove temp matrix

  /* compute omega_c and omega_c_dot
  Omega_c      = vee(R_c'*R_c_dot);
  Omega_c_dot  = vee(R_c'*R_c_ddot - hat(Omega_c)*hat(Omega_c));
  */
  matrix* R_cT    = newMatrix(3,1);
  matrix* Omega_c = newMatrix(3,1);
  matrix* Omega_c_dot = newMatrix(3,1);
  matrix* Omega_c_dot_hat = newMatrix(3,3);
  matrix* Omega_c_hat  = newMatrix(3,3);

  transposeMatrix(R_c, R_cT); // R_cT is used multiple times. Compute onces.
  // Omega_c
  vee(returnProductMatrix(R_cT,R_c_dot),Omega_c);

  hat(Omega_c, Omega_c_hat); // hat(Omega_c) is used below twice. Compute once.
  subtractMatrix(returnProductMatrix(R_cT,R_c_ddot),returnProductMatrix(Omega_c_hat,Omega_c_hat), Omega_c_dot_hat);
  // Omega_c_dot
  vee(Omega_c_dot_hat, Omega_c_dot);

  /* proper error definition in SO3 and in its tangent
  compute error_R and error_Omega
  error_R     = (1/2)*vee(R_c.'*R - R.'*R_c);
  error_Omega = Omega - R.'*R_c*Omega_c;
  */
  matrix* error_R = newMatrix(3,1);
  matrix* error_R_hat = newMatrix(3,3);
  matrix* error_Omega = newMatrix(3,1);
  matrix* RTRc = newMatrix(3,3);
  matrix* RcTR = newMatrix(3,3);
  matrix* RTRcOmegaC = newMatrix(3,1);

  productMatrix(returnTransposedMatrix(R),R_c,RTRc);
  productMatrix(returnTransposedMatrix(R_c),R,RcTR);
  subtractMatrix(RcTR,RTRc,error_R_hat);

  productScalarMatrix(0.5, returnVee(error_R_hat),error_R);
  productMatrix(RTRc, Omega_c, RTRcOmegaC);

  subtractMatrix(rotVel, RTRcOmegaC, error_Omega);

  /* finally the control torques
    M = -kR*error_R - kOmega*error_Omega + vec_cross(Omega, J*Omega) ...
    - J*(hat(Omega)*R.'*R_c*Omega_c - R.'*R_c*Omega_c_dot);
  */
  sum5Matrix(returnProductMatrix(returnNegMatrix(se3->kR), error_R),
             returnProductMatrix(returnNegMatrix(se3->kOmega), error_Omega),
             returnCrossProduct3DVec(rotVel, returnProductMatrix(se3->J_kgm2,rotVel)),
             returnNegMatrix(returnProductMatrix(se3->J_kgm2,returnProductMatrix(returnHat(rotVel),RTRcOmegaC))),
             returnProductMatrix(RTRc, Omega_c_dot),
             se3->ctrlMoments_Nm);

  // free memory: delete all "explicitly created" matrices at the end
  deleteMatrix(error_pos);
  deleteMatrix(error_vel);
  deleteMatrix(error_acc);
  deleteMatrix(error_jerk);
  deleteMatrix(error_R);
  deleteMatrix(error_R_hat);
  deleteMatrix(error_Omega);
  deleteMatrix(RTRc);
  deleteMatrix(RcTR);
  deleteMatrix(Omega_c_hat);
  deleteMatrix(Omega_c_dot_hat);
  deleteMatrix(RTRcOmegaC);

  // IMPORTANT: returnX functions create new matrices inside their functions (see matrix.c)
  // which are NOT deleted. This possibly creates excessive usage of memory for everytime calling a returnX function.
  // I have implemented returnX function for better coding style. For better memory allocation this might need another revision.
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

void updateIDAPBCCtrl(IDAPBCCtrl *idapbc, matrix* vel,
                      matrix* rotMat, matrix* eul_rad,
                      matrix* omega_rps, matrix* eul_des_rad,
                      matrix* wrench_ext)
{
  // source [1]: https://hal.laas.fr/hal-01964753/file/2019b-YueSecBueFra-preprint.pdf
  // source [2]: https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf

  /* This controller is developed for controlling the physical interaction behavior of aerial robots, which are
     multirotor platformms with n number of actuators, where all actuators face upwards. This means underactuated systems
     where 4 control inputs control the physically interacting aircraft in 6 dimensional space (3 position and 3 rotation).

     Control consists of 4 parts:
     0 - precompensating term for turning a rigid body dynamics model of a multirotor into a Port-Hamiltonian (PH) system
     1 - energy shaping
     2 - damping injection
     3 - external disturbance compensation
  */

  /* 0. Compute the precompensating control input
        This term turns multirotor rigid body dynamics (rotational part) into
        a port-hamiltonian model as a whole (both translational and rotational), as shown in eq. (10).

        Important: in [1] this is presented with eq. (8), because this equation explicitly shows the relations.
        Although it brings mathematical clarity, we do not need to implement eq. (8); in fact we should not.

        What we implement is the following (rotational dynamics precompensation)

        u_r_precomp_Nm = w x Jw - kd * eta_dot

        Notice: above equation does two things:
        1- it cancels the nonlinear terms,
        2- assumes tau_ext = 0

        Together this term does the same job as eq. (8) of [1].

        Assuming tau_ext = 0 in precomensation is a valid assumption, since the dissipating term kd*eta_dot provides local stability.
        Meaning, under bounded external disturbances and sufficiently high kd term (but low enough for not amplifiying noises coming from eta_dot)
        we can exclude using tau_ext in the control input computation.

        Following gives the above presented simpler version of the precomensation control input term.

        The implementation will result in the following:

        u_r_Nm = u_r_precomp_Nm + u_r_bar_Nm

        u_r_Nm is the final control torque input to the multirotor (rotational dynamics). We will store it at the end in idapbc->ctrlMoments_Nm
        u_r_precomp_Nm is the simplified version of eq. (8) as explained above
        u_r_bar_Nm is the novelty of [1], which will bring all the energy shaping control terms.
  */

    matrix* u_r_precomp_Nm = newMatrix(3,1);
    matrix* omegaXJomega = newMatrix(3,1);
    matrix* eta_dot_rps = newMatrix(3,1);

    bodyRates2EulerRates(eul_rad, omega_rps, eta_dot_rps); // we have now euler rates.

    crossProduct3DVec(omega_rps, returnProductMatrix(idapbc->J_kgm2, omega_rps), omegaXJomega);
    subtractMatrix(omegaXJomega, returnProductMatrix(idapbc->KD_eta, eta_dot_rps), u_r_precomp_Nm);

    // free memory where you can
    deleteMatrix(omegaXJomega);

    // now let us compute the rest of the ida-pbc controller step by step.

    /* 1. Energy Shaping. See Sec.2.3.1 of [1]. See eq. (20).
          Idea: Physical systems (in this example mechanical ones) can be seen as energy storing and exchanging systems.
                They store kinetic and potential energies (other forms of energies are neglected), and they change it from one to another
                (kinetic to potential and vice versa)
                depending on their state and external disturbances. In PH systems we imagine this exchange happening via "ports", which
                do not consume any additional energy during exchange. Hence the name "Port-Hamiltonian", where the total energy of the system is called
                Hamiltonian. In this part of the controller we will focus only on the "pure energy exchange without loss". Energy losses due to dissipation
                e.g. friction, damping, etc are considered in the second step of the controller.

          First thing you should notice: If you compute the partial derivatives yourself with a
          piece of paper and pancil you will see that the control thrust is zero. This makes sense for the following reasons:

          - Energy shaping part of the controller does computations under perfect vacuum (Hamiltonian is sum of potential and kineatic energies)
          and dissipations are left to the second stage. In vacuum, two different masses accelerate the same under the same gravitational effect,
          hence original vs desired mass does not make any difference. This is the reason we have the mass scaling terms in the equations.
          - The choice of desired hamiltonian in eq. (16) and (21) introduces only the gravitational potential energy in the translational dimensions.

          Hence only contribution of this part will be to the rotational dynamics.

          In [1] the desired rotational potential energy is chosen as in (22), which satisfies the PDE solution requirements
          (so caled matching equations) explained in eq. (4.21-4.25) of [2].

          Also, we will compute pseudo inverse of the control input matrix explicitly for reducing computational complexities.
          For that reason we need to make a choice for the multirotor platform design already.
          Here we will decide for an n-actuator design, where ALL actuators are facing upwards w.r.t. the body of the multirotor
          platform (could be octo, quad, hexa, etc). For different designs (tilted rotors etc) this part needs to be changed.
    */

   // compute the pseudo inverse of the G matrix explicitly for efficiency. See in [1] eq. (4) and the sentece after.
   matrix* G_cross = newMatrix(4,6);
   /* G_cross is the pseudo inverse of G matrix, where
           _         _
      G = | -R*e3  0  | and G_cross = (G^T*G)^-1*G^T
          |_ 0     I _|

      where R*e3 is the third column of the rotationa matrix [R31 R32 R33]^T and I is a 3x3 identity matrix.
      If you do the computations explicitly, you will find the following:

      r3_square = R31^2 + R32^2 + R33^2
                  _                                                                       _
                 |  -R31/r3_square    -R32/r3_square     -R33/r3_square      0    0    0   |
                 |          0                0                0              1    0    0   |
      G_cross =  |          0                0                0              0    1    0   |
                 |_         0                0                0              0    0    1  _|
   */
   float r3_square = ELEM(rotMat,3,1)*ELEM(rotMat,3,1) + ELEM(rotMat,3,2)*ELEM(rotMat,3,2) + ELEM(rotMat,3,3)*ELEM(rotMat,3,3);
   setMatrixElement(G_cross,1,1,-ELEM(rotMat,3,1)/r3_square);
   setMatrixElement(G_cross,1,2,-ELEM(rotMat,3,2)/r3_square);
   setMatrixElement(G_cross,1,3,-ELEM(rotMat,3,3)/r3_square);
   setMatrixElement(G_cross,2,4, 1.0);
   setMatrixElement(G_cross,3,5, 1.0);
   setMatrixElement(G_cross,4,6, 1.0);
   // u_es = G_cross * f_es, so f_es is the partial derivative terms of eq. (22), here written explicitly after hand computations.
   matrix* u_es = newMatrix(4,1);
   matrix* f_es = newMatrix(6,1);
   /* f_es = parder(H,q) - M*M_d^-1 * parder(Hd,q)
                                          _             _
             [0, 0, 0, -mg, 0, 0, 0]^T - | m/md    0     |[0, 0, -m_dg, kp_roll*roll_err, kp_roll*pitch_err, kp_roll*yaw_err]^T
                                         |_ 0    J_d^-1 _|

            = [0, 0, 0, J_d^-1 * KP_eta * eta_error]
  */
   // euler angle errors
   matrix* eta_error = newMatrix(3,1);
   subtractMatrix(eul_des_rad, eul_rad, eta_error);
   // first 3 elements of f_es are zero because newMatrix assigns all elements zero in the beginning. So let us set the last three terms.
   // IMPORTANT: assuming J_des_inv_kgm2 and KP_des_eta are both diagonal matrices (otherwise matrix multiplication should be written)
   setMatrixElement(f_es, 4, 1, ELEM(idapbc->J_des_inv_kgm2, 1, 1)* ELEM(idapbc->KP_des_eta,1, 1) * ELEM(eta_error, 1, 1));
   setMatrixElement(f_es, 5, 1, ELEM(idapbc->J_des_inv_kgm2, 2, 2)* ELEM(idapbc->KP_des_eta,2, 2) * ELEM(eta_error, 2, 1));
   setMatrixElement(f_es, 6, 1, ELEM(idapbc->J_des_inv_kgm2, 3, 3)* ELEM(idapbc->KP_des_eta,3, 3) * ELEM(eta_error, 3, 1));
   // compute u_es
   productMatrix(G_cross,f_es,u_es);
   /* Concluding remark for u_es:
   The explicit implementation shows that the choice of the desired energy functions (hence the physical behavior) affects the structure of the energy
   shaping controller dramatically. Having the desired Hamiltonian very similar to the original one (and hence in the class of standard mechanical systems)
   brings simplicity in implementation. Finally, u_es acts like a torsional spring, where its parameters are tuned for a desired moment of inertia behavior.
   More exoctic desired energy funcctions can be choosen for different goals, which would change u_es. In that case, go to [1] and [2] for the implicit
   computations of u_es, so that you can create your own explicit version of it before implementing for more efficient computation.
   */
   // free memory
   deleteMatrix(eta_error);
   deleteMatrix(f_es);
   deleteMatrix(G_cross);

   /* 2. Damping Injection.
      This part of the controller regulates the way system dissipates its energy (no more pure energy exchange).
      Dissipation happens all the time, due to friction, damping, etc, which reduces the total energy of the system stored as kinetic and potential
      and usually results as "loss of kinetic + potential = Hamiltonian" in terms of heat.

      Although dissipation results in loss of useful energy, it also increases the stability of the system. Imagine a brake of a car or a parachute.
      A proper and nonzero damping injection puts a negative definite upper bound to the time derivative of a Lyapunov candidate,
      which turns an unstable (or even stable) system to an assymptotically stable one.
      See Section C.2 of [2] for more details for the stability theory.

      In this part we control the dissipation behavior of the newly shaped physics using damping injection method.
   */
  // eq. (24) and the third line of eq. (29) of [1] show the damping injection control input u_di
  /*
    u_di = -Kv*G^T*M^-T*M_d^T*parder(Hd,p_bar)

    Let's explain the parts of this term. Assuming that J_d is diagonal

    parder(Hd,p_bar) = M_d^-1*p_bar = M_d^-1*M_d*M^-1*p = M^-1*p = q_dot (see [1] for the terminology. q_dot is the time derivative of the 6x1 states)

    Kv = idapbc->KV (computed in the init fcn)

    G^T*M^-T*M_d^T*q_dot = [-md/m*(R31*vx + R32*vy + R33*vz), Jd_x*roll_dot, Jd_y*pitch_dot, Jd_z*yaw_dot]^T

    u_di = [kT*md/m*(R31*vx + R32*vy + R33*vz), -KR_x*Jd_x*roll_dot, -KR_y*Jd_y*pitch_dot, -KR_z*Jd_z*yaw_dot]^T

  */
  matrix* u_di = newMatrix(4,1);
  setMatrixElement(u_di,1,1, (ELEM(idapbc->KV,1,1)*idapbc->mass_des_kg/idapbc->mass_kg) *
                             (ELEM(rotMat,3,1)*ELEM(vel,1,1) + ELEM(rotMat,3,2)*ELEM(vel,2,1) + ELEM(rotMat,3,3)*ELEM(vel,3,1)));
  setMatrixElement(u_di,2,1, -ELEM(idapbc->KV,2,2)*ELEM(idapbc->J_des_kgm2,1,1)*ELEM(eta_dot_rps,1,1));
  setMatrixElement(u_di,3,1, -ELEM(idapbc->KV,3,3)*ELEM(idapbc->J_des_kgm2,2,2)*ELEM(eta_dot_rps,2,1));
  setMatrixElement(u_di,4,1, -ELEM(idapbc->KV,4,4)*ELEM(idapbc->J_des_kgm2,3,3)*ELEM(eta_dot_rps,3,1));

  /* 3. External Wrench Compensation
     In step 1 and step 2 above we have computed the control inputs that changes the physical properties of the system from the original one
     to a desired one. Once they are applied, system will behave differently.

     In order to capture the correct reaction of the desired sytem to the external forces and torques (if they are measurable or estimated),
     we apply the following control input.

     For more details, see eq. (27) of [1].

     In this implementation we assume the same things for G_cross and the desired moment of inertia is a diagonal matrix.
     Consider w_ext = [f_ext_x, f_ext_y, f_ext_z, tau_ext_x, tau_ext_y, tau_ext_z]^T as the 6x1 external wrench.
     In this case the long computation

     u_w = G_cross * M*M_d^-1 * w_ext - G_cross*w_ext = (G_cross*M*M_d^-1 - G_cross)*w_ext

     simplifies to:

                 _                                                                         _
                |  (f_ext_x * R31 + f_ext_y * R32 + f_ext_z * R33) * (1-m/m_d) / r3_square  |
    u_w =       |                       tau_ext_x  * (J_d_xx^-1 - 1)                        |
                |                       tau_ext_y  * (J_d_yy^-1 - 1)                        |
                |_                      tau_ext_z  * (J_d_zz^-1 - 1)                       _|


    with

    r3_square = R31^2 + R32^2 + R33^2
  */
  matrix* u_w = newMatrix(4,1);
  setMatrixElement(u_w,1,1, (ELEM(wrench_ext,1,1)*ELEM(rotMat,3,1) + ELEM(wrench_ext,2,1)*ELEM(rotMat,3,2) + ELEM(wrench_ext,3,1)*ELEM(rotMat,3,3))
                             * (1-idapbc->mass_kg / idapbc->mass_des_kg) / r3_square);
  setMatrixElement(u_w,2,1, ELEM(wrench_ext,4,1) * (ELEM(idapbc->J_des_inv_kgm2,1,1)-1));
  setMatrixElement(u_w,3,1, ELEM(wrench_ext,5,1) * (ELEM(idapbc->J_des_inv_kgm2,2,2)-1));
  setMatrixElement(u_w,4,1, ELEM(wrench_ext,6,1) * (ELEM(idapbc->J_des_inv_kgm2,3,3)-1));

  /* Total energy shaping control inout for a desired physical behavior then becomes (see eq. 29 of [1]):
    u_i = u_es + u_di + u_w
  except the outer loop terms u_o that can be added here, e.g. for position control etc.
  */
  matrix* u_i = newMatrix(4,1);
  sum3Matrix(u_es, u_di, u_w, u_i);

  /* Output of the correct controller is the sum of the pre-compensating terms + u_i
  */
  matrix* u_pre = newMatrix(4,1);
  matrix* u_idapbc = newMatrix(4,1);
  // first element is thrust, where we do not do any precompensation (hence says zero). We did precompensation above for the rotational dynamics already.
  setMatrixElement(u_pre,2,1, ELEM(u_r_precomp_Nm,1,1));
  setMatrixElement(u_pre,3,1, ELEM(u_r_precomp_Nm,2,1));
  setMatrixElement(u_pre,4,1, ELEM(u_r_precomp_Nm,3,1));
  // u_idapbc = u_pre + u_i
  sumMatrix(u_pre, u_i, u_idapbc);
  // now set the outputs correctly
  setMatrixElement(idapbc->ctrlThrust_N,1,1, ELEM(u_idapbc,1,1));
  setMatrixElement(idapbc->ctrlMoments_Nm,1,1, ELEM(u_idapbc,2,1));
  setMatrixElement(idapbc->ctrlMoments_Nm,2,1, ELEM(u_idapbc,3,1));
  setMatrixElement(idapbc->ctrlMoments_Nm,3,1, ELEM(u_idapbc,4,1));

  // free memory
  deleteMatrix(u_r_precomp_Nm);
  deleteMatrix(u_pre);
  deleteMatrix(u_i);
  deleteMatrix(u_idapbc);
  deleteMatrix(u_es);
  deleteMatrix(u_di);
  deleteMatrix(u_w);
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