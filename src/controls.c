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
  // source 1: Conference paper: https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf
  // source 2: Journal paper: https://journals.sagepub.com/doi/full/10.1177/0278364919835605
  // source 3: Ch 4 of thesis https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf

  /*precompensation controller gains*/
  idapbc->KD_eta = newMatrix(3,3);
  float omega = 0.3;
  float damping = 1.0;
  //float kp_eta = omega*omega // to be used for the desired rotational potential energy
  float kd_eta = 2*omega*damping;
  setMatrixElement(idapbc->KD_eta,  1,  1,  kd_eta);
  setMatrixElement(idapbc->KD_eta,  2,  2,  kd_eta);
  setMatrixElement(idapbc->KD_eta,  3,  3,  kd_eta);
  /*damping injection gains*/
  idapbc->kT_bar = 1.0f;
  idapbc->KR_bar = copyMatrix(idapbc->KD_eta);
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

void updateIDAPBCCtrl(IDAPBCCtrl *idapbc, matrix* eul_rad, matrix* omega_rps)
{
  // source [1]: https://hal.laas.fr/hal-01964753/file/2019b-YueSecBueFra-preprint.pdf

  /*compute the precompensating control input
    which turns multirotor rigid body dynamics (rotational part) into
    a port-hamiltonian model as a whole (both translational and rotational), as shown in eq. (10).

    Important: in [1] this is presented with eq. (8), because this equation explicitly shows the relations.
    Although it brings mathematical clarity, we do not need to implement eq. (8); in fact we should not.

    What we implement is the following (rotational dynamics precompensation)

    u_r_bar_Nm = w x Jw - kd * eta_dot

    Notice: above equation does two things:
    1- it cancels the nonlinear terms,
    2- assumes tau_ext = 0

    Together this term does the same job as eq. (8) of [1].

    Assuming tau_ext = 0 in precomensation is a valid assumption, since the dissipating term kd*eta_dot provides local stability.
    Meaning, under bounded external disturbances and sufficiently high kd term (but low enough for not amplifiying noises coming from eta_dot)
    we can exclude using tau_ext in the control input computation.

    Following gives the above presented simpler version of the precomensation control input term.
  */

    matrix* u_r_bar_Nm = newMatrix(3,1);
    matrix* omegaXJomega = newMatrix(3,1);
    matrix* eta_dot_rps = newMatrix(3,1);

    bodyRates2EulerRates(eul_rad, omega_rps, eta_dot_rps); // we have now euler rates.

    crossProduct3DVec(omega_rps, returnProductMatrix(idapbc->J_kgm2, omega_rps), omegaXJomega);
    subtractMatrix(omegaXJomega, returnProductMatrix(idapbc->KD_eta, eta_dot_rps), u_r_bar_Nm);
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