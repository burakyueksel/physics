/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Vector libraries for physics
 * @addtogroup VECTOR
 **/

#include "vector.h"
#include <math.h> // for sqrtf

void crossProduct(vector3 a, vector3 b, vector3 axb)
{
  axb[0] = a[1]*b[2] - a[2]*b[1];
  axb[1] = a[2]*b[0] - a[0]*b[2];
  axb[2] = a[0]*b[1] - a[1]*b[0];
}

void quaternionProduct(quaternion q1, quaternion q2, quaternion* q1xq2)
{
  // source: https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
  // equations (2) and (3), explicit implementation
  // q1xq2 is like R=R1*R2, where R1 and R2 are two rotation matrices, where each corresponds to q1 and q2, respectively.
  q1xq2->w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
  q1xq2->x = q2.w*q1.x + q2.x*q1.w + q2.y*q1.z - q2.z*q1.y;
  q1xq2->y = q2.w*q1.y - q2.x*q1.z + q2.y*q1.w + q2.z*q1.x;
  q1xq2->z = q2.w*q1.z + q2.x*q1.y - q2.y*q1.x + q2.z*q1.w;
}

void quaternionInverse(quaternion q, quaternion* q_inverse)
  // for q is the quaternion representing orientation from frame F1 to frame F2,
  // q_inverse is the quaternion that represents the orientation from F2 to frame F1.
{
  q_inverse->w = q.w;
  q_inverse->x = -q.x;
  q_inverse->y = -q.y;
  q_inverse->z = -q.z;
}

// computes 1/sqrt(q.w² + q.z²)
float quaternionZAlignNorm(quaternion q)
{
  float norm = q.w*q.w + q.z*q.z;
  // in case q isn not well defined
  // this happens in the following case as an example:
  // q is the quaternion between a desired frame and the current frame
  // z axis of the desired frame is aligned exactly at the opposite direction of the z axis of the current frame
  if (norm<1e-6)
  {
    return 0.001; // a small number.
  }
  else
  {
    return 1/sqrtf(norm);
  }
}

int getQuaternionVectorPart(quaternion q, matrix* qv)
{
  // qv should be a defined 3x1 matrix (rows x cols)
  if (qv->cols != 1 || qv->rows != 3) return 0;
  setMatrixElement(qv,1,1,q.x);
  setMatrixElement(qv,2,1,q.y);
  setMatrixElement(qv,3,1,q.z);
  return 1;
}

int bodyRates2EulerRates(matrix* eul_rad, matrix* bodyRates_rps, matrix* eulerRates_rps)
{
  // source [1]: https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf
  // bodyRates_rps and eulerRates_rps should be defined 3x1 matrices (rows x cols)
  if (bodyRates_rps->cols != 1 || bodyRates_rps->rows != 3 ||
      eulerRates_rps->cols != 1 || eulerRates_rps->rows != 3) return 0; // failure
  // get the angle and the body rates explicitly for better code readability
  float roll = ELEM(eul_rad,1,1);
  float pitch = ELEM(eul_rad,2,1);
  //float yaw = ELEM(eul_rad,3,1); // not used.

  float p = ELEM(bodyRates_rps,1,1);
  float q = ELEM(bodyRates_rps,2,1);
  float r = ELEM(bodyRates_rps,3,1);
  // compute the trigonometric relations once and store
  float sin_roll = sin(roll);
  float cos_roll = cos(roll);
  float tan_pitch = tan(pitch);
  float cos_pitch = cos(pitch);
  // see eq. (2.2) of [1];
  setMatrixElement(eulerRates_rps,1,1, p + q*sin_roll/tan_pitch + r*cos_roll/tan_pitch); //p
  setMatrixElement(eulerRates_rps,2,1,     q*cos_roll           - r*sin_roll);           //q
  setMatrixElement(eulerRates_rps,3,1,   + q*sin_roll/cos_pitch + r*cos_roll/cos_pitch); //r

  return 1; // success.
}

int eulerRates2BodyRates(matrix* eul_rad, matrix* eulerRates_rps, matrix* bodyRates_rps)
{
  // source [1]: https://homepages.laas.fr/afranchi/robotics/sites/default/files/phd-thesis-2017-Yueksel.pdf
  // bodyRates_rps and eulerRates_rps should be defined 3x1 matrices (rows x cols)
  if (bodyRates_rps->cols != 1 || bodyRates_rps->rows != 3 ||
      eulerRates_rps->cols != 1 || eulerRates_rps->rows != 3) return 0; // failure
  // get the angle and the body rates explicitly for better code readability
  float roll = ELEM(eul_rad,1,1);
  float pitch = ELEM(eul_rad,2,1);
  //float yaw = ELEM(eul_rad,3,1); // not used.

  float roll_dot = ELEM(eulerRates_rps,1,1);
  float pitch_dot = ELEM(eulerRates_rps,2,1);
  float yaw_dot = ELEM(eulerRates_rps,3,1);
  // compute the trigonometric relations once and store
  float sin_roll = sin(roll);
  float cos_roll = cos(roll);
  float sin_pitch = sin(pitch);
  float cos_pitch = cos(pitch);
  // see eq. (2.2) of [1];
  setMatrixElement(bodyRates_rps,1,1, roll_dot                        -sin_pitch*yaw_dot);
  setMatrixElement(bodyRates_rps,2,1,           cos_roll*pitch_dot    +sin_roll*cos_pitch*yaw_dot);
  setMatrixElement(bodyRates_rps,3,1,          -sin_roll*pitch_dot    +cos_roll*cos_pitch*yaw_dot);

  return 1; // success.
}