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