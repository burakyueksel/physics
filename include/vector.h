#ifndef VECTOR_H_
#define VECTOR_H_

/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Vector libraries for physics
 * @addtogroup VECTOR
 **/

#include <stdio.h> // for print function
#include "matrix.h" // for converting vectors to matrix form

typedef float vector;
typedef vector vector2[2];
typedef vector vector3[3];
typedef vector vector4[4];

typedef struct
{
  float w; // scalar part
  float x;
  float y;
  float z;
} quaternion;

typedef struct vector
{
  float roll;
  float pitch;
  float yaw;
} euler;


/*
********************************************
** FUNCTION DECLERATIONS
********************************************
*/
void crossProduct(vector3 a, vector3 b, vector3 axb);
void quaternionProduct(quaternion q1, quaternion q2, quaternion* q1xq2);
void quaternionInverse(quaternion q, quaternion* q_inverse);
float quaternionZAlignNorm(quaternion q);
int getQuaternionVectorPart(quaternion q, matrix* qv);
int bodyRates2EulerRates(matrix* eul_rad, matrix* bodyRates_rps, matrix* eulerRates_rps);
int eulerRates2BodyRates(matrix* eul_rad, matrix* eulerRates_rps, matrix* bodyRates_rps);

#endif // VECTOR_H_