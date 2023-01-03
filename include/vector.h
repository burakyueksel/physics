#ifndef VECTOR_H_
#define VECTOR_H_

/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Vector libraries for physics
 * @addtogroup VECTOR
 **/

#include <stdio.h> // for print function

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


#endif // VECTOR_H_