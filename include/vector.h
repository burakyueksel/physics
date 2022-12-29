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

/* Prints the 3D vector to stdout.  Returns 0 if successful 
 * and -1 if vector is NULL.
 */
int printVector3(vector3* vec3)
{
  if (!vec3) return -1;
  
  int item;
  for (item = 0; item < 3; item++) 
  {
    // Print the floating-point element with
    //  - either a - if negative or a space if positive
    //  - at least 3 spaces before the .
    //  - precision to the hundredths place
    printf("% 6.2f ", *vec3[item]);
    // separate rows by newlines
    printf("\n");
  }
  return 0;
}

/* Prints the 4D quaternion vector to stdout.  Returns 0 if successful 
 * and -1 if mtx is NULL.
 */
int printVectorQuaternion(quaternion* q)
{
  if (!q) return -1;
  
  int item;
  for (item = 0; item < 4; item++) 
  {
    // Print the floating-point element
    printf("% 6.2f ", *((float*)q + item));
    // separate rows by newlines
    printf("\n");
  }
  return 0;
}

#endif // VECTOR_H_