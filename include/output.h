#ifndef OUTPUT_H_
#define OUTPUT_H_

/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Outputting libraries
 * @addtogroup OUTPUT_H_
 **/

#include <stdio.h> // for print function
#include "vector.h"
/*
********************************************
** FUNCTION DECLERATIONS
********************************************
*/
int printVector3(vector3 vec3);
int printVectorQuaternion(quaternion* q);
int printVectorEuler(euler* e);


#endif // OUTPUT_H_