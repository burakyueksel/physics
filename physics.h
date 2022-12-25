#ifndef PHYSICS_H_
#define PHYICS_H_

/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics libraries
 * @addtogroup PHYSICS
 **/

#include "math.h"

typedef struct {
  float mass_kg;
  matrix * I_kgm2;
  float dragCoeff;
} pointObject;



#endif // PHYSICS_H_