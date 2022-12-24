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
  float mass;
  matrix momentOfInertia;
} rigidBody_mass_inertia;



#endif // PHYSICS_H_