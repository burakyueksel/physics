#ifndef PHYSICS_H_
#define PHYICS_H_

/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics libraries
 * @addtogroup PHYSICS
 **/

#include "matrix.h"


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

typedef struct
{
  float mass_kg;
  matrix * I_kgm2;
  float dragCoeff;
} pointObject;

typedef struct
{
  vector3 accInertial_mps2;
  vector3 velInertial_mps;
  vector3 pos_Inertial_m;
} translationalStates;

typedef struct
{
  vector3 rotAccBody_rps2;
  vector3 rotVelBody_rps;
  vector3 euler_r;
  quaternion q;
} rotationalStates;

typedef struct
{
  translationalStates trState;
  rotationalStates rtState;
} states;



/*
********************************************
** GLOBALS
********************************************
*/
extern pointObject g_physicsPointObj;

extern states g_phsicsPointStates;

/*
********************************************
** FUNCTIONS
********************************************
*/
extern void physicsMain();
extern void physicsInit();


#endif // PHYSICS_H_