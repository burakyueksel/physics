#ifndef PHYSICS_H_
#define PHYICS_H_

/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics libraries
 * @addtogroup PHYSICS
 **/

#include "matrix.h"
#include "vector.h"


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
  euler euler_r;
  quaternion q;
} rotationalStates;

typedef struct
{
  translationalStates trState;
  rotationalStates rtState;
} states;

typedef struct
{
  float liftForce_N;
  float dragForce_N;
  float sideForce_N;
  float rollMoment_Nm;
  float pitchMoment_Nm;
  float yawMoment_Nm;
} aeroForcesMoments;


/*
********************************************
** GLOBALS
********************************************
*/
extern pointObject g_physicsPointObj;

extern states g_phsicsPointStates;

/*
********************************************
** FUNCTION DECLERATIONS
********************************************
*/
void physicsInit();
void physicsUpdate(states* ps, vector3 extForces_N, vector3 extMoments_Nm, float dt_s);


#endif // PHYSICS_H_