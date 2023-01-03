/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics functions
 * @addtogroup PHYSICS
 **/

#include "parameters.h"
#include "physics.h"
#include <math.h> // added for computations like sqrt
#include <string.h> // added for memset operations

/*
BODY FRAME      : NED convention
INERTIAL FRAME  : NED convention

*/


/*define a particle as a point object*/
pointObject g_physicsPointObj;

/*define its states*/
states g_phsicsPointStates;

/* time of simulation*/
float g_time_s;



/** @brief Convert quaternions to a 3x3 rotation matrix*/

void quaternionToRotMatrix(matrix *R, quaternion q) 
{
    /*Source: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm */
    float r11 = 1 - 2*q.y*q.y - 2*q.z*q.z;
    float r12 = 2*q.x*q.y - 2*q.z*q.w;
    float r13 = 2*q.x*q.z + 2*q.y*q.w;

    float r21 = 2*q.x*q.y + 2*q.z*q.w;
    float r22 = 1 - 2*q.x*q.x - 2*q.z*q.z;
    float r23 = 2*q.y*q.z - 2*q.x*q.w;

    float r31 = 2*q.x*q.z - 2*q.y*q.w;
    float r32 = 2*q.y*q.z + 2*q.x*q.w;
    float r33 = 1 - 2*q.x*q.x - 2*q.y*q.y;

    R = newMatrix(3, 3);

    setMatrixElement(R, 1, 1, r11);
    setMatrixElement(R, 1, 2, r12);
    setMatrixElement(R, 1, 3, r13);
    setMatrixElement(R, 2, 1, r21);
    setMatrixElement(R, 2, 2, r22);
    setMatrixElement(R, 2, 3, r23);
    setMatrixElement(R, 3, 1, r31);
    setMatrixElement(R, 3, 2, r32);
    setMatrixElement(R, 3, 3, r33);
}

/** @brief Convert quaternions to Euler angles in radians*/

void quaternionToEuler(euler *euler_r, quaternion q)
// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
{
    //Compute the roll angle
    float sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler_r->roll = atan2(sinr_cosp, cosr_cosp);

    //Compute the pitch angle
    float sinp = 1 + 2.0 * (q.w * q.x - q.y * q.z);
    float cosp = 1 - 2.0 * (q.y * q.x - q.y * q.y);
    euler_r->pitch= 2 * atan2(sinp, cosp) - M_PI / 2;

    /*
    // alternative
    if (fabs(sinp) >= 1)
        euler_r->pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler_r->pitch = asin(sinp);
    */

    //Compute the yaw angle
    float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler_r->yaw = atan2(siny_cosp, cosy_cosp);
}


/** @brief Update the quaternion using quaternion kinematics and angular velocities*/

void updateQuaternion(quaternion *q, vector3 rotVelBody_rps, float dt)
{
    /*Source for the simple math: https://ahrs.readthedocs.io/en/latest/filters/angular.html*/
    // Compute the derivative of the quaternion
    quaternion q_dot;
    q_dot.w = -0.5 * (rotVelBody_rps[0] * q->x + rotVelBody_rps[1] * q->y + rotVelBody_rps[2] * q->z);
    q_dot.x =  0.5 * (rotVelBody_rps[0] * q->w + rotVelBody_rps[2] * q->y - rotVelBody_rps[1] * q->z);
    q_dot.y =  0.5 * (rotVelBody_rps[1] * q->w - rotVelBody_rps[2] * q->x + rotVelBody_rps[0] * q->z);
    q_dot.z =  0.5 * (rotVelBody_rps[2] * q->w + rotVelBody_rps[1] * q->x - rotVelBody_rps[0] * q->y);

    // Update the quaternion using the derivative and the time step
    q->w = q->w + q_dot.w * dt;
    q->x = q->x + q_dot.x * dt;
    q->y = q->y + q_dot.y * dt;
    q->z = q->z + q_dot.z * dt;

    // Normalize the quaternion to ensure it remains a unit quaternion
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w = q->w / norm;
    q->x = q->x / norm;
    q->y = q->y / norm;
    q->z = q->z / norm;
}

/** @brief Update the translational and rotational motion in an inertial frame*/

void translationalDynamics(states* ps, vector3 extForces_N)
{

    /* start simple, falling point mass in a quadratic drag model */
    vector3 dragForce;
    // write the drag forces
    for (int i=0; i<3; i++)
    {
        dragForce[i] = g_physicsPointObj.dragCoeff*ps->trState.velInertial_mps[i]*ps->trState.velInertial_mps[i];
    }

    // write down the model that computes the accelerations
    ps->trState.accInertial_mps2[0] = (extForces_N[0]-dragForce[0])/g_physicsPointObj.mass_kg;
    ps->trState.accInertial_mps2[1] = (extForces_N[1]-dragForce[1])/g_physicsPointObj.mass_kg;
    ps->trState.accInertial_mps2[2] = ENV_GRAVITY_MPS2 + ((extForces_N[2]-dragForce[2])/g_physicsPointObj.mass_kg);

    // that's it.

/*
    // Let's implement the above logic using the matrix libraries.
    // You will see how ridiculously long it is.

    matrix* dragMatrix              =   newMatrix(3, 3);
    matrix* invMassVector           =   newMatrix(3, 1);
    matrix* velocitySquaredVector   =   newMatrix(3, 1);
    matrix* gravityVector           =   newMatrix(3, 1);
    matrix* extForceVector          =   newMatrix(3, 1);

    setMatrixElement(dragMatrix, 1, 1, g_physicsPointObj.dragCoeff);
    setMatrixElement(dragMatrix, 2, 2, g_physicsPointObj.dragCoeff);
    setMatrixElement(dragMatrix, 3, 3, g_physicsPointObj.dragCoeff);

    setMatrixElement(invMassVector, 1, 1, 1.0f/g_physicsPointObj.mass_kg);
    setMatrixElement(invMassVector, 2, 1, 1.0f/g_physicsPointObj.mass_kg);
    setMatrixElement(invMassVector, 3, 1, 1.0f/g_physicsPointObj.mass_kg);

    setMatrixElement(velocitySquaredVector, 1, 1, ps->trState.velInertial_mps[0]*ps->trState.velInertial_mps[0]);
    setMatrixElement(velocitySquaredVector, 2, 1, ps->trState.velInertial_mps[1]*ps->trState.velInertial_mps[1]);
    setMatrixElement(velocitySquaredVector, 3, 1, ps->trState.velInertial_mps[2]*ps->trState.velInertial_mps[2]);

    setMatrixElement(extForceVector, 1, 1, extForces_N[0]);
    setMatrixElement(extForceVector, 2, 1, extForces_N[1]);
    setMatrixElement(extForceVector, 3, 1, extForces_N[2]);

    setMatrixElement(gravityVector, 1, 1, 0.0f);
    setMatrixElement(gravityVector, 2, 1, 0.0f);
    setMatrixElement(gravityVector, 3, 1, ENV_GRAVITY_MPS2);

    // resulting acc vector
    matrix* accVector   = newMatrix(3,1);
    // net forces. It will be overwritten with every computation in the following:
    matrix* netForces   = newMatrix(3,1);
    // 1) compute the quadratic drag forces as drag*vel^2 and put it into the net forces
    productMatrix(dragMatrix, velocitySquaredVector, netForces);
    // 2) subtract the drag forces from the external forces and put it into the net forces
    subtractMatrix(extForceVector, netForces, netForces);
    // 3) divide the net forces with the mass and put it to the acc vector
    productMatrix(invMassVector,netForces,accVector);
    // 4) add the gravitational acc terms
    sumMatrix(accVector,gravityVector,accVector);

    // Put the acc vector to the struct
    ps->trState.accInertial_mps2[0] = accVector->data[0];
    ps->trState.accInertial_mps2[1] = accVector->data[1];
    ps->trState.accInertial_mps2[2] = accVector->data[2];
*/
}

void rotationalDynamics(states* ps, vector3 extMoments_Nm)
{
    // source: https://en.wikipedia.org/wiki/Rigid_body_dynamics
    matrix* omega           = newMatrix(3,1);
    matrix* Jomega          = newMatrix(3,1);
    matrix* omegaMoments    = newMatrix(3,1);
    matrix* netMoments      = newMatrix(3,1);
    matrix* externalMoments = newMatrix(3,1);
    matrix* rotAcc          = newMatrix(3,1);
    matrix* inverseInertia  = newMatrix(3,3);

    // set the rotational velocities
    setMatrixElement(omega,1,1,ps->rtState.rotVelBody_rps[0]);
    setMatrixElement(omega,2,1,ps->rtState.rotVelBody_rps[1]);
    setMatrixElement(omega,3,1,ps->rtState.rotVelBody_rps[2]);

    // set the external moments
    setMatrixElement(externalMoments,1,1,extMoments_Nm[0]);
    setMatrixElement(externalMoments,2,1,extMoments_Nm[1]);
    setMatrixElement(externalMoments,3,1,extMoments_Nm[2]);
    // w x Jw term
    productMatrix(g_physicsPointObj.I_kgm2, omega, Jomega);
    crossProduct3DVec(omega, Jomega, omegaMoments);
    // tau - w x Jw term, which equates itself to J.omega_dot
    subtractMatrix(externalMoments, omegaMoments, netMoments);
    // invert inertia matrix, which is diagonal
    // TODO: consider doing this in init and once.
    invertDiagonalMatrix(g_physicsPointObj.I_kgm2, inverseInertia);
    // compute rotational accelerations (omega_dot)
    productMatrix(inverseInertia,netMoments,rotAcc);

    // now put the rot acc to the state struct
    getMatrixElement(rotAcc, 1, 1, &ps->rtState.rotAccBody_rps2[0]);
    getMatrixElement(rotAcc, 2, 1, &ps->rtState.rotAccBody_rps2[1]);
    getMatrixElement(rotAcc, 3, 1, &ps->rtState.rotAccBody_rps2[2]);
}

/** @brief Update the translational and rotational motion in an inertial frame*/

void update_motion_states(states *ps, float dt_s)
{
    // Update translational motion (positions)
    for (int i=0; i<3; i++)
    {
        ps->trState.pos_Inertial_m[i] = ps->trState.pos_Inertial_m[i] + ps->trState.velInertial_mps[i]*dt_s + 0.5f*ps->trState.accInertial_mps2[i]*dt_s*dt_s;
    }
    
    // Update translational motion (velocities)
    for (int i=0; i<3; i++)
    {
        ps->trState.velInertial_mps[i] = ps->trState.velInertial_mps[i] + ps->trState.accInertial_mps2[i]*dt_s;
    }

    // Update rotational motion (velocities)
    for (int i=0; i<3; i++)
    {
        ps->rtState.rotVelBody_rps[i] = ps->rtState.rotVelBody_rps[i] + ps->rtState.rotAccBody_rps2[i]*dt_s;
    }

    // Update attitude/orientation/pose (quaternions)
    updateQuaternion(&ps->rtState.q, ps->rtState.rotVelBody_rps, dt_s);

    // Convert quaternion to euler in radians (redundant definition, but for later visibility)
    quaternionToEuler(&ps->rtState.euler_r, ps->rtState.q);
}


/** @brief Update Physics Function */
void physicsUpdate(states* ps, vector3 extForces_N, vector3 extMoments_Nm, float dt_s)
{
    // translational dynamics
    translationalDynamics(ps, extForces_N);
    // rotational dynamics
    rotationalDynamics(ps, extMoments_Nm);
    // update the motion (integration with time)
    update_motion_states(ps, dt_s);
}

/** @brief Initialize Physics */
void physicsInit()
{
    /* SET PARAMETERS*/

    /*simulation start time*/
    g_time_s = 0.0f;
    /*define its mass*/
    g_physicsPointObj.mass_kg    = POINT_MASS_KG;
    /*define its drag coeff*/
    g_physicsPointObj.dragCoeff  = POINT_DRAGCOEFF;
    /*define moment of inertia as a 3x3 matrix*/
    g_physicsPointObj.I_kgm2 = newMatrix(3, 3);
    /*set the moment of inertia terms*/
    setMatrixElement(g_physicsPointObj.I_kgm2, 1, 1, POINT_I_XX_KGM2);//Ixx
    setMatrixElement(g_physicsPointObj.I_kgm2, 2, 2, POINT_I_YY_KGM2);//IYY
    setMatrixElement(g_physicsPointObj.I_kgm2, 3, 3, POINT_I_ZZ_KGM2);//IZZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 1, 2, POINT_I_XY_KGM2);//IXY
    setMatrixElement(g_physicsPointObj.I_kgm2, 2, 1, POINT_I_XY_KGM2);//IYX=IXY
    setMatrixElement(g_physicsPointObj.I_kgm2, 1, 3, POINT_I_XZ_KGM2);//IXZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 3, 1, POINT_I_XZ_KGM2);//IZX=IXZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 2, 3, POINT_I_YZ_KGM2);//IYZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 3, 2, POINT_I_YZ_KGM2);//IZY=IYZ

    /* SET STATES */
    // all initial states are zero
    memset(&g_phsicsPointStates, 0, sizeof(g_phsicsPointStates));
    // but this would break the quaternion: its scalar part cannot be zero
    g_phsicsPointStates.rtState.q.w = 1.0f;
}