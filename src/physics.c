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

void quaternionToEuler(vector3 *euler_r, quaternion q)
// Heavy WIP!! Not stable.
{
    /*Compute the roll angle*/
    float sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    *euler_r[0] = atan2(sinr_cosp, cosr_cosp);

    /*Compute the pitch angle*/
    float sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        *euler_r[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        *euler_r[1] = asin(sinp);

    /*Compute the yaw angle*/
    float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    *euler_r[3] = atan2(siny_cosp, cosy_cosp);
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

void translationalDynamics(states* ps, vector3 forces_N) 
{
    // start simple, falling mass in vacuum
    ps->trState.accInertial_mps2[0] = forces_N[0]/g_physicsPointObj.mass_kg;
    ps->trState.accInertial_mps2[1] = forces_N[1]/g_physicsPointObj.mass_kg;
    ps->trState.accInertial_mps2[2] = GRAVITY_MPS2 + (forces_N[2]/g_physicsPointObj.mass_kg);
}

/*
void rotationalDynamics(vector3 *rotAccBody_rps2, states pm, vector3 moments_Nm) 
{

}
*/

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
    // Heavily in WIP. Not stable!
    //quaternionToEuler(&pm->rtState.euler_r, pm->rtState.q);
}


/** @brief Update Physics Function */
void physicsUpdate(states* ps, float dt_s)
{
    // external forces
    vector3 extForces_N = {0.0f, 0.0f, 0.0f};
    // translational dynamics
    translationalDynamics(ps, extForces_N);
    //TODO: same for the rotational dynamics

    // update the motion (integration with time)
    update_motion_states(ps, dt_s);

    /*print the time*/
    printf("Current time is:\n");
    printf("% 6.4f ", g_time_s);
    printf("\n");
    /*Print the positions*/
    printf("3D Positions (x,y,z) in meters are:\n");
    //printVector3(ps->trState.pos_Inertial_m); // gives same result as using the global
    printVector3(g_phsicsPointStates.trState.pos_Inertial_m);

    /*Print the velocities*/
    printf("3D Velocities (x,y,z) in m/s are:\n");
    //printVector3(ps->trState.pos_Inertial_m); // gives same result as using the global
    printVector3(g_phsicsPointStates.trState.velInertial_mps);

    /*Print the quaternions*/
    printf("Quaternions:\n");
    printVectorQuaternion(&g_phsicsPointStates.rtState.q);
}
/** @brief Main  Physics Function */
void physicsMain()
{
    // short test
    printf("Mass the Point Particle:\n");
    printf("% 6.2f ", g_physicsPointObj.mass_kg);
    printf("\n");

    printf("Drag Coeff of the Point Particle:\n");
    printf("% 6.2f ", g_physicsPointObj.dragCoeff);
    printf("\n");

    printf("Moment of Inertia of the Point Particle:\n");
    printMatrix(g_physicsPointObj.I_kgm2);

    printf("Quaternions:\n");
    printVectorQuaternion(&g_phsicsPointStates.rtState.q);

/*
    quaternionToEuler(&g_phsicsPointStates.rtState.euler_r, g_phsicsPointStates.rtState.q);
    printf("Euler Angles in rad:\n");
    printVector3(&g_phsicsPointStates.rtState.euler_r);
*/

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