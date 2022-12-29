#ifndef PARAMETERS_H_
#define PARAMETERS_H_

/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics libraries
 * @addtogroup PHYSICS
 **/

// Object parameters
#define POINT_MASS_KG    10.0f //kg
#define POINT_I_XX_KGM2  1.0f
#define POINT_I_YY_KGM2  1.0f
#define POINT_I_ZZ_KGM2  1.0f
#define POINT_I_XY_KGM2  0.0f
#define POINT_I_XZ_KGM2  0.0f
#define POINT_I_YZ_KGM2  0.0f
#define POINT_DRAGCOEFF  0.1f

// Environment parameters
#define GRAVITY_MPS2 9.80665f // https://en.wikipedia.org/wiki/Standard_gravity

// Simulation parameters
#define DT_S        0.001f   // discrete time step in seconds
#define T_END_S     20.0f    // end of simulation time

#endif // PARAMETERS_H_