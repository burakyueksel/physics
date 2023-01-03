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
#define ENV_GRAVITY_MPS2 9.80665f // https://en.wikipedia.org/wiki/Standard_gravity
#define ENV_R 287.0  // gas constant for air (J/kg/K)
#define ENV_L 0.0065  // temperature lapse rate (K/m)
#define ENV_P0 101325.0  // standard atmospheric pressure at sea level (Pa)

// Barometer parameters
#define BARO_NOISE_SD 10.0  // standard deviation of noise (in Pa)

// Simulation parameters
#define DT_S        0.001f   // discrete time step in seconds
#define T_END_S     20.0f    // end of simulation time

// Control parameters
#define CTRL_PID_OMEGA_RPS  8.0f
#define CTRL_PID_DAMPING    8.0f
#define CTRL_PID_KP         POINT_MASS_KG * CTRL_PID_OMEGA_RPS*CTRL_PID_OMEGA_RPS/ENV_GRAVITY_MPS2
#define CTRL_PID_KD         2.0 * CTRL_PID_OMEGA_RPS * CTRL_PID_DAMPING/ENV_GRAVITY_MPS2
#define CTRL_PID_KI         CTRL_PID_KD
#endif // PARAMETERS_H_