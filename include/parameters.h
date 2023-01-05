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
// https://en.wikipedia.org/wiki/Standard_gravity
// https://community.bosch-sensortec.com/t5/Question-and-answers/How-to-calculate-the-altitude-from-the-pressure-sensor-data/qaq-p/5702?lightbox-message-images-47339=11439i69B5F86B1DCF9625
#define ENV_GRAVITY_MPS2 9.80665f
#define ENV_R 8.3144598   // universal gas constant for air (J/kg/K)
#define ENV_L -0.0065  // temperature lapse rate (K/m) at P0 and T0
#define ENV_P0 101325.0  // standard atmospheric pressure at sea level (Pa)
#define ENV_AIR_MOLAR_MASS 0.0289644 // Molar mass of air (kg/mol)
#define ENV_T0  288.15 // Standard temperature at sea level (K)
#define ENV_ZERO_CELCIUS 273.15 // Zero (0) Celcius in Kelvins (K)

// Barometer parameters
#define BARO_NOISE_SD 3.0  // standard deviation of noise (in Pa)
#define BARO_BIAS 5.0 // bias of noise (in Pa)

// Parameters for the IMU sensor model
#define BIAS_ACCELEROMETER 0.1  // m/s^2
#define BIAS_GYROSCOPE 0.01     // rad/s
#define NOISE_ACCELEROMETER 0.01  // m/s^2
#define NOISE_GYROSCOPE 0.001     // rad/s

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