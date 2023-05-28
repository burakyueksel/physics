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
#define ENV_TEMP_C 20   // temperature of the air outside (C)
#define ENV_AIR_DENSITY 1.225 // air density (kg/m^3)

// Barometer parameters
#define BARO_NOISE_SD 3.0  // standard deviation of noise (in Pa)
#define BARO_BIAS 5.0 // bias of noise (in Pa)

// Parameters for the IMU sensor model
#define BIAS_ACCELEROMETER 0.1  // m/s^2
#define BIAS_GYROSCOPE 0.01     // rad/s
#define NOISE_ACCELEROMETER 0.01  // m/s^2
#define NOISE_GYROSCOPE 0.001     // rad/s

// Simulation parameters
#define REALTIME_DT_S        0.001f   // discrete time step of the reality (simulated) in seconds
#define REALTIME_T_END_S     20.0f    // end of time of reality (simulated) in seconds

// Control parameters
#define CTRL_DT_S           0.005f   // discrete time step of the controller in seconds
#define CTRL_DT_SD_S        0.0002f  // standard deviation in noise in discrete time steps of the controller (no MCU is perfect) in seconds
#define CTRL_PID_OMEGA_RPS  8.0f
#define CTRL_PID_DAMPING    8.0f
#define CTRL_PID_KP         POINT_MASS_KG * CTRL_PID_OMEGA_RPS*CTRL_PID_OMEGA_RPS/ENV_GRAVITY_MPS2
#define CTRL_PID_KD         2.0 * CTRL_PID_OMEGA_RPS * CTRL_PID_DAMPING/ENV_GRAVITY_MPS2
#define CTRL_PID_KI         CTRL_PID_KD

// Motion Planning Parameters
#define MOT_PLAN_NR_OF_OBSTACLES 1  // nr of obstacles in the map
#define MOT_PLAN_GEOFENCE_X_MAX_M 20.0 // geofence limit in x axis
#define MOT_PLAN_GEOFENCE_Y_MAX_M 20.0 // geofence limit in y axis
#define MOT_PLAN_STEPSIZE_M 0.5 // step size used in search in rrt
#define MOT_PLAN_START_X_M 0.0 //start location of serch in x axis
#define MOT_PLAN_START_Y_M 0.0 // start location of sarch in y axis
#define MOT_PLAN_GOAL_X_M 10.0 // target location of search in x axis
#define MOT_PLAN_GOAL_Y_M 10.0 // target location of search in y axis
#define MOT_PLAN_RRT_STEP_SIZE_M 0.5 // Define step size
#define MOT_PLAN_RRT_MAXNODES 10000 // Define maximum number of nodes to generate

// Fixed-wing parameters. TODO: move to .json config file
#define S   20.0    // Wing area (m^2)
#define C0  2       // Mean chord of the wing (m)
#define B0  4       // Wingspan of the aircraft
// longitudinal
#define CL0 0.2     // Coefficient of lift at zero angle of attack
#define CLA 5.7     // Coefficient of lift slope (per radian)
#define CLQ 0.1     // Coefficient of lift due to pitch rate
#define CD0 0.04    // Coefficient of drag at zero lift
#define CDA 0.3     // Coefficient of drag slope (per radian)
#define CDQ 0.1     // Coefficient of drag due to pitch rate
#define Cm0 0.1     // Coefficient of pitching moment at zero angle of attack
#define CmA -0.08   // Coefficient of pitching moment slope (per radian)
#define CmQ 0.1     // Coefficient of pitching moment due to pitch rate
// lateral
#define CY0 0.2     // Coefficient of side force at zero side slip angle
#define CYB 0.3     // Coefficient of side force slope (per radian)
#define CYP 0.05    // Coefficient of side force due to roll rate
#define CYR 0.05    // Coefficient of side force due to yaq rate
#define Cl0 0.1     // Coefficient of rolling moment at zero side slip angle
#define ClB 0.1     // Coefficient of rolling moment slope (per radian)
#define ClP 0.4     // Coefficient of rolling moment due to roll rate
#define ClR 0.4     // Coefficient of rolling moment due to yaw rate
#define Cn0 0.1     // Coefficient of yawing moment at zero side slip angle
#define CnB 0.1     // Coefficient of yawing moment slope (per radian)
#define CnP 0.05    // Coefficient of yawing moment due to roll rate
#define CnR 0.05    // Coefficient of yawing moment due to yaq rate

#endif // PARAMETERS_H_