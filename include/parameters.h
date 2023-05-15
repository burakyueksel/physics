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
#define ENV_RADIUS_OF_EARTH_M 6371000
#define ENV_GRAVITY_MPS2 9.80665f
#define ENV_R 8.3144598   // universal gas constant for air (J/kg/K)
#define ENV_L -0.0065  // temperature lapse rate (K/m) at P0 and T0
#define ENV_P0 101325.0  // standard atmospheric pressure at sea level (Pa)
#define ENV_AIR_MOLAR_MASS 0.0289644 // Molar mass of air (kg/mol)
#define ENV_T0  288.15 // Standard temperature at sea level (K)
#define ENV_ZERO_CELCIUS 273.15 // Zero (0) Celcius in Kelvins (K)
#define ENV_TEMP_C 20   // temperature of the air outside (C)

// Barometer parameters
#define BARO_PRESSURE_BIAS_PA 5.0 // bias of noise (in Pa)
#define BARO_PRESSURE_1_SIGMA_ERROR_PA 3.0  // standard deviation of noise (in Pa)

// Parameters for the IMU sensor model
#define IMU_ACCELEROMETER_BIAS_MPS2 0.1  // m/s^2
#define IMU_GYROSCOPE_BIAS_RPS 0.01     // rad/s
#define IMU_ACCELEROMETER_1_SIGMA_ERROR_MPS2 0.01  // m/s^2
#define IMU_GYROSCOPE_1_SIGMA_ERROR_RPS 0.001     // rad/s

// Parameters for GNSS sensor model
#define GNSS_LATTITUDE_BIAS_M 0.0 // m
#define GNSS_LATTITUDE_1_SIGMA_ERROR_M 1.0 // m. 1-sigma error of 1 meter
#define GNSS_LONGTITUDE_BIAS_M 0.0 // m
#define GNSS_LONGTITUDE_1_SIGMA_ERROR_M 1.0 // m. 1-sigma error of 1 meter
#define GNSS_ALTITUDE_BIAS_M 0.0 // m
#define GNSS_ALTITUDE_1_SIGMA_ERROR_M 0.5 // m. 1-sigma error of 0.5 meters
#define GNSS_VELOCITY_BIAS_M 0.0 // m/s
#define GNSS_VELOCITY_1_SIGMA_ERROR_M 0.1 // m/s. 1-sigma error of 0.1 m/s

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

// Geographic Parameters
#define GEO_INIT_LATITUDE_DEG   37.7749 // initial (start) location latitude coordinate in deg
#define GEO_INIT_LONGITUDE_DEG  -122.4194 // initial (start) location longitude coordinate in deg
#define GEO_INIT_ALTITUDE_M     10.0 // initial (start) altitude in meters

// Double precision parameters, beacause float precision would not be sufficient
static const double WGS84_SMAA_M = 6378137.0; // semi-major axis of the WGS84 ellipsoid (in meters)
static const double WGS84_SMIA_M = 6356752.3142; // semi-minor axis of the WGS84 ellipsoid (in meters)
static const double FLAT_WGS84 = (WGS84_SMAA_M - WGS84_SMIA_M) / WGS84_SMAA_M; // flattening of the WGS84 ellipsoid
static const double ECCENTRICITY_2_WGS84 = 2 * FLAT_WGS84 - FLAT_WGS84 * FLAT_WGS84; // eccentricity squared of the ellipsoid

#endif // PARAMETERS_H_