/**
 * @author  Burak Yueksel
 * @date    26 December 2022
 * @brief   Main routine for the simulations
 * @addtogroup SIM
 **/

#include "controls.h"
#include "publish.h"
#include "parameters.h"
#include "physics.h"
#include "sensors.h"
#include <unistd.h> // for usleep function  

extern float g_time_s;

int main ()
{
    // open a file to do the logging
    FILE* log_file = fopen("log.txt", "a");
    // initiate phyiscs (parameters and the states)
    physicsInit();
    // states
    states* realStates;
    // PID controller for the z position
    PIDController pid;
    // init PID
    initPID(&pid, CTRL_PID_KP, CTRL_PID_KD, CTRL_PID_KI);
    float setAltitude_m = 10.0f;
    // run the time loop (constant dt_sec, i.e. discrete time)
    while (g_time_s<=REALTIME_T_END_S)
    {
        realStates = &g_phsicsPointStates;

        // Call barometer for the altitude
        BarometerData barometer = getBarometerReadings(ENV_TEMP_C, -realStates->trState.pos_Inertial_m[2]);


        float errZPos_m = setAltitude_m - barometer.altitude;
        // TODO: put controller time step here (but notice the usleep fcn below. Instead of usleep consider writing callback fcns)
        float zCmd = updatePID(&pid, errZPos_m, REALTIME_DT_S);

        // external forces (control + disturbances)
        vector3 extForces_N = {0.0f, 0.0f, -zCmd};
        // external torques (control + disturbances)
        vector3 extMoments_Nm = {0.1f, 0.0f, 0.0f};

        physicsUpdate(realStates, extForces_N, extMoments_Nm, REALTIME_DT_S);
        // sleep for DT_S microseconds
        usleep(REALTIME_DT_S*1000000.f);
        g_time_s = g_time_s + REALTIME_DT_S;

        // log real altitude, velocity and acceleration in z axis
        publishToLogScalar(ID_REAL_POS_Z_M, g_time_s, realStates->trState.pos_Inertial_m[2], log_file);
        publishToLogScalar(ID_REAL_VEL_Z_MPS, g_time_s, realStates->trState.velInertial_mps[2], log_file);
        publishToLogScalar(ID_REAL_ACC_Z_MPS2, g_time_s, realStates->trState.accInertial_mps2[2], log_file);
        // log barometric data
        publishToLogScalar(ID_MEAS_BARO_ALT_M, g_time_s, barometer.altitude, log_file);
        publishToLogScalar(ID_MEAS_BARO_PRESSURE_PA, g_time_s, barometer.pressure, log_file);


        // Barometer test
        printf("Temperature: %f C\n", barometer.temperature);
        printf("Pressure: %f Pa\n", barometer.pressure);
        printf("Altitude: %f m\n", barometer.altitude);

        /*print the time*/
        printf("Current time is:\n");
        printf("% 6.4f ", g_time_s);
        printf("\n");
        /*Print the positions*/
        printf("3D Positions (x,y,z) in meters are:\n");
        //publishToTerminalVector3(realStates->trState.pos_Inertial_m); // gives same result as using the global
        publishToTerminalVector3(g_phsicsPointStates.trState.pos_Inertial_m);

        /*Print the velocities*/
        printf("3D Velocities (x,y,z) in m/s are:\n");
        //publishToTerminalVector3(realStates->trState.pos_Inertial_m); // gives same result as using the global
        publishToTerminalVector3(g_phsicsPointStates.trState.velInertial_mps);

        /*Print the quaternions*/
        printf("Quaternions:\n");
        publishToTerminalQuaternion(&g_phsicsPointStates.rtState.q);

        /*Print the Euler Angles in rad*/
        printf("Euler angles in radians are:\n");
        //publishToTerminalVector3(realStates->trState.pos_Inertial_m); // gives same result as using the global
        publishToTerminalEuler(&g_phsicsPointStates.rtState.euler_r);


    }
    // close the logging file
    fclose(log_file);
    return 0;
}