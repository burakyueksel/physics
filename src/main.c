/**
 * @author  Burak Yueksel
 * @date    26 December 2022
 * @brief   Main routine for the simulations
 * @addtogroup SIM
 **/

#include "controls.h"
#include "parameters.h"
#include "physics.h"
#include "sensors.h"
#include <unistd.h> // for usleep function  

extern float g_time_s;


int main ()
{
    // initiate phyiscs (parameters and the states)
    physicsInit();
    // states
    states* ps;
    // PID controller for the z position
    PIDController pid;
    float kp = POINT_MASS_KG * CTRL_PID_OMEGA_RPS*CTRL_PID_OMEGA_RPS/ENV_GRAVITY_MPS2;
    float kd = 2.0 * CTRL_PID_OMEGA_RPS * CTRL_PID_DAMPING/ENV_GRAVITY_MPS2;
    float ki = kd;

    initPID(&pid, kp, kd, ki);
    float setZPos_m = -10.0f;
    // run the time loop (constant dt_sec, i.e. discrete time)
    while (g_time_s<=T_END_S)
    {
        ps = &g_phsicsPointStates;

        float errZPos_m = setZPos_m - ps->trState.pos_Inertial_m[2];
        float zCmd = updatePID(&pid, errZPos_m, DT_S);

        // external forces (control + disturbances)
        vector3 extForces_N = {0.0f, 0.0f, zCmd};
        // external torques (control + disturbances)
        vector3 extMoments_Nm = {0.1f, 0.0f, 0.0f};

        physicsUpdate(ps, extForces_N, extMoments_Nm, DT_S);
        // sleep for DT_S microseconds
        usleep(DT_S*1000000.f);
        g_time_s = g_time_s + DT_S;

        // Barometer test

        BarometerData barometer = getBarometerReadings(20, -ps->trState.pos_Inertial_m[2]);

        printf("Temperature: %f C\n", barometer.temperature);
        printf("Pressure: %f Pa\n", barometer.pressure);
        printf("Altitude: %f m\n", barometer.altitude);
    }

    physicsMain();

    return 0;
}