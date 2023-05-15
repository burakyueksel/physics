/**
 * @author  Burak Yueksel
 * @date    26 December 2022
 * @brief   Main routine for the simulations
 * @addtogroup SIM
 **/

#include "controls.h"
#include "config.h"
#include "publish.h"
#include "parameters.h"
#include "planning.h"
#include "physics.h"
#include "sensors.h"
#include <unistd.h> // for usleep function  
#include <time.h> // for clock() fcn for measuring the clock ticks that have elapsed
#include <stdlib.h> // for malloc fcns

extern float g_time_s;
clock_t start, end;
float cpu_time_used;

int main ()
{

    // Generate header file from configs
    generate_header_from_json("configs/quadrotor.json", "parameters/quadrotor_config.h", "QUADROTOR_CONFIG");
    printf("Header file generated successfully!\n");
    // break point
    // return 0;

    // open a file to do the logging
    FILE* log_file = fopen("log.txt", "a");
/*
 * MOTION PLANNING
*/
///////////////
    // Generate an obstacle
    float** obstacles = generateSingleObstacle(5.0, 5.0, 2.0);
    // Run RRT algorithm
    Node** nodes = RRT(MOT_PLAN_START_X_M, MOT_PLAN_START_Y_M,
                      MOT_PLAN_GOAL_X_M, MOT_PLAN_GOAL_Y_M,
                      MOT_PLAN_GEOFENCE_X_MAX_M, MOT_PLAN_GEOFENCE_Y_MAX_M,
                      MOT_PLAN_RRT_STEP_SIZE_M, MOT_PLAN_RRT_MAXNODES,
                      MOT_PLAN_NR_OF_OBSTACLES, obstacles);

    // Print the results
    printRRTPath(nodes, MOT_PLAN_RRT_MAXNODES);
///////////////
    // initiate phyiscs (parameters and the states)
    physicsInit();
    // initiate sensors
    sensorsInit();
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
        start = clock();

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


        // test script for cholesky decomposition
        matrix* L = newMatrix(3,3);
        matrix* A = newMatrix(3,3);
        matrix* Ai = newMatrix(3,3);
        matrix* AAi = newMatrix(3,3);
        // chose A matrix from the example here: https://rosettacode.org/wiki/Cholesky_decomposition#C
        setMatrixElement(A, 1, 1, 25);
        setMatrixElement(A, 1, 2, 15);
        setMatrixElement(A, 1, 3, -5);
        setMatrixElement(A, 2, 1, 15);
        setMatrixElement(A, 2, 2, 18);
        setMatrixElement(A, 2, 3, 0);
        setMatrixElement(A, 3, 1, -5);
        setMatrixElement(A, 3, 2, 0);
        setMatrixElement(A, 3, 3, 11);
        coleskyDecomp(A, L);
        inverseMatrixChol(A,Ai);
        productMatrix(A,Ai,AAi);
        /*
        int a, b, c;
        a=isMatrixSymmetricPositiveDefinite(A);
        b=isMatrixSymmetric(A);
        c=isMatrixHermitian(A);
        printf(" isMatrixSymmetric: %d .\n", b);
        printf(" isMatrixHermitian: %d .\n", c);
        */
        //matrix* LT = newMatrix(3,3);
        //transposeMatrix(L,LT);
        //productMatrix(L,LT,Ai);
        printf("Matrix A:\n");
        printMatrix(A);
        printf("Matrix L:\n");
        printMatrix(L);
        printf("Matrix A inverse:\n");
        printMatrix(Ai);
        printf("Matrix A times A inverse (expect identity):\n");
        printMatrix(AAi);
        // delete them
        deleteMatrix(A);
        deleteMatrix(L);
        deleteMatrix(Ai);
        deleteMatrix(AAi);

/*
        // test script for matrixConc. To be removed.
        matrix * A, * B, * C;
        A = newMatrix(3, 3);
        C = newMatrix(3,6);
        setMatrixElement(A, 1, 1, 1.0);
        setMatrixElement(A, 1, 2, .25);
        setMatrixElement(A, 1, 3, -.1);
        setMatrixElement(A, 2, 2, .4);
        setMatrixElement(A, 2, 3, .3);
        setMatrixElement(A, 3, 2, .1);
        setMatrixElement(A, 3, 3, -.3);
        printf("Matrix A:\n");
        printMatrix(A);
        B = copyMatrix(A);

        matrixConcatenation(A, B, C);
        printf("Matrix B:\n");
        printMatrix(B);
        printf("Matrix C:\n");
        printMatrix(C);
*/


    end = clock();
    cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
    printf("Time taken by function: %f seconds.\n", cpu_time_used);
    }
    // close the logging file
    fclose(log_file);
    return 0;
}