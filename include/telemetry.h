#ifndef TELEMETRY_H_
#define TELEMETRY_H_

/**
 * @author  Burak Yueksel
 * @date    06 January 2023
 * @brief   Telemetry variable IDs
 * @addtogroup TELEMETRY
 **/

// Structure to store PID controller data
typedef enum
{
    ID_REAL_TIME = 0,
    ID_REAL_POS_X_M= 1,
    ID_REAL_POS_Y_M= 2,
    ID_REAL_POS_Z_M= 3,
    ID_REAL_VEL_X_MPS= 4,
    ID_REAL_VEL_Y_MPS= 5,
    ID_REAL_VEL_Z_MPS= 6,
    ID_REAL_ACC_X_MPS2= 7,
    ID_REAL_ACC_Y_MPS2= 8,
    ID_REAL_ACC_Z_MPS2= 9,
    ID_REAL_QUAT_W = 10,
    ID_REAL_QUAT_X = 11,
    ID_REAL_QUAT_Y = 12,
    ID_REAL_QUAT_Z = 13,
    ID_REAL_ROLL_R = 14,
    ID_REAL_PITCH_R= 15,
    ID_REAL_YAW_R  = 16,
    ID_REAL_RRATE_RPS = 17,
    ID_REAL_PRATE_RPS = 18,
    ID_REAL_YRATE_RPS = 19,
    ID_REAL_RACC_RPS2 = 20,
    ID_REAL_PACC_RPS2 = 21,
    ID_REAL_YACC_RPS2 = 22,

    ID_MEAS_BARO_ALT_M = 50,
    ID_MEAS_BARO_PRESSURE_PA = 51
} TelemetryID;

#endif // TELEMETRY_H_