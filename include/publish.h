#ifndef PUBLISH_H_
#define PUBLISH_H_

/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Publish libraries
 * @addtogroup PUBLISH_H_
 **/

#include <stdio.h> // for print function
#include "vector.h"
#include "telemetry.h"
/*
********************************************
** FUNCTION DECLERATIONS
********************************************
*/
int publishToTerminalVector3(vector3 vec3);
int publishToTerminalQuaternion(quaternion* q);
int publishToTerminalEuler(euler* e);
void publishToLogScalar(TelemetryID ID, float time_s, float value, FILE* log_file);
#endif // PUBLISH_H_