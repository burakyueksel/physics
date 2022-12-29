/**
 * @author  Burak Yueksel
 * @date    26 December 2022
 * @brief   Main routine for the simulations
 * @addtogroup SIM
 **/

#include "parameters.h"
#include <unistd.h> // for usleep function  

void physicsInit();
void physicsMain();
void physicsUpdate(float dt_sec);

extern float g_time_s;

int main ()
{
    // initiate phyiscs (parameters and the states)
    physicsInit();
    // run the time loop (constant dt_sec, i.e. discrete time)
    while (g_time_s<=T_END_S)
    {
        physicsUpdate(DT_S);
        // sleep for DT_S microseconds
        usleep(DT_S*1000000.f);
        g_time_s = g_time_s + DT_S;
    }

    physicsMain();

    return 0;
}