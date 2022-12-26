/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics functions
 * @addtogroup PHYSICS
 **/

#include "parameters.h"
#include "physics.h"


/*define a particle as a point object*/
pointObject g_physicsPointObj;

/** @brief Main  Physics Function */
void physicsMain()
{
    // short test
    printf("Mass the Point Particle:\n");
    printf("% 6.2f ", g_physicsPointObj.mass_kg);
    printf("\n");

    printf("Drag Coeff of the Point Particle:\n");
    printf("% 6.2f ", g_physicsPointObj.dragCoeff);
    printf("\n");

    printf("Moment of Inertia of the Point Particle:\n");
    printMatrix(g_physicsPointObj.I_kgm2);

}

/** @brief Initialize Physics */
void physicsInit()
{
    /*define its mass*/
    g_physicsPointObj.mass_kg    = POINT_MASS_KG;
    /*define its drag coeff*/
    g_physicsPointObj.dragCoeff  = POINT_DRAGCOEFF;
    /*define moment of inertia as a 3x3 matrix*/
    g_physicsPointObj.I_kgm2 = newMatrix(3, 3);
    /*set the moment of inertia terms*/
    setMatrixElement(g_physicsPointObj.I_kgm2, 1, 1, POINT_I_XX_KGM2);//Ixx
    setMatrixElement(g_physicsPointObj.I_kgm2, 2, 2, POINT_I_YY_KGM2);//IYY
    setMatrixElement(g_physicsPointObj.I_kgm2, 3, 3, POINT_I_ZZ_KGM2);//IZZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 1, 2, POINT_I_XY_KGM2);//IXY
    setMatrixElement(g_physicsPointObj.I_kgm2, 2, 1, POINT_I_XY_KGM2);//IYX=IXY
    setMatrixElement(g_physicsPointObj.I_kgm2, 1, 3, POINT_I_XZ_KGM2);//IXZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 3, 1, POINT_I_XZ_KGM2);//IZX=IXZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 2, 3, POINT_I_YZ_KGM2);//IYZ
    setMatrixElement(g_physicsPointObj.I_kgm2, 3, 2, POINT_I_YZ_KGM2);//IZY=IYZ
}