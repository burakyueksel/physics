/**
 * @author  Burak Yueksel
 * @date    24 December 2022
 * @brief   Physics functions
 * @addtogroup PHYSICS
 **/

#include "parameters.h"
#include "physics.h"

int main() {
    
    /*define a particle as a point object*/
    pointObject particle;

    /*define its mass*/
    particle.mass_kg    = POINT_MASS_KG;
    /*define its drag coeff*/
    particle.dragCoeff  = POINT_DRAGCOEFF;
    /*define moment of inertia as a 3x3 matrix*/
    particle.I_kgm2 = newMatrix(3, 3);
    /*set the moment of inertia terms*/
    setMatrixElement(particle.I_kgm2, 1, 1, POINT_I_XX_KGM2);//Ixx
    setMatrixElement(particle.I_kgm2, 2, 2, POINT_I_YY_KGM2);//IYY
    setMatrixElement(particle.I_kgm2, 3, 3, POINT_I_ZZ_KGM2);//IZZ
    setMatrixElement(particle.I_kgm2, 1, 2, POINT_I_XY_KGM2);//IXY
    setMatrixElement(particle.I_kgm2, 2, 1, POINT_I_XY_KGM2);//IYX=IXY
    setMatrixElement(particle.I_kgm2, 1, 3, POINT_I_XZ_KGM2);//IXZ
    setMatrixElement(particle.I_kgm2, 3, 1, POINT_I_XZ_KGM2);//IZX=IXZ
    setMatrixElement(particle.I_kgm2, 2, 3, POINT_I_YZ_KGM2);//IYZ
    setMatrixElement(particle.I_kgm2, 3, 2, POINT_I_YZ_KGM2);//IZY=IYZ
    
    
    // short test
    printf("Mass the Point Particle:\n");
    printf("% 6.2f ", particle.mass_kg);
    printf("\n");

    printf("Drag Coeff of the Point Particle:\n");
    printf("% 6.2f ", particle.dragCoeff);
    printf("\n");

    printf("Moment of Inertia of the Point Particle:\n");
    printMatrix(particle.I_kgm2);

}