/**
 * @author  Burak Yueksel
 * @date    03 January 2023
 * @brief   Math and statistics libraries
 * @addtogroup MATH
 **/

#include "mathematics.h"
#include <math.h>       // for sqrt, cos, log, etc
#include <stdlib.h>     // for RAND_MAX

/** @brief Generate a normally distributed random number*/
float randn()
{
  float u = (float)rand() / (float)RAND_MAX;
  float v = (float)rand() / (float)RAND_MAX;
  return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

/** @brief Compute the approximation square root of a number using Newton's method */
/* Babylonian method" or "Newton's method" to approximate the square root of a number
 * Source: https://en.wikipedia.org/wiki/Methods_of_computing_square_roots
*/
float newton_sqrt(float x)
{
    if (x <= 0) {
        return 0;
    }
    float result = x;
    float delta;
    int i;
    for (i = 0; i < 10; i++) {
        if (result <= 0) {
            result = 0.1;
        }
        delta = x - (result * result);
        result = result + 0.5 * delta / result;
    }
    return result;
}

/** @brief Sign function. Returns the sign of the float variable*/
float signumf(float x)
{
    if (x>=1e-15) return 1;
    else return -1;
}

/** @brief min function. Returns min of 2 input values*/
float minf(float x, float y)
{
    if (x < y) return x;
    else return y;
}

/** @brief max function. Returns max of 2 input values*/
float maxf(float x, float y)
{
    if (x > y) return x;
    else return y;
}