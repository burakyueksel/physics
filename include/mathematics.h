#ifndef MATHEMATICS_H_
#define MATHEMATICS_H_

/**
 * @author  Burak Yueksel
 * @date    28 December 2022
 * @brief   Math libraries
 * @addtogroup MATH
 **/



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

#endif // MATHEMATICS_H_