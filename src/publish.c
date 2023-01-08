/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Publish libraries
 * @addtogroup PUBLISH_H_
 **/

#include "publish.h"

/* Prints the 3D vector to stdout.  Returns 0 if successful 
 * and -1 if vector is NULL.
 */
int publishToTerminalVector3(vector3 vec3)
{
  if (!vec3) return -1;
  
  int item;
  for (item = 0; item < 3; item++) 
  {
    // Print the floating-point element with
    //  - either a - if negative or a space if positive
    //  - at least 3 spaces before the .
    //  - precision to the hundredths place
    printf("% 6.4f ", vec3[item]);
    // separate rows by newlines
    printf("\n");
  }
  return 0;
}

/* Prints the 4D quaternion vector to stdout.  Returns 0 if successful 
 * and -1 if mtx is NULL.
 */
int publishToTerminalQuaternion(quaternion* q)
{
  if (!q) return -1;
  
  int item;
  for (item = 0; item < 4; item++) 
  {
    // Print the floating-point element
    printf("% 6.2f ", *((float*)q + item));
    // separate rows by newlines
    printf("\n");
  }
  return 0;
}

/* Prints the 3D euler angle vector to stdout.  Returns 0 if successful
 * and -1 if mtx is NULL.
 */
int publishToTerminalEuler(euler* e)
{
  if (!e) return -1;

  int item;
  for (item = 0; item < 3; item++) 
  {
    // Print the floating-point element
    printf("% 6.2f ", *((float*)e + item));
    // separate rows by newlines
    printf("\n");
  }
  return 0;
}

void publishToLogScalar(TelemetryID ID, float time_s, float value, FILE* log_file)
{
  fprintf(log_file, "%d %f %f\n", ID, time_s, value);
}