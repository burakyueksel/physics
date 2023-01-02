/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Sensor functions
 * @addtogroup SENSORS
 **/

#include "parameters.h"
#include "sensors.h"
#include <math.h>

// Function to simulate barometer readings
BarometerData getBarometerReadings(float temperature, float altitude)
{
  BarometerData barometer;

  barometer.temperature = temperature;
  barometer.altitude = altitude;

  // Compute pressure using the barometric formula
  barometer.pressure = ENV_P0 * pow(1 - ENV_L * altitude / (temperature + 273.15), (ENV_GRAVITY_MPS2 * 0.0289644) / (ENV_R * ENV_L));

  return barometer;
}
