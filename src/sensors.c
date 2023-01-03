/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Sensor functions
 * @addtogroup SENSORS
 **/

#include "mathematics.h"
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

// Generate a new IMU measurement with noise and bias
IMUData getIMUReadings(float true_accelerometer_x, float true_accelerometer_y,
                                   float true_accelerometer_z, float true_gyroscope_x,
                                   float true_gyroscope_y, float true_gyroscope_z) 
{
  IMUData measurement;
  measurement.accelerometer_x = true_accelerometer_x + BIAS_ACCELEROMETER + NOISE_ACCELEROMETER * randn();
  measurement.accelerometer_y = true_accelerometer_y + BIAS_ACCELEROMETER + NOISE_ACCELEROMETER * randn();
  measurement.accelerometer_z = true_accelerometer_z + BIAS_ACCELEROMETER + NOISE_ACCELEROMETER * randn();
  measurement.gyroscope_x = true_gyroscope_x + BIAS_GYROSCOPE + NOISE_GYROSCOPE * randn();
  measurement.gyroscope_y = true_gyroscope_y + BIAS_GYROSCOPE + NOISE_GYROSCOPE * randn();
  measurement.gyroscope_z = true_gyroscope_z + BIAS_GYROSCOPE + NOISE_GYROSCOPE * randn();
  return measurement;
}