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

float computeAltitude(float pressure, float temperature) {
    // Compute the altitude using the ideal gas law and the international standard atmosphere model
    float altitude = 0 + ((temperature + ENV_ZERO_CELCIUS) / ENV_L) * (pow((pressure / ENV_P0), (-1.0 * ENV_L * ENV_R / (ENV_AIR_MOLAR_MASS * ENV_GRAVITY_MPS2)))-1.0);
    return altitude;
}

// Function to simulate barometer readings
BarometerData getBarometerReadings(float temperature, float true_altitude_m)
{
  BarometerData barometer;

  barometer.temperature = temperature;

  // Compute pressure using the barometric formula
  barometer.pressure = BARO_BIAS + BARO_NOISE_SD * randn() + ENV_P0 * pow(1 + ENV_L * (true_altitude_m-0) / (temperature + ENV_ZERO_CELCIUS), -1.0 * (ENV_GRAVITY_MPS2 * ENV_AIR_MOLAR_MASS) / (ENV_R * ENV_L));

  // compute altitude from pressure and temperature
  barometer.altitude = computeAltitude(barometer.pressure, barometer.temperature);


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