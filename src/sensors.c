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
  barometer.pressure = BARO_PRESSURE_BIAS_PA + BARO_PRESSURE_1_SIGMA_ERROR_PA * randn() + ENV_P0 * pow(1 + ENV_L * (true_altitude_m-0) / (temperature + ENV_ZERO_CELCIUS), -1.0 * (ENV_GRAVITY_MPS2 * ENV_AIR_MOLAR_MASS) / (ENV_R * ENV_L));

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
  measurement.accelerometer_x = true_accelerometer_x + IMU_ACCELEROMETER_BIAS_MPS2 + IMU_ACCELEROMETER_1_SIGMA_ERROR_MPS2 * randn();
  measurement.accelerometer_y = true_accelerometer_y + IMU_ACCELEROMETER_BIAS_MPS2 + IMU_ACCELEROMETER_1_SIGMA_ERROR_MPS2 * randn();
  measurement.accelerometer_z = true_accelerometer_z + IMU_ACCELEROMETER_BIAS_MPS2 + IMU_ACCELEROMETER_1_SIGMA_ERROR_MPS2 * randn();
  measurement.gyroscope_x = true_gyroscope_x + IMU_GYROSCOPE_BIAS_RPS + IMU_GYROSCOPE_1_SIGMA_ERROR_RPS * randn();
  measurement.gyroscope_y = true_gyroscope_y + IMU_GYROSCOPE_BIAS_RPS + IMU_GYROSCOPE_1_SIGMA_ERROR_RPS * randn();
  measurement.gyroscope_z = true_gyroscope_z + IMU_GYROSCOPE_BIAS_RPS + IMU_GYROSCOPE_1_SIGMA_ERROR_RPS * randn();
  return measurement;
}

GNSSData getGNSSReadings(double trueLatitude, double trueLongitude, double trueAltitude, double trueSpeed)
{
  GNSSData gnss;
    // Simulate GNSS data with random noise and error
    // TODO: Replace this with the following:
    /*
    1- set initial coordinates as LLA
    2- get current position from the physics
    3- update the LLA with it.
    4- Use that LLA then here as true LLA.
    */
    /*
    double trueLatitude = 37.7749;
    double trueLongitude = -122.4194;
    double trueAltitude = 10.0;
    double trueSpeed = 5.0;
    */

    // TODO: errors should come from datasheet. So from a parameter file.
    double latitudeError = GNSS_LATTITUDE_BIAS_M + GNSS_LATTITUDE_1_SIGMA_ERROR_M * randn();
    double longitudeError = GNSS_LONGTITUDE_BIAS_M + GNSS_LONGTITUDE_1_SIGMA_ERROR_M * randn();
    double altitudeError = GNSS_ALTITUDE_BIAS_M + GNSS_ALTITUDE_1_SIGMA_ERROR_M * randn();
    double speedError = GNSS_VELOCITY_BIAS_M + GNSS_VELOCITY_1_SIGMA_ERROR_M * randn();

    gnss.latitude = trueLatitude + latitudeError / ENV_RADIUS_OF_EARTH_M * (180 / M_PI);
    gnss.longitude = trueLongitude + longitudeError / (ENV_RADIUS_OF_EARTH_M * cos(trueLatitude * M_PI / 180)) * (180 / M_PI);
    gnss.altitude = trueAltitude + altitudeError;
    gnss.speed = trueSpeed + speedError;

    return gnss;
}