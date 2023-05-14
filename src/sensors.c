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

GNSSData ENU2LLA(float east, float north, float up, double lat0, double lon0, double alt0)
{
    // source [1]: https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
    GNSSData data;

    const double f = (WGS84_SMAA_M - WGS84_SMIA_M) / WGS84_SMAA_M; // flattening of the WGS84 ellipsoid
    const double e2 = 2 * f - f * f; // eccentricity squared of the ellipsoid

    // Convert the ENU positions to ECEF positions
    /*
    From ECEF to ENU, see the matrix in eq. (6) of [1].
    From ENU to ECEF, see eq. (7) of [1].
    */
    double sinlat0 = sin(lat0 * M_PI / 180.0);
    double coslat0 = cos(lat0 * M_PI / 180.0);
    double sinlon0 = sin(lon0 * M_PI / 180.0);
    double coslon0 = cos(lon0 * M_PI / 180.0);
    double x = -sinlon0 * east - coslon0 * sinlat0 * north + coslon0 * coslat0 * up;
    double y = coslon0 * east - sinlon0 * sinlat0 * north + sinlon0 * coslat0 * up;
    double z = coslat0 * north + sinlat0 * up;

    // Compute the geocentric latitude, longitude, and altitude of the point (x, y, z)
    /*
    r = sqrt(x^2 + y^2 + z^2)
    latgc = asin(z / r)
    longc = atan2(y, x)
    altgc = r - WGS84_SMAA_M / sqrt(1 - e^2 * sin^2(latgc))
    */
    double r = sqrt(x * x + y * y + z * z);
    double latgc = asin(z / r);
    double longc = atan2(y, x);
    double altgc = r - WGS84_SMAA_M / sqrt(1 - e2 * sin(latgc) * sin(latgc));

    // Convert the geocentric latitude, longitude, and altitude to the corresponding WGS84 latitude, longitude, and altitude
    /*
    N = a / sqrt(1 - e^2 * sin^2(latgc))
    lat = atan2(sin(latgc) * N + alt0, r * cos(latgc) + (1 - e^2) * N + alt0)
    lon = atan2(sin(longc), cos(longc) * cos(latgc))
    alt = r * cos(latgc) + altgc * sin(latgc) - N * (1 - e^2 * sin^2(latgc)) + alt0
    */
    double sinlatgc = sin(latgc);
    double coslatgc = cos(latgc);
    double sinlongc = sin(longc);
    double coslongc = cos(longc);
    double N = WGS84_SMAA_M / sqrt(1 - e2 * sinlatgc * sinlatgc);

    data.latitude   = atan2(sinlatgc * N + alt0, r * coslatgc + coslatgc * N * (1 - e2) + alt0);
    data.longitude  = atan2(sinlongc, coslongc * coslatgc);
    data.altitude = r * coslatgc + altgc * sinlatgc - N * (1 - e2 * (sinlatgc * sinlatgc)) + alt0;
    // convert lat and lon to degrees
    data.latitude *= 180.0 / M_PI; // Convert radians to degrees
    data.latitude *= 180.0 / M_PI; // Convert radians to degrees
    // Notice: data.vel_x, data.vel_y, data.vel_z are NOT set here.
    return data;
}


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

GNSSData getGNSSReadings(double lat0, double lon0, double alt0, float east, float north, float up, float trueSpeed_x, float trueSpeed_y, float trueSpeed_z)
{
  GNSSData gnssIn;
  GNSSData gnssOut;
  // Simulate GNSS data with random noise and error
  /*
  double trueLatitude = 37.7749;
  double trueLongitude = -122.4194;
  double trueAltitude = 10.0;
  double trueSpeed = 5.0;
  */

  // convert ENU positions to LLA and get the true LLA coordinates
  gnssIn = ENU2LLA(east, north, up, lat0, lon0, alt0);

  // compute bias and random 1 sigma error for the coordinates
  double latitudeError = GNSS_LATTITUDE_BIAS_M + GNSS_LATTITUDE_1_SIGMA_ERROR_M * randn();
  double longitudeError = GNSS_LONGTITUDE_BIAS_M + GNSS_LONGTITUDE_1_SIGMA_ERROR_M * randn();
  double altitudeError = GNSS_ALTITUDE_BIAS_M + GNSS_ALTITUDE_1_SIGMA_ERROR_M * randn();
  // compute bias and random 1 sigma error for the speeds
  float speedError_x = GNSS_VELOCITY_BIAS_M + GNSS_VELOCITY_1_SIGMA_ERROR_M * randn();
  float speedError_y =  GNSS_VELOCITY_BIAS_M + GNSS_VELOCITY_1_SIGMA_ERROR_M * randn();
  float speedError_z =  GNSS_VELOCITY_BIAS_M + GNSS_VELOCITY_1_SIGMA_ERROR_M * randn();
  // update the true values with this noise model
  gnssOut.latitude = gnssIn.latitude + latitudeError / ENV_RADIUS_OF_EARTH_M * (180 / M_PI);
  gnssOut.longitude = gnssIn.longitude + longitudeError / (ENV_RADIUS_OF_EARTH_M * cos(gnssIn.latitude * M_PI / 180)) * (180 / M_PI);
  gnssOut.altitude = gnssIn.altitude + altitudeError;
  gnssOut.vel_x = trueSpeed_x + speedError_x;
  gnssOut.vel_y = trueSpeed_y + speedError_y;
  gnssOut.vel_z = trueSpeed_z + speedError_z;

  return gnssOut;
}