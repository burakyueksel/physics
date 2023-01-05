#ifndef SENSORS_H_
#define SENSORS_H_

/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Sensor libraries
 * @addtogroup SENSORS
 **/


// Structure to store barometer data
typedef struct
{
  float temperature;  // temperature (in C)
  float pressure;     // pressure (in Pa)
  float altitude;     // altitude (in m)
} BarometerData;

// Struct for storing IMU sensor measurements
typedef struct
{
  float accelerometer_x;
  float accelerometer_y;
  float accelerometer_z;
  float gyroscope_x;
  float gyroscope_y;
  float gyroscope_z;
} IMUData;

BarometerData getBarometerReadings(float temperature, float true_altitude_m);
IMUData getIMUReadings(float true_accelerometer_x, float true_accelerometer_y,
                                   float true_accelerometer_z, float true_gyroscope_x,
                                   float true_gyroscope_y, float true_gyroscope_z);

#endif // SENSORS_H_