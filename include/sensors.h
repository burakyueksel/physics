#ifndef SENSORS_H_
#define SENSORS_H_

/**
 * @author  Burak Yueksel
 * @date    29 December 2022
 * @brief   Sensor libraries
 * @addtogroup SENSORS
 **/


// Structure to store barometer data
typedef struct {
  double temperature;  // temperature (in C)
  double pressure;     // pressure (in Pa)
  double altitude;     // altitude (in m)
} BarometerData;

BarometerData getBarometerReadings(float temperature, float altitude);

#endif // SENSORS_H_