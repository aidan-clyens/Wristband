/*
 * lis3dh.h
 *
 *  Created on: Feb 17, 2021
 *      Author: Aidan Clyens
 */

#ifndef APPLICATION_LIS3DH_H_
#define APPLICATION_LIS3DH_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * TYPEDEFS
 */
// Sensor Data
typedef struct {
    uint8_t x_L;
    uint8_t x_H;
    uint8_t y_L;
    uint8_t y_H;
    uint8_t z_L;
    uint8_t z_H;
} sensor_data_t;

/*********************************************************************
 * FUNCTIONS
 */
extern bool Lis3dh_init(void);
extern bool Lis3dh_getNumUnreadSamples(int *num_samples);
extern bool Lis3dh_readSensorData(sensor_data_t *data, int numSamples);

extern void Lis3dh_printSample(sensor_data_t data);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_LIS3DH_H_ */
