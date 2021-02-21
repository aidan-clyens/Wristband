/*
 * mis2dh_task.h
 *
 *  Created on: Feb 17, 2021
 *      Author: Aidan Clyens
 */

#ifndef APPLICATION_MIS2DH_TASK_H_
#define APPLICATION_MIS2DH_TASK_H_

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
extern bool Mis2dh_init(void);
extern bool Mis2dh_readSensorData(sensor_data_t *data);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MIS2DH_TASK_H_ */
