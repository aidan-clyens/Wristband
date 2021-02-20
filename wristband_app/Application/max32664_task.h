/*
 * max32664_task.h
 *
 *  Created on: Jan 15, 2021
 *      Author: Aidan Clyens
 */

#ifndef APPLICATION_MAX32664_TASK_H_
#define APPLICATION_MAX32664_TASK_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * TYPEDEFS
 */
// Heart Rate Data
typedef struct {
    uint16_t heartRate;
    uint8_t heartRateConfidence;
    uint16_t spO2;
    uint8_t spO2Confidence;
    uint8_t scdState;
} heartrate_data_t;

/*********************************************************************
 * FUNCTIONS
 */
// MAX32664 commands
extern void Max32664_initApplicationMode();
extern void Max32664_initHeartRateAlgorithm();
extern void Max32664_readHeartRate(heartrate_data_t *data);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MAX32664_TASK_H_ */
