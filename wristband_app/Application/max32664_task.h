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
// Application messages
typedef struct {
    uint8_t event;
    void    *pData;
} max32664_msg_t;

// Event types
typedef enum {
    INIT_HEARTRATE_MODE,
    INIT_ECG_MODE
} max32664_event_t;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Max32664 Biometric Sensor Hub.
 */
extern void Max32664_createTask(void);
extern bool Max32664_enqueueMsg(max32664_event_t event, void *pData);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MAX32664_TASK_H_ */
