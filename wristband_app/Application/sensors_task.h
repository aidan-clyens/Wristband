/*
 * sensors_task.h
 *
 *  Created on: Feb 19, 2021
 *      Author: Aidan Clyens
 */

#ifndef APPLICATION_SENSORS_TASK_H_
#define APPLICATION_SENSORS_TASK_H_

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
} sensors_msg_t;

// Event types
typedef enum {
    SENSORS_INIT_HEARTRATE_MODE,
    SENSORS_TRIGGER_ALERT
} sensors_event_t;

/*********************************************************************
 * FUNCTIONS
 */
extern void Sensors_createTask(void);
extern bool Sensors_enqueueMsg(sensors_event_t event, void *pData);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SENSORS_TASK_H_ */
