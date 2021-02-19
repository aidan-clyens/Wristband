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
    MAX32664_INIT_HEARTRATE_MODE,
    MAX32664_INIT_ECG_MODE,
    MAX32664_TRIGGER_ALERT
} max32664_event_t;

/*********************************************************************
 * FUNCTIONS
 */

extern bool Max32664_enqueueMsg(max32664_event_t event, void *pData);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MAX32664_TASK_H_ */
