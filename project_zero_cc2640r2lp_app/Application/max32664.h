/*
 * max32664_task.h
 *
 *  Created on: Jan 15, 2021
 *      Author: Aidan Clyens
 */

#ifndef APPLICATION_MAX32664_H_
#define APPLICATION_MAX32664_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */
#define MAX32664_MAXIMFAST_REPORT_ALGO_SIZE     6
#define MAX32664_FIFO_THRESHOLD                 10

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

// Status Byte Value
typedef enum {
  STATUS_SUCCESS = 0x00,
  STATUS_ILLEGAL_FAMILY = 0x01,
  STATUS_NOT_IMPLEMENTED = 0x02,
  STATUS_INCORRECT_NUM_BYTES = 0x03,
  STATUS_ILLEGAL_CONFIG = 0x04,
  STATUS_INCORRECT_MODE = 0x05,
  STATUS_ERROR_FLASHING = 0x80,
  STATUS_CHECKSUM_ERROR = 0x81,
  STATUS_AUTH_ERROR = 0x82,
  STATUS_APPLICATION_INVALID = 0x83,
  STATUS_DEVICE_BUSY = 0xFE,
  STATUS_UNKNOWN_ERROR = 0xFF
} max32664_status_t;

/*********************************************************************
 * FUNCTIONS
 */
// MAX32664 commands
extern max32664_status_t Max32664_initApplicationMode(void *isr_fxn);
extern max32664_status_t Max32664_initMaximFastAlgorithm();

extern max32664_status_t Max32664_readSensorHubStatus(uint8_t *status);
extern max32664_status_t Max32664_readFifoNumSamples(uint8_t *num_samples);
extern max32664_status_t Max32664_readHeartRate(heartrate_data_t *reports, int num_reports);
extern max32664_status_t Max32664_writeInputFifo(uint8_t *data, int num_bytes);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MAX32664_H_ */
