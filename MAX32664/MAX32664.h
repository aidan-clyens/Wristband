#pragma MIS2DH_H
/*****************************************************
 *  Includes
 * ***************************************************/
#include <Arduino.h>
#include <Wire.h>

#include "QueueArray.h"

/*****************************************************
 *  Defines
 * ***************************************************/
#define MFIO                        10
#define RESET                       11

#define ADDRESS                     0xAA

#define FIFO_DEPTH                  128
#define SAMPLE_PERIOD_MS            4*1000  // TODO: Change to 25 Hz

#define REPORT_ALGORITHM_NORMAL_SIZE    20

// Family names
#define READ_SENSOR_HUB_STATUS      0x00
#define SET_OUTPUT_MODE             0x10
#define READ_OUTPUT_MODE            0x11
#define READ_OUTPUT_FIFO            0x12
#define SENSOR_MODE_ENABLE          0x44
#define ALGORITHM_MODE_ENABLE       0x52

// Sensors
#define MAX86141_ENABLE             0x00

// Algorithms
#define AGC_ALGORITHM               0x00
#define WHRM_WSPO2_ALGORITHM        0x07

// Accelerometer
#define ACCELEROMETER_ENABLE       0x04
#define EXTERNAL_ACCELEROMETER     0x00

// FIFO
#define NUM_FIFO_SAMPLES            0x00
#define READ_FIFO_DATA              0x01

/*****************************************************
 *  Typedefs
 * ***************************************************/
// Sensor Hub Operating Mode
typedef enum {
    SENSOR_MODE_NONE,
    SENSOR_MODE_APPLICATION,
    SENSOR_MODE_BOOTLOADER
} sensor_mode_t;

// Sensor Output Mode
typedef enum {
    OUTPUT_MODE_PAUSE = 0x00,
    OUTPUT_MODE_SENSOR_DATA = 0x01,
    OUTPUT_MODE_ALGORITHM_DATA = 0x02,
    OUTPUT_MODE_SENSOR_AND_ALGORITHM_DATA = 0x03,
    OUTPUT_MODE_PAUSE2 = 0x04,
    OUTPUT_MODE_SAMPLE_COUNTER_BYTE_SENSOR_DATA = 0x05,
    OUTPUT_MODE_SAMPLE_COUNTER_BYTE_ALGORITHM_DATA = 0x06,
    OUTPUT_MODE_SAMPLE_COUNTER_BYTE_SENSOR_AND_ALGORITHM_DATA = 0x07
} output_mode_t;

// Status Byte Values
typedef enum {
    SUCCESS = 0x00,
    ILLEGAL_FAMILY = 0x01,
    NOT_IMPLEMENTED = 0x02,
    INCORRECT_NUM_BYTES = 0x03,
    ILLEGAL_CONFIG = 0x04,
    INCORRECT_MODE = 0x05,
    ERROR_FLASHING = 0x80,
    CHECKSUM_ERROR = 0x81,
    AUTH_ERROR = 0x82,
    APPLICATION_INVALID = 0x83,
    DEVICE_BUSY = 0xFE,
    UNKNOWN_ERROR = 0xFF
} status_t;

// SCD State
typedef enum {
    SCD_UNDETECTED,
    SCD_OFF_SKIN,
    SCD_ON_SUBJECT,
    SCD_ON_SKIN
} scd_state_t;

/*****************************************************
 *  Global Variables
 * ***************************************************/
static uint8_t family = 0x00;
static uint8_t index = 0x00;
static uint8_t data = 0x00;
static uint8_t second_data = 0x00;

static int num_samples_read = 0;

/*****************************************************
 *  Class Definition
 * ***************************************************/
class max32664 {
    public:
        max32664();
        virtual ~max32664();

        void init();
        void reset();
        void runSensorHub();

        bool checkResetPinAsserted() const;

        void addSampleToFifoNormalReport();

        // Static functions
        static void receiveEvent(int num);
        static void requestEvent();

    private:
        static QueueArray<uint8_t> m_output_fifo;

        static sensor_mode_t m_sensor_mode;

        static int m_fifo_interrupt_threshold;
        static uint8_t m_sensor_hub_status;
        static uint8_t m_output_mode;

        static bool m_agc_enable;
        static bool m_max86141_enable;
        static int m_whrm_wspo2_mode;

        unsigned long m_current_millis;
        unsigned long m_prev_millis;
};