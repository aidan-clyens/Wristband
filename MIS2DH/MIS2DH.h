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
#define ADDRESS         0x18
#define FIFO_DEPTH      32

// Registers
#define MIS2DH_CTRL_REG1            0x20
#define MIS2DH_CTRL_REG5            0x24
#define MIS2DH_OUT_X_L              0x28
#define MIS2DH_OUT_X_H              0x29
#define MIS2DH_OUT_Y_L              0x2A
#define MIS2DH_OUT_Y_H              0x2B
#define MIS2DH_OUT_Z_L              0x2C
#define MIS2DH_OUT_Z_H              0x2D
#define MIS2DH_FIFO_CTRL_REG        0x2E
#define MIS2DH_FIFO_SRC_REG         0x2F


/*****************************************************
 *  Typedefs
 * ***************************************************/
// Sensor Data
typedef struct {
    uint8_t x_L;
    uint8_t x_H;
    uint8_t y_L;
    uint8_t y_H;
    uint8_t z_L;
    uint8_t z_H;
} sensor_data_t;

// Message
typedef struct {
    uint8_t register_address;
    uint8_t data;
} message_t;

// Power Mode
typedef enum {
    MODE_LOW_POWER,
    MODE_NORMAL,
    MODE_HIGH_RESOLUTION
} power_mode_t;

// Data Rate
typedef enum {
    DATARATE_POWER_DOWN = 0x00,
    DATARATE_1HZ = 0x01,
    DATARATE_10HZ = 0x02,
    DATARATE_25HZ = 0x03,
    DATARATE_50HZ = 0x04,
    DATARATE_100HZ = 0x05,
    DATARATE_200HZ = 0x06,
    DATARATE_400HZ = 0x07,
    DATARATE_1620HZ = 0x08,
    DATARATE_5376HZ = 0x09
} data_rate_t;

// FIFO Mode
typedef enum {
    FIFO_MODE_BYPASS = 0x0,
    FIFO_MODE_FIFO = 0x01,
    FIFO_MODE_STREAM = 0x10,
    FIFO_MODE_STREAM_TO_FIFO = 0x11
} fifo_mode_t;

/*****************************************************
 *  Global Variables
 * ***************************************************/
static QueueArray<message_t> message_queue(128);

static uint8_t register_address;
static uint8_t data;

/*****************************************************
 *  Class Definition
 * ***************************************************/
class mis2dh {
    public:
        mis2dh();
        virtual ~mis2dh();

        void init();

        void process_messages();
        void read_accelerometer();

        // Registers
        void set_ctrl_reg1(uint8_t data);
        void set_ctrl_reg5(uint8_t data);
        void set_fifo_ctrl_reg(uint8_t data);

        // Attribute getters
        power_mode_t get_power_mode() const;
        data_rate_t get_data_rate() const;
        fifo_mode_t get_fifo_mode() const;
        bool is_fifo_enabled() const;

        // Static functions
        static void receiveEvent(int num);
        static void requestEvent();

    private:
        data_rate_t m_data_rate;
        fifo_mode_t m_fifo_mode;

        bool m_low_power_mode;
        bool m_high_resolution_mode;

        bool m_fifo_enabled;

        static QueueArray<sensor_data_t> m_fifo;

        static uint8_t m_ctrl_reg1;
        static uint8_t m_ctrl_reg5;
        static uint8_t m_fifo_ctrl_reg;
        static uint8_t m_fifo_src_reg;

        unsigned long m_sensor_period_ms;
        unsigned long m_current_ms;
        unsigned long m_prev_ms;

        unsigned long get_period_from_data_rate() const;
};