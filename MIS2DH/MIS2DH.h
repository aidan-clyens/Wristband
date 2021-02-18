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
#define MIS2DH_CTRL_REG1    0x20

/*****************************************************
 *  Typedefs
 * ***************************************************/
// Event
typedef enum {
    EVENT_WRITE_CTRL_REG1
} event_t;

// Message
typedef struct {
    event_t event;
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

        // Registers
        void set_ctrl_reg1(uint8_t data);

        // Attribute getters
        power_mode_t get_power_mode() const;
        data_rate_t get_data_rate() const;

        // Static functions
        static void receiveEvent(int num);
        static void requestEvent();

    private:
        QueueArray<uint8_t> m_fifo;

        data_rate_t m_data_rate;

        bool m_low_power_mode;
        bool m_high_resolution_mode;

        uint8_t m_ctrl_reg1;
};