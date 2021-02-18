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

/*****************************************************
 *  Typedefs
 * ***************************************************/
typedef enum {
    MODE_LOW_POWER,
    MODE_NORMAL,
    MODE_HIGH_RESOLUTION
} power_mode_t;

/*****************************************************
 *  Global Variables
 * ***************************************************/
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

        static void receiveEvent(int num);
        static void requestEvent();

    private:
        QueueArray<uint8_t> m_fifo;

        power_mode_t m_power_mode;

        static uint8_t m_register;
        static uint8_t m_data;
};