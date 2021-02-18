#include "MIS2DH.h"

/*********************************************************************
 * @fn      mis2sh
 *
 * @brief   MIS2DH constructor.
 */
mis2dh::mis2dh():
m_fifo(FIFO_DEPTH),
m_ctrl_reg1(0x0),
m_power_mode(MODE_LOW_POWER),
m_data_rate(DATARATE_POWER_DOWN)
{

}

/*********************************************************************
 * @fn      ~mis2sh
 *
 * @brief   MIS2DH destructor.
 */
mis2dh::~mis2dh() {
    Wire.end();
}

/*********************************************************************
 * @fn      init
 *
 * @brief   Initialize I2C on MIS2DH.
 */
void mis2dh::init() {
    Wire.begin(ADDRESS);
    Wire.onReceive(this->receiveEvent);
    Wire.onRequest(this->requestEvent);

    Serial.println("MIS2DH Sensor Initialized");
}

/*********************************************************************
 * @fn      process_messages
 *
 * @brief   Process messages from the sensor message queue. 
 */
void mis2dh::process_messages() {
    while (!message_queue.empty()) {
        message_t msg = message_queue.front();
        message_queue.pop();

        switch (msg.event) {
            case EVENT_WRITE_CTRL_REG1:
                this->set_ctrl_reg1(msg.data);
                break;
            default:
                break;
        }
    }
}

/*********************************************************************
 * @fn      set_ctrl_reg1
 *
 * @brief   Write to CTRL_REG1.
 * 
 * @param   data - Data to write.
 */
void mis2dh::set_ctrl_reg1(uint8_t data) {
    m_ctrl_reg1 = data;
    m_data_rate = (data_rate_t)(data >> 4);
}

/*********************************************************************
 * @fn      get_data_rate
 *
 * @brief   Get selected data rate.
 * 
 * @returns Data rate selection.
 */
data_rate_t mis2dh::get_data_rate() const {
    return m_data_rate;
}

/*********************************************************************
 * @fn      receiveEvent
 *
 * @brief   Receive data over I2C.
 * 
 * @param   num - Number of bytes received.
 */
void mis2dh::receiveEvent(int num) {
    Serial.println("\n/*** RECEIVE EVENT ***/");

    if (Wire.available()) {
        register_address = Wire.read();
        Serial.print("Register: ");
        Serial.println(register_address);
    }

    if (Wire.available()) {
        data = Wire.read();
        Serial.print("Data: ");
        Serial.println(data);
    }
}

/*********************************************************************
 * @fn      requestEvent
 *
 * @brief   Data requested over I2C.
 */
void mis2dh::requestEvent() {
    Serial.println("\n/*** REQUEST EVENT ***/");

    switch (register_address) {
        case MIS2DH_CTRL_REG1: {
            Serial.print("Write CTRL_REG1: ");
            Serial.println(data);

            message_t msg;
            msg.event = EVENT_WRITE_CTRL_REG1;
            msg.data = data;
            message_queue.push(msg);
            break;
        }
        default:
            break;
    }
}