#include "MIS2DH.h"

/*********************************************************************
 * @fn      mis2sh
 *
 * @brief   MIS2DH constructor.
 */
mis2dh::mis2dh() : m_fifo(FIFO_DEPTH),
                   m_ctrl_reg1(0x0),
                   m_ctrl_reg5(0x0),
                   m_fifo_ctrl_reg(0x0),
                   m_fifo_mode(FIFO_MODE_BYPASS),
                   m_data_rate(DATARATE_POWER_DOWN),
                   m_low_power_mode(false),
                   m_high_resolution_mode(false),
                   m_fifo_enabled(false)
{
    m_current_ms = millis();
    m_prev_ms = m_current_ms;

    m_sensor_period_ms = this->get_period_from_data_rate();
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
            case EVENT_READ:
                switch (msg.register_address) {
                    case MIS2DH_CTRL_REG1:
                        this->set_ctrl_reg1(msg.data);
                        break;
                    case MIS2DH_CTRL_REG5:
                        this->set_ctrl_reg5(msg.data);
                        break;
                    case MIS2DH_FIFO_CTRL_REG:
                        this->set_fifo_ctrl_reg(msg.data);
                        break;
                }
                break;
            case EVENT_WRITE:
                switch (msg.register_address) {
                    case MIS2DH_CTRL_REG1:
                        Wire.write(m_ctrl_reg1);
                        break;
                    case MIS2DH_CTRL_REG5:
                        Wire.write(m_ctrl_reg5);
                        break;
                    case MIS2DH_FIFO_CTRL_REG:
                        Wire.write(m_fifo_ctrl_reg);
                        break;
                }
                break;
            default:
                break;
        }
    }
}

/*********************************************************************
 * @fn      read_accelerometer
 *
 * @brief   Read accelerometer data. 
 */
void mis2dh::read_accelerometer() {
    m_current_ms = millis();

    // Read accelerometer data
    if (m_data_rate != DATARATE_POWER_DOWN) {
        if (m_current_ms - m_prev_ms > m_sensor_period_ms) {
            // TODO: Add data to FIFO
            Serial.println("Read accelerometer data");
            m_prev_ms = m_current_ms;
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
    m_data_rate = (data_rate_t)(m_ctrl_reg1 >> 4);

    m_low_power_mode = (m_ctrl_reg1 & 0x8) > 0;
}

/*********************************************************************
 * @fn      set_ctrl_reg5
 *
 * @brief   Write to CTRL_REG5.
 * 
 * @param   data - Data to write.
 */
void mis2dh::set_ctrl_reg5(uint8_t data) {
    m_ctrl_reg5 = data;

    m_fifo_enabled = (m_ctrl_reg5 & 0x40) > 0;
}

/*********************************************************************
 * @fn      set_fifo_ctrl_reg
 *
 * @brief   Write to FIFO_CTRL_REG.
 * 
 * @param   data - Data to write.
 */
void mis2dh::set_fifo_ctrl_reg(uint8_t data) {
    m_fifo_ctrl_reg = data;

    m_fifo_mode = (fifo_mode_t)(m_fifo_ctrl_reg >> 6);
}

/*********************************************************************
 * @fn      get_power_mode
 *
 * @brief   Get current power mode.
 * 
 * @returns Current power mode.
 */
power_mode_t mis2dh::get_power_mode() const {
    if (m_low_power_mode) {
        return MODE_LOW_POWER;
    }

    if (m_high_resolution_mode) {
        return MODE_HIGH_RESOLUTION;
    }

    return MODE_NORMAL;
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
 * @fn      get_fifo_mode
 *
 * @brief   Get current FIFO mode.
 * 
 * @returns Current FIFO mode.
 */
fifo_mode_t mis2dh::get_fifo_mode() const {
    return m_fifo_mode;
}

/*********************************************************************
 * @fn      is_fifo_enabled
 *
 * @brief   Check whether FIFO is enabled or not.
 * 
 * @returns Boolean indicating whether FIFO is enabled or not.
 */
bool mis2dh::is_fifo_enabled() const {
    return m_fifo_enabled;
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

    // Read incoming data
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

    // Read
    if (num == 1) {
        Serial.println("READ");
    }
    // Write
    else if (num == 2) {
        Serial.println("WRITE");
        message_t msg;
        msg.event = EVENT_WRITE;
        msg.register_address = register_address;
        msg.data = data;

        message_queue.push(msg);
    }
}

/*********************************************************************
 * @fn      requestEvent
 *
 * @brief   Data requested over I2C.
 */
void mis2dh::requestEvent() {
    Serial.println("\n/*** REQUEST EVENT ***/");

    message_t msg;
    msg.event = EVENT_READ;
    msg.register_address = register_address;
    message_queue.push(msg);
}

/*********************************************************************
 * @fn      get_period_from_data_rate
 *
 * @brief   Get the sensor period from the data rate.
 * 
 * @returns Period in ms.
 */
unsigned long mis2dh::get_period_from_data_rate() const {
    int frequency;

    switch (m_data_rate) {
        case DATARATE_POWER_DOWN:
            return 100000000;
        case DATARATE_1HZ:
            frequency = 1;
            break;
        case DATARATE_10HZ:
            frequency = 10;
            break;
        case DATARATE_25HZ:
            frequency = 25;
            break;
        case DATARATE_50HZ:
            frequency = 50;
            break;
        case DATARATE_100HZ:
            frequency = 100;
            break;
        case DATARATE_200HZ:
            frequency = 200;
            break;
        case DATARATE_400HZ:
            frequency = 400;
            break;
        case DATARATE_1620HZ:
            frequency = 1620;
            break;
        case DATARATE_5376HZ:
            frequency = 5376;
            break;
    }

    return (1 / frequency) * 1000;
}