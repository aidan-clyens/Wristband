#include "MIS2DH.h"

QueueArray<sensor_data_t> mis2dh::m_fifo(FIFO_DEPTH);
uint8_t mis2dh::m_ctrl_reg1 = 0x0;
uint8_t mis2dh::m_ctrl_reg5 = 0x0;
uint8_t mis2dh::m_fifo_ctrl_reg = 0x0;
uint8_t mis2dh::m_fifo_src_reg = 0x0;


/*********************************************************************
 * @fn      mis2sh
 *
 * @brief   MIS2DH constructor.
 */
mis2dh::mis2dh() : m_fifo_mode(FIFO_MODE_BYPASS),
                   m_data_rate(DATARATE_POWER_DOWN),
                   m_low_power_mode(false),
                   m_high_resolution_mode(false),
                   m_fifo_enabled(false)
{
    m_current_ms = millis();
    m_prev_ms = m_current_ms;

    m_sensor_period_ms = this->get_period_from_data_rate();

    m_ctrl_reg1 |= (m_data_rate << 4);
    if (m_low_power_mode) m_ctrl_reg1 |= 0x08;
    if (m_fifo_enabled) m_ctrl_reg5 |= 0x40;
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
            sensor_data_t data;
            data.x_L = 10;
            data.x_H = 11;
            data.y_L = 12;
            data.y_H = 13;
            data.z_L = 14;
            data.z_H = 15;
            
            // Add data to FIFO
            if (m_fifo_enabled && m_fifo_mode != FIFO_MODE_BYPASS) {
                this->push_fifo(data);
            }

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
    m_sensor_period_ms = this->get_period_from_data_rate();

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
 * @fn      set_fifo_src_reg
 *
 * @brief   Write to FIFO_SRC_REG.
 * 
 * @param   data - Data to write.
 */
void mis2dh::set_fifo_src_reg(uint8_t data) {
    m_fifo_src_reg = data;
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

    switch (register_address) {
        case MIS2DH_CTRL_REG1:
            Wire.write(m_ctrl_reg1);
            Serial.println(m_ctrl_reg1);
            break;
        case MIS2DH_CTRL_REG5:
            Wire.write(m_ctrl_reg5);
            Serial.println(m_ctrl_reg5);
            break;
        case MIS2DH_FIFO_CTRL_REG:
            Wire.write(m_fifo_ctrl_reg);
            Serial.println(m_fifo_ctrl_reg);
            break;
        case MIS2DH_OUT_X_L:
        case MIS2DH_OUT_X_H:
        case MIS2DH_OUT_Y_L:
        case MIS2DH_OUT_Y_H:
        case MIS2DH_OUT_Z_L:
        case MIS2DH_OUT_Z_H: {
            if (m_fifo.empty()) break;

            sensor_data_t data = pop_fifo();

            Wire.write(data.x_L);
            Wire.write(data.x_H);
            Wire.write(data.y_L);
            Wire.write(data.y_H);
            Wire.write(data.z_L);
            Wire.write(data.z_H);

            Serial.print("XL: ");
            Serial.print(data.x_L);
            Serial.print(" XH: ");
            Serial.print(data.x_H);
            Serial.print(" YL: ");
            Serial.print(data.y_L);
            Serial.print(" YH: ");
            Serial.print(data.y_H);
            Serial.print(" ZL: ");
            Serial.print(data.z_L);
            Serial.print(" ZH: ");
            Serial.println(data.z_H);
            break;
        }
        case MIS2DH_FIFO_SRC_REG:
            Wire.write(m_fifo_src_reg);
            Serial.println(m_fifo_src_reg);
            break;
    }
}

/*********************************************************************
 * @fn      pop_fifo
 *
 * @brief   Get the first sample from the FIFO.
 * 
 * @returns Sensor data.
 */
sensor_data_t mis2dh::pop_fifo() {
    // If OVRN bit is set high, reset after reading sample
    m_fifo_src_reg &= ~0x40;

    // Decrement FSS (number of unread samples)
    int num_samples = m_fifo_src_reg & 0x1F;
    if (num_samples > 0) num_samples--;
    m_fifo_src_reg |= (num_samples & 0x1F);

    sensor_data_t data = m_fifo.front();
    m_fifo.pop();

    // Set EMPTY bit if FIFO is empty
    m_fifo_src_reg &= ~0x20;
    if (m_fifo.empty()) {
        m_fifo_src_reg |= 0x20;
    }

    return data;
}

/*********************************************************************
 * @fn      push_fifo
 *
 * @brief   Push data to the FIFO.
 * 
 * @param   data - Sensor data.
 */
void mis2dh::push_fifo(sensor_data_t data) {
    m_fifo.push(data);
    // Increment FSS (number of unread samples)
    int num_samples = m_fifo_src_reg & 0x1F;
    num_samples++;
    m_fifo_src_reg |= (num_samples & 0x1F);

    // Set OVRN bit if FIFO is full
    m_fifo_src_reg &= ~0x40;
    if (m_fifo.size() == FIFO_DEPTH) {
        m_fifo_src_reg |= 0x40;
    }
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