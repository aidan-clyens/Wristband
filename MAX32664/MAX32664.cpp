#include "MAX32664.h"

QueueArray<uint8_t> max32664::m_output_fifo(FIFO_DEPTH);
sensor_mode_t max32664::m_sensor_mode = SENSOR_MODE_NONE;
int max32664::m_fifo_interrupt_threshold = 0;
uint8_t max32664::m_sensor_hub_status = 0x0;
uint8_t max32664::m_output_mode = OUTPUT_MODE_PAUSE;
bool max32664::m_agc_enable = false;
bool max32664::m_max86141_enable = false;
int max32664::m_whrm_wspo2_mode = 0;


/*********************************************************************
 * @fn      max32664
 *
 * @brief   MAX32664 constructor.
 */
max32664::max32664() {
    m_current_millis = millis();
    m_prev_millis = millis();
}

/*********************************************************************
 * @fn      max32664
 *
 * @brief   MAX32664 destructor.
 */
max32664::~max32664() {
    Wire.end();
}

/*********************************************************************
 * @fn      init
 *
 * @brief   Initialize MAX32664.
 */
void max32664::init() {
    pinMode(MFIO, INPUT);
    pinMode(RESET, INPUT);
}

/*********************************************************************
 * @fn      reset
 *
 * @brief   Reset Sensor Hub.
 */
void max32664::reset() {
    Wire.end();

    m_sensor_mode = SENSOR_MODE_NONE;
    m_fifo_interrupt_threshold = 0;
    m_sensor_hub_status = 0x0;
    m_output_mode = OUTPUT_MODE_PAUSE;

    m_agc_enable = false;
    m_max86141_enable = false;
    m_whrm_wspo2_mode = 0;
}

/*********************************************************************
 * @fn      runSensorHub
 *
 * @brief   Run Sensor Hub main loop.
 */
void max32664::runSensorHub() {
    int reset_pin = digitalRead(RESET);
    m_current_millis = millis();

    switch (m_sensor_mode) {
        case SENSOR_MODE_NONE: {
            // RESET has been asserted, read MFIO pin to determine whether to start in Application or Bootloader mode
            if (reset_pin == LOW) {
                delay(10);
                int mfio_pin = digitalRead(MFIO);
                if (mfio_pin == HIGH) {
                    m_sensor_mode = SENSOR_MODE_APPLICATION;
                    delay(50);
                    Serial.println("Switched to Application Mode");
                    // Initialize I2C
                    Wire.begin(ADDRESS);
                    Wire.onReceive(this->receiveEvent);
                    Wire.onRequest(this->requestEvent);

                    delay(1000);
                    Serial.println("Sensor ready");
                }
            }
            break;
        }
        case SENSOR_MODE_APPLICATION: {
            // If algorithm and sensor are enabled, measure heart rate data
            if (m_max86141_enable && m_agc_enable) {
                if (m_current_millis - m_prev_millis > SAMPLE_PERIOD_MS) {
                    // Add a sample report to the FIFO
                    // Mode 1: Normal Report
                    // if (whrm_wspo2_mode) addSampleToFifoNormalReport();
                    Serial.println("Add report");
                    m_prev_millis = m_current_millis;
                }
            }
            break;
        }
        case SENSOR_MODE_BOOTLOADER:
            break;
    }
}

/*********************************************************************
 * @fn      checkResetPinAsserted
 *
 * @brief   Check if RESET pin is asserted.
 */
bool max32664::checkResetPinAsserted() const {
    int reset_pin = digitalRead(RESET);
    return (m_sensor_mode != SENSOR_MODE_NONE && reset_pin == LOW);
}

/*********************************************************************
 * @fn      receiveEvent
 *
 * @brief   Receive data over I2C.
 * 
 * @param   num - Number of bytes received.
 */
void max32664::receiveEvent(int num) {
    Serial.println("\n/*** RECEIVE EVENT ***/");

    if (Wire.available()) {
        family = Wire.read();
        Serial.print("Family: ");
        Serial.println(family);
    }

    if (Wire.available()) {
        index = Wire.read();
        Serial.print("Index: ");
        Serial.println(index);
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
void max32664::requestEvent() {
    Serial.println("\n/*** REQUEST EVENT ***/");

    int count;
    uint8_t data_buffer[2];

    switch (family) {
        case READ_SENSOR_HUB_STATUS:
            data_buffer[0] = SUCCESS;
            data_buffer[1] = m_sensor_hub_status;
            Wire.write(data_buffer, 2);
            Serial.println("Read sensor hub status");
            break;
        case SET_OUTPUT_MODE:
            switch (index) {
                // Set output format of the sensor hub
                case 0x0:
                    m_output_mode = data;
                    Serial.print("Set output mode to: ");
                    Serial.println(m_output_mode);
                    Wire.write(SUCCESS);
                    break;
                // Set threshold for the FIFO interrupts
                case 0x1:
                    m_fifo_interrupt_threshold = data;
                    Serial.print("Set FIFO interrupt threshold to: ");
                    Serial.println(m_fifo_interrupt_threshold);
                    Wire.write(SUCCESS);
                    break;
                default:
                    Wire.write(ILLEGAL_FAMILY);
                    Serial.print("\nInvalid command: ");
                    Serial.print(family);
                    break;
            }
            break;
        case READ_OUTPUT_MODE:
            switch (index) {
                case 0x0:
                    data_buffer[0] = SUCCESS;
                    data_buffer[1] = m_output_mode;
                    Wire.write(data_buffer, 2);
                    Serial.println("Read output mode");
                    break;
                default:
                    Wire.write(ILLEGAL_FAMILY);
                    Serial.print("\nInvalid command: ");
                    Serial.print(family);
                break;
            }
            break;
        case READ_OUTPUT_FIFO:
            switch (index) {
                case NUM_FIFO_SAMPLES:
                    data_buffer[0] = SUCCESS;
                    data_buffer[1] = (uint8_t)(m_output_fifo.size() / REPORT_ALGORITHM_NORMAL_SIZE);
                    Wire.write(data_buffer, 2);
                    num_samples_read = m_output_fifo.size() / REPORT_ALGORITHM_NORMAL_SIZE;
                    Serial.print("Read number of FIFO samples: ");
                    Serial.println(m_output_fifo.size() / REPORT_ALGORITHM_NORMAL_SIZE);
                    break;
                case READ_FIFO_DATA: {
                    Serial.println("Reading data from FIFO:");
                    count = num_samples_read * REPORT_ALGORITHM_NORMAL_SIZE;
                    uint8_t report_data[count+1];
                    report_data[0] = SUCCESS;
                    for (int i = 0; i < count; i++) {
                        report_data[i+1] = m_output_fifo.front();
                        m_output_fifo.pop();
                    }

                    Wire.write(report_data, count+1);
                    break;
                }
                default:
                    Wire.write(ILLEGAL_FAMILY);
                    Serial.print("\nInvalid command: ");
                    Serial.print(family);
                    break;
            }
            break;
        case SENSOR_MODE_ENABLE:
            switch (index) {
                // Enable or disable MAX86141 sensor
                case MAX86141_ENABLE:
                    m_max86141_enable = (data == 0x01);
                    Serial.print("Set MAX86141 sensor to: ");
                    Serial.println(m_max86141_enable);
                    // Delay 20 ms for value to update
                    delay(20);
                    Wire.write(SUCCESS);
                    break;
                default:
                    Wire.write(ILLEGAL_FAMILY);
                    Serial.print("\nInvalid command: ");
                    Serial.print(family);
                break;
            }
            break;
        case ALGORITHM_MODE_ENABLE:
            switch (index) {
                // Enable or disable Automatic Gain Control algorithm
                case AGC_ALGORITHM:
                    m_agc_enable = (data == 0x01);
                    Serial.print("Set AGC algorithm to: ");
                    Serial.println(m_agc_enable);
                    // Delay 20 ms for value to update
                    delay(20);
                    Wire.write(SUCCESS);
                    break;
                // Enable or disable WHRM + WSpO2 algorithm
                case WHRM_WSPO2_ALGORITHM:
                    m_whrm_wspo2_mode = data;
                    Serial.print("Set WHRM + WSpO2 algorithm to: ");
                    Serial.println(m_whrm_wspo2_mode);
                    // Delay 120 ms for value to update
                    delay(120);
                    Wire.write(SUCCESS);
                    break;
                default:
                    Wire.write(ILLEGAL_FAMILY);
                    Serial.print("\nInvalid command: ");
                    Serial.print(family);
                    break;
            }
            break;
        default:
            Wire.write(ILLEGAL_FAMILY);
            Serial.print("\nInvalid command: ");
            Serial.print(family);
            break;
    }
}