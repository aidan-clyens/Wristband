#include "MIS2DH.h"

/*********************************************************************
 * @fn      mis2sh
 *
 * @brief   MIS2DH constructor.
 */
mis2dh::mis2dh():
m_fifo(FIFO_DEPTH),
m_power_mode(MODE_LOW_POWER)
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
 * @fn      receiveEvent
 *
 * @brief   Receive data over I2C.
 * 
 * @param   num - Number of bytes received
 */
void mis2dh::receiveEvent(int num)
{
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
        default:
            break;
    }
}