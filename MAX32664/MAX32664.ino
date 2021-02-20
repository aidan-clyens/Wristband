/*****************************************************
 *  Includes
 * ***************************************************/
#include "MAX32664.h"

/*****************************************************
 *  Global Variables
 * ***************************************************/
max32664 sensor_hub;


/*********************************************************************
 * @fn      setup
 *
 * @brief   Setup Sensor Hub.
 */
void setup() {
  sensor_hub.init();

  Serial.begin(115200);
  Serial.println("Sensor Reset");
}

/*********************************************************************
 * @fn      loop
 *
 * @brief   Sensor Hub main application loop.
 */
void loop() {
  // Check if RESET pin is asserted LOW
  if (sensor_hub.checkResetPinAsserted()) {
    Serial.println("Sensor Reset");
    sensor_hub.reset();
  }

  sensor_hub.runSensorHub();
}