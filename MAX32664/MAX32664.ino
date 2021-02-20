/*****************************************************
 *  Includes
 * ***************************************************/
#include "MAX32664.h"

/*****************************************************
 *  Global Variables
 * ***************************************************/
max32664 sensor_hub;

// SCD State
typedef enum {
  SCD_UNDETECTED,
  SCD_OFF_SKIN,
  SCD_ON_SUBJECT,
  SCD_ON_SKIN
} scd_state_t;

// /*********************************************************************
//  * @fn      addSampleToFifoNormalReport
//  *
//  * @brief   Add WHRM + WSpO2 algorithm data to FIFO. Create Normal Report.
//  */
// void addSampleToFifoNormalReport() {
//   if (fifo.size() == FIFO_DEPTH) {
//     // TODO: Update FIFO overflow flag
//     return;
//   }

//   uint16_t heart_rate = 100;
//   uint8_t heart_rate_confidence = 90;
//   uint16_t spo2 = 50;
//   scd_state_t scd_state = SCD_ON_SKIN;
//   uint8_t spo2_quality = 0;  // Good signal quality
//   uint8_t spo2_motion = 0; // No motion

//   // Generate report
// //  uint8_t report[] = {
// //    0,  // Current operation mode
// //    (uint8_t)((heart_rate >> 8) & 0xFF),  // HR MSB
// //    (uint8_t)(heart_rate & 0xFF), // HR LSB
// //    heart_rate_confidence, // HR confidence
// //    0,  // RR MSB
// //    0,  // RR LSB
// //    0,  // RR confidence
// //    0,  // Activity class
// //    0,  // R MSB
// //    0,  // R LSB
// //    0,  // SpO2 confidence
// //    (uint8_t)((spo2 >> 8) & 0xFF),  // SpO2 MSB
// //    (uint8_t)(spo2 & 0xFF), // SpO2 LSB
// //    0,  // SpO2 % complete
// //    spo2_quality,  // SpO2 low signal quality flag
// //    spo2_motion,  // SpO2 motion flag
// //    0,  // SpO2 low PI flag
// //    0,  // SpO2 unreliable R flag
// //    0,  // SpO2 state
// //    (uint8_t)scd_state // SCD state
// //  };
//   uint8_t report[REPORT_ALGORITHM_NORMAL_SIZE];
//   for (int i = 0; i < REPORT_ALGORITHM_NORMAL_SIZE; i++) {
//     report[i] = 0;
//   }

//   // Add report to FIFO
//   for (int i = 0; i < REPORT_ALGORITHM_NORMAL_SIZE; i++) {
//     if (fifo.size() == FIFO_DEPTH) {
//       // TODO: Update FIFO overflow flag
//       Serial.println("Output FIFO full");
//       return;
//     }

//     fifo.push(report[i]);
//   }

//   // TODO: If number of reports exceeds threshold, set flag

//   Serial.println("Added WHRM + WSpO2 report to Output FIFO");
// }

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