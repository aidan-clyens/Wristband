 /***  Includes  ***/
#include <Wire.h>

#include "QueueArray.h"


/***  Defines   ***/
#define SDA     A4
#define SCL     A5
#define MFIO    10
#define RESET   11

#define ADDRESS       0xAA

#define FIFO_DEPTH                  128
#define SAMPLE_PERIOD_MS            2*1000  // 25 Hz
#define REPORT_ALGORITHM_NORMAL_SIZE    20

// Family names
#define READ_SENSOR_HUB_STATUS      0x00
#define SET_OUTPUT_MODE             0x10
#define READ_OUTPUT_MODE            0x11
#define READ_OUTPUT_FIFO            0x12
#define SENSOR_MODE_ENABLE          0x44
#define ALGORITHM_MODE_ENABLE       0x52

// Sensors
#define MAX86141_ENABLE             0x00

// Algorithms
#define AGC_ALGORITHM               0x00
#define WHRM_WSPO2_ALGORITHM        0x07

// FIFO
#define NUM_FIFO_SAMPLES            0x00
#define READ_FIFO_DATA              0x01


/***  Typedefs  ***/
// Sensor Hub Operating Mode
typedef enum {
  SENSOR_MODE_NONE,
  SENSOR_MODE_APPLICATION,
  SENSOR_MODE_BOOTLOADER
} sensor_mode_t;

// Sensor Output Mode
typedef enum {
  OUTPUT_MODE_PAUSE = 0x00,
  OUTPUT_MODE_SENSOR_DATA = 0x01,
  OUTPUT_MODE_ALGORITHM_DATA = 0x02,
  OUTPUT_MODE_SENSOR_AND_ALGORITHM_DATA = 0x03,
  OUTPUT_MODE_PAUSE2 = 0x04,
  OUTPUT_MODE_SAMPLE_COUNTER_BYTE_SENSOR_DATA = 0x05,
  OUTPUT_MODE_SAMPLE_COUNTER_BYTE_ALGORITHM_DATA = 0x06,
  OUTPUT_MODE_SAMPLE_COUNTER_BYTE_SENSOR_AND_ALGORITHM_DATA = 0x07
} output_mode_t;

// Status Byte Values
typedef enum {
  SUCCESS = 0x00,
  ILLEGAL_FAMILY = 0x01,
  NOT_IMPLEMENTED = 0x02,
  INCORRECT_NUM_BYTES = 0x03,
  ILLEGAL_CONFIG = 0x04,
  INCORRECT_MODE = 0x05,
  ERROR_FLASHING = 0x80,
  CHECKSUM_ERROR = 0x81,
  AUTH_ERROR = 0x82,
  APPLICATION_INVALID = 0x83,
  DEVICE_BUSY = 0xFE,
  UNKNOWN_ERROR = 0xFF
} status_t;

// SCD State
typedef enum {
  SCD_UNDETECTED,
  SCD_OFF_SKIN,
  SCD_ON_SUBJECT,
  SCD_ON_SKIN
} scd_state_t;

/***  Global Variables  ***/
uint8_t family = 0x00;
uint8_t index = 0x00;
uint8_t data = 0x00;

sensor_mode_t sensor_mode = SENSOR_MODE_NONE;
int fifo_interrupt_threshold = 0;
uint8_t sensor_hub_status = 0x0;
uint8_t output_mode = OUTPUT_MODE_PAUSE;

bool agc_enable = false;
bool max86141_enable = false;
int whrm_wspo2_mode = 0;


QueueArray<uint8_t> fifo(FIFO_DEPTH);

int num_samples_read = 0;

unsigned long current_millis;
unsigned long prev_millis;


/***  Functions ***/
/*********************************************************************
 * @fn      receiveEvent
 *
 * @brief   Received I2C data from host.
 * 
 * @param   num - Number of bytes received.
 */
void receiveEvent(int num) {
  Serial.println("\n/***  RECEIVE   ***/");

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

  while (Wire.available()) {
    Serial.println(Wire.read());
  }
}

/*********************************************************************
 * @fn      requestEvent
 *
 * @brief   Host requested I2C data.
 */
void requestEvent() {
  Serial.println("\n/***  REQUEST   ***/");

  int count;

  switch (family) {
    case READ_SENSOR_HUB_STATUS:
      Serial.println("Read sensor hub status");
      Wire.write(SUCCESS);
      Wire.write(sensor_hub_status);
      break;
    case SET_OUTPUT_MODE:
      switch (index) {
        // Set output format of the sensor hub
        case 0x0:
          output_mode = data;
          Serial.print("Set output mode to: ");
          Serial.println(output_mode);
          Wire.write(SUCCESS);
          break;
        // Set threshold for the FIFO interrupts
        case 0x1:
          fifo_interrupt_threshold = data;
          Serial.print("Set FIFO interrupt threshold to: ");
          Serial.println(fifo_interrupt_threshold);
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
          Serial.println("Read output mode");
          Wire.write(output_mode);
          Wire.write(SUCCESS);
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
          num_samples_read = fifo.size() / REPORT_ALGORITHM_NORMAL_SIZE;
          Serial.print("Read number of FIFO samples: ");
          Serial.println(fifo.size() / REPORT_ALGORITHM_NORMAL_SIZE);
          Wire.write((uint8_t)(fifo.size() / REPORT_ALGORITHM_NORMAL_SIZE));
          Wire.write(SUCCESS);
          break;
        case READ_FIFO_DATA: {
            Serial.println("Reading data from FIFO:");
            count = num_samples_read * REPORT_ALGORITHM_NORMAL_SIZE;
            uint8_t data[count];
            for (int i = 0; i < count; i++) {
              data[i] = fifo.front();
              fifo.pop();
            }

            Wire.write(data, count);
            Wire.write(SUCCESS);
          }
          break;
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
          max86141_enable = (data == 0x01);
          Serial.print("Set MAX86141 sensor to: ");
          Serial.println(max86141_enable);
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
          agc_enable = (data == 0x01);
          Serial.print("Set AGC algorithm to: ");
          Serial.println(agc_enable);
          // Delay 20 ms for value to update
          delay(20);
          Wire.write(SUCCESS);
          break;
        // Enable or disable WHRM + WSpO2 algorithm
        case WHRM_WSPO2_ALGORITHM:
          whrm_wspo2_mode = data;
          Serial.print("Set WHRM + WSpO2 algorithm to: ");
          Serial.println(whrm_wspo2_mode);
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

/*********************************************************************
 * @fn      reset
 *
 * @brief   Reset Sensor Hub.
 */
void reset() {
  sensor_mode = SENSOR_MODE_NONE;
  fifo_interrupt_threshold = 0;
  sensor_hub_status = 0x0;
  output_mode = OUTPUT_MODE_PAUSE;

  agc_enable = false;
  max86141_enable = false;
  whrm_wspo2_mode = 0;
}

/*********************************************************************
 * @fn      addSampleToFifoNormalReport
 *
 * @brief   Add WHRM + WSpO2 algorithm data to FIFO. Create Normal Report.
 */
void addSampleToFifoNormalReport() {
  if (fifo.size() == FIFO_DEPTH) {
    // TODO: Update FIFO overflow flag
    return;
  }

  uint16_t heart_rate = 100;
  uint8_t heart_rate_confidence = 90;
  uint16_t spo2 = 50;
  scd_state_t scd_state = SCD_ON_SKIN;
  uint8_t spo2_quality = 0;  // Good signal quality
  uint8_t spo2_motion = 0; // No motion

  // Generate report
//  uint8_t report[] = {
//    0,  // Current operation mode
//    (uint8_t)((heart_rate >> 8) & 0xFF),  // HR MSB
//    (uint8_t)(heart_rate & 0xFF), // HR LSB
//    heart_rate_confidence, // HR confidence
//    0,  // RR MSB
//    0,  // RR LSB
//    0,  // RR confidence
//    0,  // Activity class
//    0,  // R MSB
//    0,  // R LSB
//    0,  // SpO2 confidence
//    (uint8_t)((spo2 >> 8) & 0xFF),  // SpO2 MSB
//    (uint8_t)(spo2 & 0xFF), // SpO2 LSB
//    0,  // SpO2 % complete
//    spo2_quality,  // SpO2 low signal quality flag
//    spo2_motion,  // SpO2 motion flag
//    0,  // SpO2 low PI flag
//    0,  // SpO2 unreliable R flag
//    0,  // SpO2 state
//    (uint8_t)scd_state // SCD state
//  };
  uint8_t report[REPORT_ALGORITHM_NORMAL_SIZE];
  for (int i = 0; i < REPORT_ALGORITHM_NORMAL_SIZE; i++) {
    report[i] = 0;
  }

  // Add report to FIFO
  for (int i = 0; i < REPORT_ALGORITHM_NORMAL_SIZE; i++) {
    if (fifo.size() == FIFO_DEPTH) {
      // TODO: Update FIFO overflow flag
      Serial.println("Output FIFO full");
      return;
    }

    fifo.push(report[i]);
  }

  // TODO: If number of reports exceeds threshold, set flag

  Serial.println("Added WHRM + WSpO2 report to Output FIFO");
}

/*********************************************************************
 * @fn      setup
 *
 * @brief   Setup Sensor Hub.
 */
void setup() {
  pinMode(MFIO, INPUT);
  pinMode(RESET, INPUT);

  current_millis = millis();
  prev_millis = millis();
  
  Serial.begin(115200);
  Serial.println("Sensor Reset");
}

/*********************************************************************
 * @fn      loop
 *
 * @brief   Sensor Hub main application loop.
 */
void loop() {
  current_millis = millis();

  // Check if RESET pin is asserted LOW
  int reset_pin = digitalRead(RESET);
  if (sensor_mode != SENSOR_MODE_NONE && reset_pin == LOW) {
    if (sensor_mode == SENSOR_MODE_APPLICATION) {
      Wire.end();
    }
    reset();
    Serial.println("Sensor Reset");
  }

  switch (sensor_mode) {
    case SENSOR_MODE_NONE:
      // RESET has been asserted, read MFIO pin to determine whether to start in Application or Bootloader mode
      if (reset_pin == LOW) {
        delay(10);
        int mfio_pin = digitalRead(MFIO);
        if (mfio_pin == HIGH) {
          sensor_mode = SENSOR_MODE_APPLICATION;
          delay(50);
          Serial.println("Switched to Application Mode");
          // Initialize I2C
          Wire.begin(ADDRESS);
          Wire.onReceive(receiveEvent);
          Wire.onRequest(requestEvent);

          delay(1000);
          Serial.println("Sensor ready");
        }
      }
      break;
    case SENSOR_MODE_APPLICATION:
      // If algorithm and sensor are enabled, measure heart rate data
      if (max86141_enable && agc_enable) {
        if (current_millis - prev_millis > SAMPLE_PERIOD_MS) {
          // Add a sample report to the FIFO
          // Mode 1: Normal Report
          if (whrm_wspo2_mode) addSampleToFifoNormalReport();
          prev_millis = current_millis;
        }
      }
      break;
    case SENSOR_MODE_BOOTLOADER:
      break;
  }
}