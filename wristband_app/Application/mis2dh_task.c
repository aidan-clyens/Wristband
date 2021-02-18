/*
 * mis2dh_task.c
 *
 *  Created on: Feb 17, 2021
 *      Author: Aidan Clyens
 */

/*******************************************************************************
 * INCLUDES
 */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/drivers/I2C.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <uartlog/UartLog.h>

#include <Board.h>

#include <icall.h>
#include <mis2dh_task.h>
#include <i2c_util.h>
#include <util.h>

/*********************************************************************
 * CONSTANTS
 */
// Task
#define MIS2DH_THREAD_STACK_SIZE        1024
#define MIS2DH_TASK_PRIORITY            1

#define MIS2DH_CLOCK_PERIOD_MS          5*1000

// I2C
#define MIS2DH_ADDRESS                  0x18

// Registers
#define MIS2DH_CTRL_REG1                0x20

/*********************************************************************
 * TYPEDEFS
 */
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

/*********************************************************************
 * GLOBAL VARIABLES
 */
Task_Struct mis2dhTask;
uint8_t mis2dhTaskStack[MIS2DH_THREAD_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clocks
static Clock_Struct clock;
static Clock_Handle clockHandle;

// Semaphores
static Semaphore_Handle semaphore;

// I2C
static I2C_Transaction transaction;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Mis2dh_init(void);
static void Mis2dh_taskFxn(UArg a0, UArg a1);
static void Mis2dh_clockSwiFxn(UArg a0);

static bool Mis2dh_configureDataRate(data_rate_t dataRate);

// I2C
static bool Mis2dh_writeRegister(uint8_t regAddress, uint8_t data);
static bool Mis2dh_readRegister(uint8_t regAddress, uint8_t *data);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Mis2dh_createTask
 *
 * @brief   Task creation function for the MIS2DH Accelerometer.
 */
void Mis2dh_createTask(void) {
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = mis2dhTaskStack;
    taskParams.stackSize = MIS2DH_THREAD_STACK_SIZE;
    taskParams.priority = MIS2DH_TASK_PRIORITY;

    Task_construct(&mis2dhTask, Mis2dh_taskFxn, &taskParams, Error_IGNORE);
}

/*********************************************************************
 * @fn      Mis2dh_inits
 *
 * @brief   Initialization for MIS2DH task.
 */
static void Mis2dh_init(void) {
    // Set I2C slave address
    transaction.slaveAddress = MIS2DH_ADDRESS;

    // Create semaphores
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semaphore = Semaphore_create(0, &semParams, Error_IGNORE);

    // Set data rate
    if (Mis2dh_configureDataRate(DATARATE_10HZ)) {
        Log_info0("Set data rate to 10Hz");
    }
    else {
        Log_error0("Error setting data rate. Exiting");
        Task_exit();
    }

    // Create clocks
    clockHandle = Util_constructClock(&clock,
                                      Mis2dh_clockSwiFxn,
                                      MIS2DH_CLOCK_PERIOD_MS,
                                      MIS2DH_CLOCK_PERIOD_MS,
                                      1,
                                      NULL
    );
}

/*********************************************************************
 * @fn      Mis2dh_taskFxn
 *
 * @brief   Application task entry point for the MIS2DH Accelerometer.
 *
 * @param   a0, a1 - not used.
 */
static void Mis2dh_taskFxn(UArg a0, UArg a1) {
    Mis2dh_init();

    for (;;) {
        Semaphore_pend(semaphore, BIOS_WAIT_FOREVER);
    }
}

/*********************************************************************
 * @fn      Mis2dh_clockSwiFxn
 *
 * @brief   Software interrupt for clock.
 *
 * @param   a0 - not used.
 */
static void Mis2dh_clockSwiFxn(UArg a0) {
    Semaphore_post(semaphore);
}

/*********************************************************************
 * @fn      Mis2dh_writeRegister
 *
 * @brief   Write data to register.
 *
 * @param   reg_address - Address of register.
 *          data - Data to write to register.
 */
static bool Mis2dh_writeRegister(uint8_t regAddress, uint8_t data) {
    uint8_t txBuffer[1];
    txBuffer[0] = regAddress;
    txBuffer[1] = data;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 2;
    transaction.readBuf    = NULL;
    transaction.readCount  = 0;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Mis2dh_readRegister
 *
 * @brief   Write data to register.
 *
 * @param   reg_address - Address of register.
 *          data - Variable to read data into
 */
static bool Mis2dh_readRegister(uint8_t regAddress, uint8_t *data) {
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    txBuffer[0] = regAddress;

    transaction.writeBuf   = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf    = rxBuffer;
    transaction.readCount  = 1;

    if (Util_i2cTransfer(&transaction)) {
        Log_info0("I2C transfer successful");
        (*data) = rxBuffer[0];
        return true;
    }
    else {
        Log_error0("I2C transfer failed");
        return false;
    }
}

/*********************************************************************
 * @fn      Mis2dh_configureDataRate
 *
 * @brief   Set MIS2DH data rate.
 *
 * @param   dataRate - Data rate for accelerometer reading.
 */
static bool Mis2dh_configureDataRate(data_rate_t dataRate) {
    uint8_t ctrl_reg1_data;

    if (!Mis2dh_readRegister(MIS2DH_CTRL_REG1, &ctrl_reg1_data)) {
        return false;
    }

    // Clear data rate and write new value
    ctrl_reg1_data &= 0x0F;
    ctrl_reg1_data |= (dataRate << 4);

    return Mis2dh_writeRegister(MIS2DH_CTRL_REG1, ctrl_reg1_data);
}
