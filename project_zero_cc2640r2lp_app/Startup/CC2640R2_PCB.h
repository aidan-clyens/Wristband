/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CC2640R2_PCB.h
 *
 *  @brief      CC2640R2 LaunchPad Board Specific header file.
 *
 *  This file is responsible for setting up the board specific items for the
 *  CC2640R2_PCB board.
 *
 *  This board file is made for the 7x7 mm QFN package, to convert this board
 *  file to use for other smaller device packages please refer to the table
 *  below which lists the max IOID values supported by each package. All other
 *  unused pins should be set to IOID_UNUSED.
 *
 *  Furthermore the board file is also used
 *  to define a symbol that configures the RF front end and bias.
 *  See the comments below for more information.
 *  For an in depth tutorial on how to create a custom board file, please refer
 *  to the section "Running the SDK on Custom Boards" with in the Software
 *  Developer's Guide.
 *
 *  Refer to the datasheet for all the package options and IO descriptions:
 *  http://www.ti.com/lit/ds/symlink/cc2640r2f.pdf
 *
 *  +-----------------------+------------------+-----------------------+
 *  |     Package Option    |  Total GPIO Pins |   MAX IOID            |
 *  +=======================+==================+=======================+
 *  |     7x7 mm QFN        |     31           |   IOID_30             |
 *  +-----------------------+------------------+-----------------------+
 *  |     5x5 mm QFN        |     15           |   IOID_14             |
 *  +-----------------------+------------------+-----------------------+
 *  |     4x4 mm QFN        |     10           |   IOID_9              |
 *  +-----------------------+------------------+-----------------------+
 *  |     2.7 x 2.7 mm WCSP |     14           |   IOID_13             |
 *  +-----------------------+------------------+-----------------------+
 *  ============================================================================
 */
#ifndef __CC2640R2_PCB_BOARD_H__
#define __CC2640R2_PCB_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#ifndef CC2640R2_PCB
  #define CC2640R2_PCB
#endif /* CC2640R2_PCB */

/*
 *  ============================================================================
 *  RF Front End and Bias configuration symbols for TI reference designs and
 *  kits. This symbol sets the RF Front End configuration in ble_user_config.h
 *  and selects the appropriate PA table in ble_user_config.c.
 *  Other configurations can be used by editing these files.
 *
 *  Define only one symbol:
 *  CC2650EM_7ID    - Differential RF and internal biasing
                      (default for CC2640R2 LaunchPad)
 *  CC2650EM_5XD    – Differential RF and external biasing
 *  CC2650EM_4XS    – Single-ended RF on RF-P and external biasing
 *  CC2640R2DK_CXS  - WCSP: Single-ended RF on RF-N and external biasing
 *                    (Note that the WCSP is only tested and characterized for
 *                     single ended configuration, and it has a WCSP-specific
 *                     PA table)
 *
 *  Note: CC2650EM_xxx reference designs apply to all CC26xx devices.
 *  ==========================================================================
 */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                  <pin mapping>
 */

/* Analog Capable DIOs */
#define CC2640R2_PCB_DIO23_ANALOG          IOID_23
#define CC2640R2_PCB_DIO24_ANALOG          IOID_24
#define CC2640R2_PCB_DIO25_ANALOG          IOID_25
#define CC2640R2_PCB_DIO26_ANALOG          IOID_26
#define CC2640R2_PCB_DIO27_ANALOG          IOID_27
#define CC2640R2_PCB_DIO28_ANALOG          IOID_28
#define CC2640R2_PCB_DIO29_ANALOG          IOID_29
#define CC2640R2_PCB_DIO30_ANALOG          IOID_30

/* Digital IOs */
#define CC2640R2_PCB_DIO0                  IOID_0
#define CC2640R2_PCB_DIO1_RFSW             IOID_1
#define CC2640R2_PCB_DIO12                 IOID_12
#define CC2640R2_PCB_DIO15                 IOID_15
#define CC2640R2_PCB_DIO16_TDO             IOID_16
#define CC2640R2_PCB_DIO17_TDI             IOID_17
#define CC2640R2_PCB_DIO21                 IOID_21
#define CC2640R2_PCB_DIO22                 IOID_22

/* Discrete Inputs */
#define CC2640R2_PCB_PIN_BTN1              IOID_23

/* GPIO */
#define CC2640R2_PCB_GPIO_LED_ON           1
#define CC2640R2_PCB_GPIO_LED_OFF          0

/* I2C */
#define CC2640R2_PCB_I2C0_SCL0             IOID_7
#define CC2640R2_PCB_I2C0_SDA0             IOID_6

/* I2S */
#define CC2640R2_PCB_I2S_ADO               IOID_0
#define CC2640R2_PCB_I2S_ADI               IOID_1
#define CC2640R2_PCB_I2S_BCLK              IOID_30
#define CC2640R2_PCB_I2S_MCLK              PIN_UNASSIGNED
#define CC2640R2_PCB_I2S_WCLK              IOID_29

/* LEDs */
#define CC2640R2_PCB_PIN_LED_ON            1
#define CC2640R2_PCB_PIN_LED_OFF           0
#define CC2640R2_PCB_PIN_RLED              IOID_21
#define CC2640R2_PCB_PIN_GLED              IOID_22

/* PWM Outputs */
#define CC2640R2_PCB_PWMPIN0               CC2640R2_PCB_PIN_RLED
#define CC2640R2_PCB_PWMPIN1               CC2640R2_PCB_PIN_GLED
#define CC2640R2_PCB_PWMPIN2               PIN_UNASSIGNED
#define CC2640R2_PCB_PWMPIN3               PIN_UNASSIGNED
#define CC2640R2_PCB_PWMPIN4               PIN_UNASSIGNED
#define CC2640R2_PCB_PWMPIN5               PIN_UNASSIGNED
#define CC2640R2_PCB_PWMPIN6               PIN_UNASSIGNED
#define CC2640R2_PCB_PWMPIN7               PIN_UNASSIGNED

/* SPI */
#define CC2640R2_PCB_SPI_FLASH_CS          IOID_20
#define CC2640R2_PCB_FLASH_CS_ON           0
#define CC2640R2_PCB_FLASH_CS_OFF          1

/* SPI Board */
#define CC2640R2_PCB_SPI0_MISO             IOID_8          /* RF1.20 */
#define CC2640R2_PCB_SPI0_MOSI             IOID_9          /* RF1.18 */
#define CC2640R2_PCB_SPI0_CLK              IOID_10         /* RF1.16 */
#define CC2640R2_PCB_SPI0_CSN              PIN_UNASSIGNED
#define CC2640R2_PCB_SPI1_MISO             PIN_UNASSIGNED
#define CC2640R2_PCB_SPI1_MOSI             PIN_UNASSIGNED
#define CC2640R2_PCB_SPI1_CLK              PIN_UNASSIGNED
#define CC2640R2_PCB_SPI1_CSN              PIN_UNASSIGNED

/* UART Board */
#define CC2640R2_PCB_UART_RX               IOID_2          /* RXD */
#define CC2640R2_PCB_UART_TX               IOID_3          /* TXD */
#define CC2640R2_PCB_UART_CTS              IOID_19         /* CTS */
#define CC2640R2_PCB_UART_RTS              IOID_18         /* RTS */

/* MAX32664 */
#define CC2640R2_PCB_MAX32664_RESET        IOID_21         /* Reset */
#define CC2640R2_PCB_MAX32664_MFIO         IOID_15         /* MFIO */

/* LIS3DH */
#define CC2640R2_PCB_LIS3DH_CS             IOID_12         /* Chip Select */
#define CC2640R2_PCB_LIS3DH_INT1           IOID_22         /* INT 1 */


/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC2640R2_PCB_initGeneral(void);

/*!
 *  @brief  Turn off the external flash on LaunchPads
 *
 */
void CC2640R2_PCB_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void CC2640R2_PCB_wakeUpExtFlash(void);

/*!
 *  @def    CC2640R2_PCB_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum CC2640R2_PCB_ADCBufName {
    CC2640R2_PCB_ADCBUF0 = 0,

    CC2640R2_PCB_ADCBUFCOUNT
} CC2640R2_PCB_ADCBufName;

/*!
 *  @def    CC2640R2_PCB_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC2640R2_PCB_ADCBuf0ChannelName {
    CC2640R2_PCB_ADCBUF0CHANNEL0 = 0,
    CC2640R2_PCB_ADCBUF0CHANNEL1,
    CC2640R2_PCB_ADCBUF0CHANNEL2,
    CC2640R2_PCB_ADCBUF0CHANNEL3,
    CC2640R2_PCB_ADCBUF0CHANNEL4,
    CC2640R2_PCB_ADCBUF0CHANNEL5,
    CC2640R2_PCB_ADCBUF0CHANNEL6,
    CC2640R2_PCB_ADCBUF0CHANNEL7,
    CC2640R2_PCB_ADCBUF0CHANNELVDDS,
    CC2640R2_PCB_ADCBUF0CHANNELDCOUPL,
    CC2640R2_PCB_ADCBUF0CHANNELVSS,

    CC2640R2_PCB_ADCBUF0CHANNELCOUNT
} CC2640R2_PCB_ADCBuf0ChannelName;

/*!
 *  @def    CC2640R2_PCB_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC2640R2_PCB_ADCName {
    CC2640R2_PCB_ADC0 = 0,
    CC2640R2_PCB_ADC1,
    CC2640R2_PCB_ADC2,
    CC2640R2_PCB_ADC3,
    CC2640R2_PCB_ADC4,
    CC2640R2_PCB_ADC5,
    CC2640R2_PCB_ADC6,
    CC2640R2_PCB_ADC7,
    CC2640R2_PCB_ADCDCOUPL,
    CC2640R2_PCB_ADCVSS,
    CC2640R2_PCB_ADCVDDS,

    CC2640R2_PCB_ADCCOUNT
} CC2640R2_PCB_ADCName;

/*!
 *  @def    CC2640R2_PCB_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC2640R2_PCB_CryptoName {
    CC2640R2_PCB_CRYPTO0 = 0,

    CC2640R2_PCB_CRYPTOCOUNT
} CC2640R2_PCB_CryptoName;

/*!
 *  @def    CC2640R2_PCB_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC2640R2_PCB_AESCCMName {
    CC2640R2_PCB_AESCCM0 = 0,

    CC2640R2_PCB_AESCCMCOUNT
} CC2640R2_PCB_AESCCMName;

/*!
 *  @def    CC2640R2_PCB_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC2640R2_PCB_AESGCMName {
    CC2640R2_PCB_AESGCM0 = 0,

    CC2640R2_PCB_AESGCMCOUNT
} CC2640R2_PCB_AESGCMName;

/*!
 *  @def    CC2640R2_PCB_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC2640R2_PCB_AESCBCName {
    CC2640R2_PCB_AESCBC0 = 0,

    CC2640R2_PCB_AESCBCCOUNT
} CC2640R2_PCB_AESCBCName;

/*!
 *  @def    CC2640R2_PCB_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC2640R2_PCB_AESCTRName {
    CC2640R2_PCB_AESCTR0 = 0,

    CC2640R2_PCB_AESCTRCOUNT
} CC2640R2_PCB_AESCTRName;

/*!
 *  @def    CC2640R2_PCB_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC2640R2_PCB_AESECBName {
    CC2640R2_PCB_AESECB0 = 0,

    CC2640R2_PCB_AESECBCOUNT
} CC2640R2_PCB_AESECBName;

/*!
 *  @def    CC2640R2_PCB_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC2640R2_PCB_AESCTRDRBGName {
    CC2640R2_PCB_AESCTRDRBG0 = 0,

    CC2640R2_PCB_AESCTRDRBGCOUNT
} CC2640R2_PCB_AESCTRDRBGName;

/*!
 *  @def    CC2640R2_PCB_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC2640R2_PCB_GPIOName {
    CC2640R2_PCB_GPIO_S1 = 0,
    CC2640R2_PCB_GPIO_S2,
    CC2640R2_PCB_SPI_MASTER_READY,
    CC2640R2_PCB_SPI_SLAVE_READY,
    CC2640R2_PCB_GPIO_LED_GREEN,
    CC2640R2_PCB_GPIO_LED_RED,
    CC2640R2_PCB_GPIO_TMP116_EN,
    CC2640R2_PCB_GPIO_SPI_FLASH_CS,
    CC2640R2_PCB_SDSPI_CS,
    CC2640R2_PCB_GPIO_LCD_CS,
    CC2640R2_PCB_GPIO_LCD_POWER,
    CC2640R2_PCB_GPIO_LCD_ENABLE,
    CC2640R2_PCB_GPIOCOUNT
} CC2640R2_PCB_GPIOName;

/*!
 *  @def    CC2640R2_PCB_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2640R2_PCB_GPTimerName {
    CC2640R2_PCB_GPTIMER0A = 0,
    CC2640R2_PCB_GPTIMER0B,
    CC2640R2_PCB_GPTIMER1A,
    CC2640R2_PCB_GPTIMER1B,
    CC2640R2_PCB_GPTIMER2A,
    CC2640R2_PCB_GPTIMER2B,
    CC2640R2_PCB_GPTIMER3A,
    CC2640R2_PCB_GPTIMER3B,

    CC2640R2_PCB_GPTIMERPARTSCOUNT
} CC2640R2_PCB_GPTimerName;

/*!
 *  @def    CC2640R2_PCB_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2640R2_PCB_GPTimers {
    CC2640R2_PCB_GPTIMER0 = 0,
    CC2640R2_PCB_GPTIMER1,
    CC2640R2_PCB_GPTIMER2,
    CC2640R2_PCB_GPTIMER3,

    CC2640R2_PCB_GPTIMERCOUNT
} CC2640R2_PCB_GPTimers;

/*!
 *  @def    CC2640R2_PCB_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC2640R2_PCB_I2CName {
    CC2640R2_PCB_I2C0 = 0,

    CC2640R2_PCB_I2CCOUNT
} CC2640R2_PCB_I2CName;

/*!
 *  @def    CC2640R2_PCB_I2SName
 *  @brief  Enum of I2S names
 */
typedef enum CC2640R2_PCB_I2SName {
    CC2640R2_PCB_I2S0 = 0,

    CC2640R2_PCB_I2SCOUNT
} CC2640R2_PCB_I2SName;

/*!
 *  @def    CC2640R2_PCB_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC2640R2_PCB_NVSName {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    CC2640R2_PCB_NVSCC26XX0 = 0,
#endif
#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH
    CC2640R2_PCB_NVSSPI25X0,
#endif

    CC2640R2_PCB_NVSCOUNT
} CC2640R2_PCB_NVSName;

/*!
 *  @def    CC2640R2_PCB_PWM
 *  @brief  Enum of PWM outputs
 */
typedef enum CC2640R2_PCB_PWMName {
    CC2640R2_PCB_PWM0 = 0,
    CC2640R2_PCB_PWM1,
    CC2640R2_PCB_PWM2,
    CC2640R2_PCB_PWM3,
    CC2640R2_PCB_PWM4,
    CC2640R2_PCB_PWM5,
    CC2640R2_PCB_PWM6,
    CC2640R2_PCB_PWM7,

    CC2640R2_PCB_PWMCOUNT
} CC2640R2_PCB_PWMName;

/*!
 *  @def    CC2640R2_PCB_SDName
 *  @brief  Enum of SD names
 */
typedef enum CC2640R2_PCB_SDName {
    CC2640R2_PCB_SDSPI0 = 0,

    CC2640R2_PCB_SDCOUNT
} CC2640R2_PCB_SDName;

/*!
 *  @def    CC2640R2_PCB_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC2640R2_PCB_SPIName {
    CC2640R2_PCB_SPI0 = 0,
    CC2640R2_PCB_SPI1,

    CC2640R2_PCB_SPICOUNT
} CC2640R2_PCB_SPIName;

/*!
 *  @def    CC2640R2_PCB_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC2640R2_PCB_UARTName {
    CC2640R2_PCB_UART0 = 0,

    CC2640R2_PCB_UARTCOUNT
} CC2640R2_PCB_UARTName;

/*!
 *  @def    CC2640R2_PCB_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2640R2_PCB_UDMAName {
    CC2640R2_PCB_UDMA0 = 0,

    CC2640R2_PCB_UDMACOUNT
} CC2640R2_PCB_UDMAName;

/*!
 *  @def    CC2640R2_PCB_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC2640R2_PCB_WatchdogName {
    CC2640R2_PCB_WATCHDOG0 = 0,

    CC2640R2_PCB_WATCHDOGCOUNT
} CC2640R2_PCB_WatchdogName;

/*!
 *  @def    CC2650_PCB_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2640R2_PCB_TRNGName {
    CC2640R2_PCB_TRNG0 = 0,
    CC2640R2_PCB_TRNGCOUNT
} CC2640R2_PCB_TRNGName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2640R2_PCB_BOARD_H__ */
