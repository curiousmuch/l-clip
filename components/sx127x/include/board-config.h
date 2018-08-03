/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "driver/gpio.h"
// TODO: Modify for ESP32 Board / Future Schematic

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0


/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 GPIO_NUM_22

#define RADIO_MOSI                                  GPIO_NUM_23
#define RADIO_MISO                                  GPIO_NUM_19
#define RADIO_SCLK                                  GPIO_NUM_18
#define RADIO_NSS                                   GPIO_NUM_5

#define RADIO_DIO_0                                 GPIO_NUM_36
#define RADIO_DIO_1                                 GPIO_NUM_39
#define RADIO_DIO_2                                 GPIO_NUM_34
#define RADIO_DIO_3                                 GPIO_NUM_35
#define RADIO_DIO_4                                 GPIO_NUM_27
#define RADIO_DIO_5                                 GPIO_NUM_17

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#endif // __BOARD_CONFIG_H__
