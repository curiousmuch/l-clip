/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
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
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "sx1276-board.h"

#include <stdlib.h>
#include "board-config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radio.h"

//TODO: Needs to be updated to reflect ESP32 board / future schematic

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;



/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime
};

void SX1276IoInit( void )
{
	// TODO: create ESP-IDF HAL integration layer
	// setup SX1267 IO struct


	// setup radio DIO pins
	gpio_config_t sx1276_io;
	sx1276_io.pin_bit_mask = (uint32_t)(BIT(RADIO_DIO_0) | BIT(RADIO_DIO_1) | BIT(RADIO_DIO_3) | BIT(RADIO_DIO_4) |
			BIT(RADIO_DIO_5));
	sx1276_io.mode = GPIO_MODE_INPUT;
	sx1276_io.pull_up_en = GPIO_PULLUP_ENABLE;
	sx1276_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
	sx1276_io.intr_type = GPIO_PIN_INTR_DISABLE;
	gpio_config(&sx1276_io);

	// setup reset and slave select pin
	sx1276_io.pin_bit_mask = BIT(RADIO_RESET) | BIT(RADIO_NSS);
	sx1276_io.mode = GPIO_MODE_OUTPUT;
	sx1276_io.pull_up_en = GPIO_PULLUP_DISABLE;
	sx1276_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
	sx1276_io.intr_type = GPIO_PIN_INTR_DISABLE;
	gpio_config(&sx1276_io);
	gpio_set_level(RADIO_RESET, 1);
	gpio_set_level(RADIO_NSS, 1);
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
	// attach interrupts
	gpio_isr_handler_add(RADIO_DIO_0, (gpio_isr_t)irqHandlers[0], NULL);
	gpio_isr_handler_add(RADIO_DIO_1, (gpio_isr_t)irqHandlers[1], NULL);
	gpio_isr_handler_add(RADIO_DIO_2, (gpio_isr_t)irqHandlers[2], NULL);
	gpio_isr_handler_add(RADIO_DIO_3, (gpio_isr_t)irqHandlers[3], NULL);
	gpio_isr_handler_add(RADIO_DIO_4, (gpio_isr_t)irqHandlers[4], NULL);
	gpio_isr_handler_add(RADIO_DIO_5, (gpio_isr_t)irqHandlers[5], NULL);

	// enable interrupts
	gpio_config_t sx1276_io;
	sx1276_io.pin_bit_mask = (BIT(RADIO_DIO_0) | BIT(RADIO_DIO_1) | BIT(RADIO_DIO_3) | BIT(RADIO_DIO_4) |
			BIT(RADIO_DIO_5));
	sx1276_io.mode = GPIO_MODE_INPUT;
	sx1276_io.pull_up_en = GPIO_PULLDOWN_ENABLE;
	sx1276_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
	sx1276_io.intr_type = GPIO_INTR_POSEDGE;
	gpio_config(&sx1276_io);
}

void SX1276IoDeInit( void )
{
	// disable pin interrupts
	gpio_config_t sx1276_io;
	sx1276_io.pin_bit_mask = (BIT(RADIO_DIO_0) | BIT(RADIO_DIO_1) | BIT(RADIO_DIO_3) | BIT(RADIO_DIO_4) |
			BIT(RADIO_DIO_5));
	sx1276_io.mode = GPIO_MODE_INPUT;
	sx1276_io.pull_up_en = GPIO_PULLDOWN_DISABLE;
	sx1276_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
	sx1276_io.intr_type = GPIO_PIN_INTR_DISABLE;
	gpio_config(&sx1276_io);

	// attach interrupts
	gpio_isr_handler_remove(RADIO_DIO_0);
	gpio_isr_handler_remove(RADIO_DIO_1);
	gpio_isr_handler_remove(RADIO_DIO_2);
	gpio_isr_handler_remove(RADIO_DIO_3);
	gpio_isr_handler_remove(RADIO_DIO_4);
	gpio_isr_handler_remove(RADIO_DIO_5);
}

/*!
 * \brief Enables/disables the TCXO if available on board design.
 *
 * \param [IN] state TCXO enabled when true and disabled when false.
 */
static void SX1276SetBoardTcxo( uint8_t state )
{
    // No TCXO component available on this board design.
#if 0
    if( state == true )
    {
        TCXO_ON( );
        DelayMs( BOARD_TCXO_WAKEUP_TIME );
    }
    else
    {
        TCXO_OFF( );
    }
#endif
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1276Reset( void )
{
    // Enables the TCXO if available on the board design
    SX1276SetBoardTcxo( true );

    // Set RESET pin to 0
    gpio_set_level(RADIO_RESET, 0);

    // Wait 1 ms
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Configure RESET as input
    gpio_set_level(RADIO_RESET, 1);


    // Wait 6 ms
    vTaskDelay(1/portTICK_PERIOD_MS);
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}


// Disable Antenna Stuff
void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1276SetBoardTcxo( true );
            SX1276AntSwInit( );
        }
        else
        {
            SX1276SetBoardTcxo( false );
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    //GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    //GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
}

void SX1276AntSwDeInit( void )
{
    //GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    //GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
}

void SX1276SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        //GpioWrite( &AntSwitchLf, 0 );
        //GpioWrite( &AntSwitchHf, 1 );
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        //GpioWrite( &AntSwitchLf, 1 );
        //GpioWrite( &AntSwitchHf, 0 );
        break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
