/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "board-config.h"
#include "sx1276-board.h"
#include "board.h"

///*!
// * Initializes the unused GPIO to a know status
// */
//static void BoardUnusedIoInit( void );

spi_device_handle_t radio_spi;

void BoardInitPeriph( void )
{
	// enable GPIO service
	gpio_install_isr_service(0);

	// enable SPI service for radio
	spi_device_interface_config_t SX1278_device_config = {
	    .clock_speed_hz = 1*1000*1000,
	    .mode = 0,
	    .spics_io_num = -1,	// driver requires manual control :(
	    .queue_size = 8,
	    .command_bits = 0,
	    .address_bits = 0,
	    .dummy_bits = 0,
	    .duty_cycle_pos = 0,
	    .flags = SPI_DEVICE_HALFDUPLEX
	    //.flags = SPI_DEVICE_HALFDUPLEX
	};

	spi_bus_config_t SX1278_bus_config = {
	    .miso_io_num = RADIO_MISO,
	    .mosi_io_num = RADIO_MOSI,
	    .sclk_io_num = RADIO_SCLK,
	    .quadwp_io_num = -1,
	    .quadhd_io_num = -1
	};

    ESP_ERROR_CHECK( spi_bus_initialize(VSPI_HOST, &SX1278_bus_config, 0) );
    ESP_ERROR_CHECK( spi_bus_add_device(VSPI_HOST, &SX1278_device_config, &radio_spi) );
    SX1276IoInit();
}

void BoardInitMcu( void )
{

}

void BoardResetMcu( void )
{

}

void BoardDeInitMcu( void )
{

}

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3300 // mV

/*!
 * VREF calibration value
 */
//#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF80078 )

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                               4095

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV

//static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;
//
////uint16_t BoardBatteryMeasureVolage( void )
//{
//    uint16_t vdd = 0;
//    uint16_t vref = VREFINT_CAL;
//    uint16_t vdiv = 0;
//    uint16_t batteryVoltage = 0;
//
//    vdiv = AdcReadChannel( &Adc, BAT_LEVEL_CHANNEL );
//    //vref = AdcReadChannel( &Adc, ADC_CHANNEL_VREFINT );
//
//    vdd = ( float )FACTORY_POWER_SUPPLY * ( float )VREFINT_CAL / ( float )vref;
//    batteryVoltage = vdd * ( ( float )vdiv / ( float )ADC_MAX_VALUE );
//
//    //                                vDiv
//    // Divider bridge  VBAT <-> 470k -<--|-->- 470k <-> GND => vBat = 2 * vDiv
//    batteryVoltage = 2 * batteryVoltage;
//    return batteryVoltage;
//}
//
//uint32_t BoardGetBatteryVoltage( void )
//{
//    return BatteryVoltage;
//}
//
//uint8_t BoardGetBatteryLevel( void )
//{
//    uint8_t batteryLevel = 0;
//
//    BatteryVoltage = BoardBatteryMeasureVolage( );
//
//    if( GetBoardPowerSource( ) == USB_POWER )
//    {
//        batteryLevel = 0;
//    }
//    else
//    {
//        if( BatteryVoltage >= BATTERY_MAX_LEVEL )
//        {
//            batteryLevel = 254;
//        }
//        else if( ( BatteryVoltage > BATTERY_MIN_LEVEL ) && ( BatteryVoltage < BATTERY_MAX_LEVEL ) )
//        {
//            batteryLevel = ( ( 253 * ( BatteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
//        }
//        else if( ( BatteryVoltage > BATTERY_SHUTDOWN_LEVEL ) && ( BatteryVoltage <= BATTERY_MIN_LEVEL ) )
//        {
//            batteryLevel = 1;
//        }
//        else //if( BatteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
//        {
//            batteryLevel = 255;
//            //GpioInit( &DcDcEnable, DC_DC_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//            //GpioInit( &BoardPowerDown, BOARD_POWER_DOWN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
//        }
//    }
//    return batteryLevel;
//}
//
//static void BoardUnusedIoInit( void )
//{
//
//}
//
//uint8_t GetBoardPowerSource( void )
//{
//
//}

