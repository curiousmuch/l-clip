/*
 * Author: Tyler Berezowsky <tberezowsky@gmail.com>
 * SX1278 LoRa Driver
 * Todo: Add SNR function
 * Todo: Add set frequency function
 * Todo: Add set preamble length function
 * Todo: Add set syncword function
 *
 */

#include "SX1278.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"


//////////////////////////////////
// logic
//////////////////////////////////

// notes: need to replace HAL_GPIO_WritePin()

void SX1278_hw_init(SX1278_hw_t * hw) {
	// setup GPIO pins
	gpio_set_direction(hw->dio0.pin, GPIO_MODE_INPUT);
	gpio_set_direction(hw->nss.pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(hw->reset.pin, GPIO_MODE_OUTPUT);
	SX1278_hw_SetNSS(hw, 1);
	gpio_set_level(hw->reset.pin, 1);

	// SPI protocol configuration
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

	// SPI pins configuration
	spi_bus_config_t SX1278_bus_config = {
	    .miso_io_num = GPIO_NUM_19,
	    .mosi_io_num = GPIO_NUM_27,
	    .sclk_io_num = GPIO_NUM_5,
	    .quadwp_io_num = -1,
	    .quadhd_io_num = -1
	};

    ESP_ERROR_CHECK( spi_bus_initialize(HSPI_HOST, &SX1278_bus_config, 0) );
    ESP_ERROR_CHECK( spi_bus_add_device(HSPI_HOST, &SX1278_device_config, &hw->spi) );
}

void SX1278_hw_SetNSS(SX1278_hw_t * hw, int value) {
	//HAL_GPIO_WritePin(hw->nss.port, hw->nss.pin,
	//		(value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	gpio_set_level(hw->nss.pin, value);
}

void SX1278_hw_Reset(SX1278_hw_t * hw) {
	SX1278_hw_SetNSS(hw, 1);
	gpio_set_level(hw->reset.pin, 0);

	SX1278_hw_DelayMs(1);

	gpio_set_level(hw->reset.pin, 1);

	SX1278_hw_DelayMs(100);
}

void SX1278_hw_SPICommand(SX1278_hw_t * hw, uint8_t cmd) {
	SX1278_hw_SetNSS(hw, 0);

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       // zero out the transaction
	t.cmd = 0;
	t.tx_buffer = &cmd;				// write only command
	t.length = 8*1;

	spi_device_transmit(hw->spi, &t);
}

// reads only one byte?
uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t * hw) {
	//uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	SX1278_hw_SetNSS(hw, 0);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = 0;
    t.rx_buffer = &rxByte;
    t.length = 0;
    t.rxlength = 8*1;
    spi_device_transmit(hw->spi, &t);
    return rxByte;
}

void SX1278_hw_DelayMs(uint32_t msec) {
	vTaskDelay(msec / portTICK_PERIOD_MS);
}

int SX1278_hw_GetDIO0(SX1278_hw_t * hw) {
	return gpio_get_level(hw->dio0.pin);
	//return (HAL_GPIO_ReadPin(hw->dio0.port, hw->dio0.pin) == GPIO_PIN_SET);
}

//////////////////////////////////
// logic
//////////////////////////////////

uint8_t SX1278_SPIRead(SX1278_t * module, uint8_t addr) {
	uint8_t tmp;
	SX1278_hw_SPICommand(module->hw, addr);
	tmp = SX1278_hw_SPIReadByte(module->hw);
	SX1278_hw_SetNSS(module->hw, 1);
	return tmp;
}

void SX1278_SPIWrite(SX1278_t * module, uint8_t addr, uint8_t cmd) {
	SX1278_hw_SetNSS(module->hw, 0);
	SX1278_hw_SPICommand(module->hw, addr | 0x80);
	SX1278_hw_SPICommand(module->hw, cmd);
	SX1278_hw_SetNSS(module->hw, 1);
}

void SX1278_SPIBurstRead(SX1278_t * module, uint8_t addr, uint8_t* rxBuf,
		uint8_t length) {
	uint8_t i;
	if (length <= 1) {
		return;
	} else {
		SX1278_hw_SetNSS(module->hw, 0);
		SX1278_hw_SPICommand(module->hw, addr);
		for (i = 0; i < length; i++) {
			*(rxBuf + i) = SX1278_hw_SPIReadByte(module->hw);
		}
		SX1278_hw_SetNSS(module->hw, 1);
	}
}

void SX1278_SPIBurstWrite(SX1278_t * module, uint8_t addr, uint8_t* txBuf,
		uint8_t length) {
	unsigned char i;
	if (length <= 1) {
		return;
	} else {
		SX1278_hw_SetNSS(module->hw, 0);
		SX1278_hw_SPICommand(module->hw, addr | 0x80);
		for (i = 0; i < length; i++) {
			SX1278_hw_SPICommand(module->hw, *(txBuf + i));
		}
		SX1278_hw_SetNSS(module->hw, 1);
	}
}

void SX1278_defaultConfig(SX1278_t * module) {
	SX1278_config(module, module->frequency, module->power, module->LoRa_Rate,
			module->LoRa_BW);
}

void SX1278_config(SX1278_t * module, uint8_t frequency, uint8_t power,
		uint8_t LoRa_Rate, uint8_t LoRa_BW) {
	SX1278_sleep(module); //Change modem mode Must in Sleep mode
	SX1278_hw_DelayMs(15);

	SX1278_entryLoRa(module);
	//SX1278_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

	SX1278_SPIBurstWrite(module, LR_RegFrMsb,
			(uint8_t*) SX1278_Frequency[frequency], 3); //setting  frequency parameter

	//setting base parameter
	SX1278_SPIWrite(module, LR_RegPaConfig, SX1278_Power[power]); //Setting output power parameter

	SX1278_SPIWrite(module, LR_RegOcp, 0x0B);			//RegOcp,Close Ocp
	SX1278_SPIWrite(module, LR_RegLna, 0x23);			//RegLNA,High & LNA Enable
	if (SX1278_SpreadFactor[LoRa_Rate] == 6) {	//SFactor=6
		uint8_t tmp;
		SX1278_SPIWrite(module,
		LR_RegModemConfig1,
				((SX1278_LoRaBandwidth[LoRa_BW] << 4) + (SX1278_CR << 1) + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		LR_RegModemConfig2,
				((SX1278_SpreadFactor[LoRa_Rate] << 4) + (SX1278_CRC << 2)
						+ 0x03));

		tmp = SX1278_SPIRead(module, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX1278_SPIWrite(module, 0x31, tmp);
		SX1278_SPIWrite(module, 0x37, 0x0C);
	} else {
		SX1278_SPIWrite(module,
		LR_RegModemConfig1,
				((SX1278_LoRaBandwidth[LoRa_BW] << 4) + (SX1278_CR << 1) + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		LR_RegModemConfig2,
				((SX1278_SpreadFactor[LoRa_Rate] << 4) + (SX1278_CRC << 2)
						+ 0x03)); //SFactor &  LNA gain set by the internal AGC loop
	}

	SX1278_SPIWrite(module, LR_RegSymbTimeoutLsb, 0xFF); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1278_SPIWrite(module, LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX1278_SPIWrite(module, LR_RegPreambleLsb, 12); //RegPreambleLsb 8+4=12byte Preamble
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	module->readBytes = 0;
	SX1278_standby(module); //Entry standby mode
}

void SX1278_standby(SX1278_t * module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x09);
	module->status = STANDBY;
}

void SX1278_sleep(SX1278_t * module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x08);
	module->status = SLEEP;
}

void SX1278_entryLoRa(SX1278_t * module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x88);
}

void SX1278_clearLoRaIrq(SX1278_t * module) {
	SX1278_SPIWrite(module, LR_RegIrqFlags, 0xFF);
}

int SX1278_LoRaEntryRx(SX1278_t * module, uint8_t length, uint32_t timeout) {
	uint8_t addr;

	module->packetLength = length;

	SX1278_defaultConfig(module);		//Setting base parameter
	SX1278_SPIWrite(module, REG_LR_PADAC, 0x84);	//Normal and RX
	SX1278_SPIWrite(module, LR_RegHopPeriod, 0xFF);	//No FHSS
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01);//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F);//Open RxDone interrupt & Timeout
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, LR_RegPayloadLength, length);//Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
	addr = SX1278_SPIRead(module, LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8d);	//Mode//Low Frequency Mode
	//SX1278_SPIWrite(module, LR_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	module->readBytes = 0;

	while (1) {
		if ((SX1278_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04) {	//Rx-on going RegModemStat
			module->status = RX;
			return 1;
		}
		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_defaultConfig(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

uint8_t SX1278_LoRaRxPacket(SX1278_t * module) {
	unsigned char addr;
	unsigned char packet_size;

	if (SX1278_hw_GetDIO0(module->hw)) {
		memset(module->rxBuffer, 0x00, SX1278_MAX_PACKET);

		addr = SX1278_SPIRead(module, LR_RegFifoRxCurrentaddr); //last packet addr
		SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (module->LoRa_Rate == SX1278_LORA_SF_6) { //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
			packet_size = module->packetLength;
		} else {
			packet_size = SX1278_SPIRead(module, LR_RegRxNbBytes); //Number for received bytes
		}

		SX1278_SPIBurstRead(module, 0x00, module->rxBuffer, packet_size);
		module->readBytes = packet_size;
		SX1278_clearLoRaIrq(module);
	}
	return module->readBytes;
}

int SX1278_LoRaEntryTx(SX1278_t * module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;

	SX1278_defaultConfig(module); //setting base parameter
	SX1278_SPIWrite(module, REG_LR_PADAC, 0x87);	//Tx for 20dBm
	SX1278_SPIWrite(module, LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(module, LR_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(module, LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	while (1) {
		temp = SX1278_SPIRead(module, LR_RegPayloadLength);
		if (temp == length) {
			module->status = TX;
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_defaultConfig(module);
			return 0;
		}
	}
}

int SX1278_LoRaTxPacket(SX1278_t * module, uint8_t* txBuffer, uint8_t length,
		uint32_t timeout) {
	SX1278_SPIBurstWrite(module, 0x00, txBuffer, length);
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8b);	//Tx Mode
	while (1) {
		if (SX1278_hw_GetDIO0(module->hw)) { //if(Get_NIRQ()) //Packet send over
			SX1278_SPIRead(module, LR_RegIrqFlags);
			SX1278_clearLoRaIrq(module); //Clear irq
			SX1278_standby(module); //Entry Standby mode
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_defaultConfig(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

void SX1278_begin(SX1278_t * module, uint8_t frequency, uint8_t power,
		uint8_t LoRa_Rate, uint8_t LoRa_BW, uint8_t packetLength) {
	SX1278_hw_init(module->hw);
	module->frequency = frequency;
	module->power = power;
	module->LoRa_Rate = LoRa_Rate;
	module->LoRa_BW = LoRa_BW;
	module->packetLength = packetLength;
	SX1278_defaultConfig(module);
}

int SX1278_transmit(SX1278_t * module, uint8_t* txBuf, uint8_t length,
		uint32_t timeout) {
	if (SX1278_LoRaEntryTx(module, length, timeout)) {
		return SX1278_LoRaTxPacket(module, txBuf, length, timeout);
	}
	return 0;
}

int SX1278_receive(SX1278_t * module, uint8_t length, uint32_t timeout) {
	return SX1278_LoRaEntryRx(module, length, timeout);
}

uint8_t SX1278_available(SX1278_t * module) {
	return SX1278_LoRaRxPacket(module);
}

uint8_t SX1278_read(SX1278_t * module, uint8_t* rxBuf, uint8_t length) {
	if (length != module->readBytes)
		length = module->readBytes;
	memcpy(rxBuf, module->rxBuffer, length);
	rxBuf[length] = '\0';
	module->readBytes = 0;
	return length;
}

uint8_t SX1278_RSSI_LoRa(SX1278_t * module) {
	uint32_t temp = 10;
	temp = SX1278_SPIRead(module, LR_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t SX1278_RSSI(SX1278_t * module) {
	uint8_t temp = 0xff;
	temp = SX1278_SPIRead(module, 0x11);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}
