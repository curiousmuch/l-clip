/* DAC Cosine Generator Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"


#include "driver/dac.h"

/* Declare global sine waveform parameters
 * so they may be then accessed and changed from debugger
 * over an JTAG interface
 */
int clk_8m_div = 7;      // RTC 8M clock divider (division is by clk_8m_div+1, i.e. 0 means 8MHz frequency)
int frequency_step = 136;  // Frequency step for CW generator
int scale = 1;           // 50% of the full scale
int offset;              // leave it default / 0 = no any offset
int invert = 2;          // invert MSB to get sine waveform


/*
 * Enable cosine waveform generator on a DAC channel
 */
void dac_cosine_enable(dac_channel_t channel)
{
    // Enable tone generator common to both channels
    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
    switch(channel) {
        case DAC_CHANNEL_1:
            // Enable / connect tone tone generator on / to this channel
            SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
            // Invert MSB, otherwise part of waveform will have inverted
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 2, SENS_DAC_INV1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
            break;
        default :
           printf("Channel %d\n", channel);
    }
}
/* Set frequency of internal CW generator common to both DAC channels
 *
 * clk_8m_div: 0b000 - 0b111
 * frequency_step: range 0x0001 - 0xFFFF
 */

void IRAM_ATTR dac_frequency_set(int clk_8m_div, int frequency_step)
{
    REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, clk_8m_div);
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, frequency_step, SENS_SW_FSTEP_S);
}
/*
 * Scale output of a DAC channel using two bit pattern:
 *
 * - 00: no scale
 * - 01: scale to 1/2
 * - 10: scale to 1/4
 * - 11: scale to 1/8
 *
 */
void dac_scale_set(dac_channel_t channel, int scale)
{
    switch(channel) {
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1, scale, SENS_DAC_SCALE1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2, scale, SENS_DAC_SCALE2_S);
            break;
        default :
           printf("Channel %d\n", channel);
    }
}
/*
 * Offset output of a DAC channel
 *
 * Range 0x00 - 0xFF
 *
 */
void dac_offset_set(dac_channel_t channel, int offset)
{
    switch(channel) {
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC1, offset, SENS_DAC_DC1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC2, offset, SENS_DAC_DC2_S);
            break;
        default :
           printf("Channel %d\n", channel);
    }
}
/*
 * Invert output pattern of a DAC channel
 *
 * - 00: does not invert any bits,
 * - 01: inverts all bits,
 * - 10: inverts MSB,
 * - 11: inverts all bits except for MSB
 *
 */
void dac_invert_set(dac_channel_t channel, int invert)
{
    switch(channel) {
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, invert, SENS_DAC_INV1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, invert, SENS_DAC_INV2_S);
            break;
        default :
           printf("Channel %d\n", channel);
    }
}
/*
 * AFSK MODEM Code Development!!!!!!!!
 */

typedef struct {
	dac_channel_t channel;
	timer_group_t timer_group;
	timer_idx_t timer_idx;
	uint32_t freq1;
	uint32_t freq2;
	uint32_t rate;
	uint32_t freq1_val;
	uint32_t freq2_val;
} afsk_config_t;

typedef struct {
	uint32_t buffer[125];
	uint32_t len;
	uint32_t index;
} afsk_buffer_t;

afsk_buffer_t buf = {{0xFFFF0000}, 32, 0};
afsk_config_t modem_config;


// TODO: Check if this works
void IRAM_ATTR afsk_modem_timer_isr(void *param)
{
	timer_idx_t timer_idx = modem_config.timer_idx;

	// get interrupt status and get timer value
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value =
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    // clear interrupt
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        TIMERG0.int_clr_timers.t0 = 1;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        //evt.type = -1; // not supported even type
    }

	// set frequency for AFSK
    if (buf.len)
    {
    	if (buf.buffer[buf.index] & BIT1)
    		frequency_step = 136;
    	else
    		frequency_step = 74;
    }


    dac_frequency_set(clk_8m_div, frequency_step);
    buf.len =- 1;
    if (!buf.len)
    {
    	buf.len = 32;
    	buf.buffer[0] = 0xFFFF0000;
    	buf.index = 0;
    }

    buf.buffer[buf.index] = buf.buffer[buf.index]>>1;
    //if (!(buf.len % 32))
    //	buf.index =- 1;


    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}


void afsk_modem_init(afsk_config_t *config)
{
	// store settings in public struc
	modem_config.channel = config->channel; // does this work?
	modem_config.timer_group = config->timer_group;
	modem_config.timer_idx = config->timer_idx;
	modem_config.freq1 = config->freq1;
	modem_config.freq2 = config->freq2;

	// waveform generator
	dac_cosine_enable(modem_config.channel);
	dac_frequency_set(clk_8m_div, frequency_step);
	dac_scale_set(modem_config.channel, scale);
	dac_offset_set(modem_config.channel, offset);
	dac_invert_set(modem_config.channel, invert);

	// dac
	//dac_output_disable(modem_config.channel);
	dac_output_enable(modem_config.channel);				// debugging

	// timer
	// TODO: Configure to be adjustable based on rate
    timer_config_t timer_config;
    timer_config.divider = 2;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.auto_reload = TIMER_AUTORELOAD_EN;
    timer_config.intr_type = TIMER_INTR_LEVEL;
    timer_config.alarm_en = TIMER_ALARM_EN;
    timer_config.counter_en = TIMER_PAUSE;

    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 33333);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, afsk_modem_timer_isr, NULL,
    		ESP_INTR_FLAG_IRAM, NULL);

	timer_start(modem_config.timer_group, modem_config.timer_idx);
}


void afsk_modem_task(void *param)
{
	while(1)
	{
		// check queue

	}

}

/*
 * Generate a sine waveform on both DAC channels:
 *
 * DAC_CHANNEL_1 - GPIO25
 * DAC_CHANNEL_2 - GPIO26
 *
 * Connect scope to both GPIO25 and GPIO26
 * to observe the waveform changes
 * in response to the parameter change
*/

#define APRS_DATA_RATE 1200
//
//void app_main()
//{
////	// DAC init
//    dac_cosine_enable(DAC_CHANNEL_1);
//    dac_cosine_enable(DAC_CHANNEL_2);
//
//    dac_output_enable(DAC_CHANNEL_1);
//    dac_output_enable(DAC_CHANNEL_2);
//
////    // timer init
////    timer_config_t timer_config;
////    timer_config.divider = 2;
////    timer_config.counter_dir = TIMER_COUNT_UP;
////    timer_config.auto_reload = TIMER_AUTORELOAD_EN;
////    timer_config.intr_type = TIMER_INTR_LEVEL;
////    timer_config.alarm_en = TIMER_ALARM_EN;
////    timer_config.counter_en = TIMER_PAUSE;
//
////    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
////    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
////    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 33333);
////    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
////    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, NULL,
////    		ESP_INTR_FLAG_IRAM, NULL);
//
////    // waveform generator init
////    dac_frequency_set(clk_8m_div, frequency_step);
////    dac_scale_set(DAC_CHANNEL_2, scale);
////    dac_offset_set(DAC_CHANNEL_2, offset);
////    dac_invert_set(DAC_CHANNEL_2, invert);
//
//
////    timer_start(TIMER_GROUP_0, TIMER_0);
//
//	// setup configuration struct for AFSK modem
//	afsk_config_t afsk_config = {
//			DAC_CHANNEL_1,			// DAC channel
//			TIMER_GROUP_0,			// timer group
//			TIMER_0, 				// timer idx
//			100,					// frequency 1
//			50,						// frequency 2
//			1200,					// data rate
//			0,
//			0
//	};
//
//	// initial example
//	afsk_modem_init(&afsk_config);
//
//    while(1)
//    {
//    	printf("fuck\n");
//    	vTaskDelay(2000/portTICK_PERIOD_MS);
//    }
//
//    //xTaskCreate(dactask, "dactask", 1024*3, NULL, 10, NULL);
//}
