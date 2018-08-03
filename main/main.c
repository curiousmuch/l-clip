
#include "board.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RF_FREQUENCY_MIN                            440000000 // Hz
#define TX_OUTPUT_POWER                             14        // 14 dBm
#define RF_FREQUENCY_MAX							140050000

#define FSK_FDEV                                    0         // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   true

uint32_t freq;

void app_main(void)
{
	BoardInitMcu();
	BoardInitPeriph();

}
