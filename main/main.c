#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "SX1278.h"
#include "fap/fap.h"

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int ret;
char buffer[64];
int message = 0;
int message_length;

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "access_point_name",
            .password = "password",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );


//   // LoRa Module Configuration
//	SX1278_hw.dio0.pin = GPIO_NUM_26;
//	SX1278_hw.nss.pin = GPIO_NUM_18;
//	SX1278_hw.reset.pin = GPIO_NUM_14;
//	SX1278.hw = &SX1278_hw;
//
//	printf("Configuring LoRa Radio\r\n");
//	SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
//			SX1278_LORA_BW_20_8KHZ, 10);
//	printf("Completed configuration for LoRa Radio \r\n");
//	//ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
//	ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);



    while (true) {
//    	printf("Sending Packet...\n\r");
//
//    	message_length = sprintf(buffer, "Hello %d", message);
//		ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
//		printf("Entry: %d\r\n", ret);
//
//
//		printf("Sending %s\r\n", buffer);
//		ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t *) buffer, message_length,
//				2000);
//		message += 1;
//
//		printf("Transmission: %d\r\n", ret);
//		printf("Package sent...\r\n");

		printf("Receiving package...\r\n");

//		ret = SX1278_LoRaRxPacket(&SX1278);
//		printf("Received: %d\r\n", ret);
//		if (ret > 0) {
//			SX1278_read(&SX1278, (uint8_t *) buffer, ret);
//			printf("Content (%d): %s\r\n", ret, buffer);
//		}
//		printf("Package received ...\r\n");

		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

