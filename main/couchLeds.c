#include <stdio.h>
#include	<string.h>
#include	<esp_log.h>
#include	"esp_event.h"	//	for usleep
#include <esp_system.h>
#include "freertos/task.h"

#include "config_rmt.h"
#include "neopixel.h"
#define NEOPIXEL_RMT_CHANNEL 0
#define GPIO_OUTPUT_NEOPIXEL 23
#define NR_LED (1)
#define	NEOPIXEL_WS2812 1
uint32_t		pixels[NR_LED];
pixel_settings_t px;
static const char *TAG = "CouchLED";

extern volatile unsigned xmit_intr_count;
void app_main(void)
{

		//ESP_LOGI(TAG,"Init Neopixels");
    //ESP_ERROR_CHECK(ws2812_control_init());
		ESP_LOGI(TAG,"Init RC");
    rmt_init();
		ESP_LOGI(TAG,"On the Air.");
    struct led_state new_state;
    new_state.leds[0] = 0x000080;

    rmt_write_leds(new_state);

     while (1) {
        ESP_LOGI(TAG,"%d %d %d IC=%u",ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2],xmit_intr_count);
        //set_pixels(ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2],255);
        //set_pixels(16,16,16,255);
        vTaskDelay(250 / portTICK_PERIOD_MS);
     }
}
