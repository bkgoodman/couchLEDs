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

void set_pixels(uint16_t r, uint16_t g, uint16_t b, uint16_t l) {
	int i;
	for (i=0;i<NR_LED;i++) {
		np_set_pixel_rgbw_level(&px, i , r,g,b,0,l);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
}

void init_leds() {
	int rc;
	rc = neopixel_init(GPIO_OUTPUT_NEOPIXEL, NEOPIXEL_RMT_CHANNEL);
	if (rc < 0)
		ESP_LOGE(TAG, "neopixel_init rc = %d", rc);

	px.pixels = (uint8_t *)pixels;
	px.pixel_count = NR_LED;
#ifdef	NEOPIXEL_WS2812
	strcpy(px.color_order, "GRB");
#else
	strcpy(px.color_order, "GRBW");
#endif

	memset(&px.timings, 0, sizeof(px.timings));
	px.timings.mark.level0 = 1;
	px.timings.space.level0 = 1;
	px.timings.mark.duration0 = 12;
#ifdef	NEOPIXEL_WS2812
	px.nbits = 24;
	px.timings.mark.duration1 = 14;
	px.timings.space.duration0 = 7;
	px.timings.space.duration1 = 16;
	px.timings.reset.duration0 = 600;
	px.timings.reset.duration1 = 600;
#endif
#ifdef	NEOPIXEL_SK6812
	px.nbits = 32;
	px.timings.mark.duration1 = 12;
	px.timings.space.duration0 = 6;
	px.timings.space.duration1 = 18;
	px.timings.reset.duration0 = 900;
	px.timings.reset.duration1 = 900;
#endif
	px.brightness = 0x80;


	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	np_set_pixel_rgbw_level(&px, 0 , 16,0,0,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	np_set_pixel_rgbw_level(&px, 0 , 0,16,0,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	np_set_pixel_rgbw_level(&px, 0 , 0,0,16,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	np_set_pixel_rgbw_level(&px, 0 , 0,0,0,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
}
void app_main(void)
{

		ESP_LOGI(TAG,"On the Air.");
    init_leds();
    rmt_init();
     while (1) {
        ESP_LOGI(TAG,"%d %d %d",ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2]);
        set_pixels(ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2],255);
        vTaskDelay(250 / portTICK_PERIOD_MS);
     }
}
