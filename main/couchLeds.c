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

extern volatile uint16_t rc_val;
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

#if 0
void app_main(void)
{

		ESP_LOGI(TAG,"On the Air.");
    //init_leds();
    rmt_init();
     while (1) {
        ESP_LOGI(TAG,"%u %d %d %d",rc_val,ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2]);
        //set_pixels(ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2],255);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        //set_pixels(16,0,0,255);
        vTaskDelay(250 / portTICK_PERIOD_MS);
     }
}
#endif

#if 1
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"
#include "esp_intr_alloc.h"
#include <soc/dport_reg.h>

#define RMT_CHANNEL     RMT_CHANNEL_2
#define RMT_GPIO_NUM    GPIO_NUM_18
#define RMT_CLK_DIV     80
#define RMT_RX_FILTER_THRES    100

volatile uint32_t test=0;
void IRAM_ATTR rmt_isr(void* arg) {
  uint32_t intr_st = RMT.int_st.val;
        uint32_t channel_mask = BIT(RMT_CHANNEL_2*3+1);

RMT.int_clr.val = intr_st;
        short channel = RMT_CHANNEL_2;
        if (!(intr_st & channel_mask)) return;

        RMT.conf_ch[channel].conf1.rx_en = 0;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
        volatile rmt_item32_t* item = RMTMEM.chan[RMT_CHANNEL_2].data32;
        if (item) {
            test = item->duration0;
        }

        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[channel].conf1.rx_en = 1;

        //clear RMT interrupt status.

}

rmt_isr_handle_t xHandler = NULL;
intr_handle_t rmt_intr_handle;
void app_main() {

    rmt_config_t config;
    config.rmt_mode = RMT_MODE_RX;
    config.channel = (rmt_channel_t) RMT_CHANNEL_2;
    config.gpio_num = (gpio_num_t)18;
    config.mem_block_num = 1;                 //how many memory blocks 64 x N (0-7)
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 100;
    config.rx_config.idle_threshold = 9500;
    config.clk_div = 80; //1MHz
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_set_rx_intr_en(RMT_CHANNEL, true));
    ESP_ERROR_CHECK(rmt_rx_start(RMT_CHANNEL, 1));
    ESP_ERROR_CHECK(rmt_isr_register(rmt_isr, NULL, ESP_INTR_FLAG_LEVEL1, &rmt_intr_handle));
    while (1) {
        ESP_LOGI(TAG,"%u",test);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
#endif
