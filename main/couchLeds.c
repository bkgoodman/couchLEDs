#include <stdio.h>
#include	<string.h>
#include	<esp_log.h>
#include	"esp_event.h"	//	for usleep
#include <esp_system.h>
#include "freertos/task.h"

//#include "config_rmt.h"
#include "neopixel.h"
#define NEOPIXEL_RMT_CHANNEL 0
#define GPIO_OUTPUT_NEOPIXEL 23
#define NR_LED (4)
#define	NEOPIXEL_WS2812 1
#include "soc/rtc_wdt.h"
volatile uint32_t rc_test_val=0;
volatile unsigned long recv=0;
uint32_t		pixels[NR_LED];
pixel_settings_t px;
static const char *TAG = "CouchLED";

void set_all_pixels(uint16_t r, uint16_t g, uint16_t b) {
	int i;
	for (i=0;i<NR_LED;i++) {
		np_set_pixel_rgbw_level(&px, i , r,g,b,0,255);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
}

void set_leftright_pixels(uint16_t rl, uint16_t gl, uint16_t bl,
      uint16_t rr, uint16_t gr, uint16_t br) {
	int i;
	for (i=0;i<NR_LED;i++) {
    if (i < NR_LED/2)
		np_set_pixel_rgbw_level(&px, i , rl,gl,bl,0,255);
    else
		np_set_pixel_rgbw_level(&px, i , rr,gr,br,0,255);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
}
#define RC_CHANNELS 6
unsigned short rc_gpio[RC_CHANNELS]={4,18,19,34,35,36};
unsigned short rc_gpio_binary_out[RC_CHANNELS]={33,32,27,26,25,22};
unsigned long rc_val[RC_CHANNELS];
// Measure RC data
#define OLD_MIN 430
#define OLD_MAX 730
#define RC_MEDIAN (((OLD_MAX-OLD_MIN)/2)+OLD_MIN)
void rc_recv(void *arg) {
  unsigned long tm[RC_CHANNELS];
  short i;
  unsigned short state[RC_CHANNELS];

  for (i=0;i<RC_CHANNELS;i++) {
    gpio_set_direction(rc_gpio[i],GPIO_MODE_INPUT);
    gpio_set_direction(rc_gpio_binary_out[i],GPIO_MODE_OUTPUT);
    state[i]=0;
    tm[i]=0;
  }
  while(1) {
    for (i=0;i<RC_CHANNELS;i++){
      unsigned short level = gpio_get_level(rc_gpio[i]);
      switch (state[i]) {
        case 0: // Await High
                if (level==1) {
                  state[i]=1;
                  tm[i]=0;
                }
          break;
        case 1: // Await Low
                tm[i]++;
                if (level==0) {
                  state[i]=0;
                  rc_val[i]=tm[i];
                  gpio_set_level(rc_gpio_binary_out[i],(rc_val[i]>= RC_MEDIAN)? 1: 0);
                }
          break;
      }
    }
  }
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

int map_range(int value) {
    // Calculate the old range and the new range
    int old_range = OLD_MAX - OLD_MIN;
    int new_range = 255 - 0;
    
    // Map the value from the old range to the new range
    int mapped_value = (((value - OLD_MIN) * new_range) / old_range) + 0;
    
    if (mapped_value<0)  return 0;
    if (mapped_value>255)  return 255;
    return mapped_value;
}

typedef enum state {
  STATE_INIT=0,
  STATE_IDLE,
  STATE_FWD,
  STATE_REV,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_STOP
} state_e;

state_e state=STATE_INIT;
state_e nextstate=STATE_INIT;

const char *statestr[] = {
  "Init",
  "Idle",
  "Fwd",
  "Rev",
  "Left",
  "Right",
  "Stop"
};
unsigned long state_time=0;

int abs(int a) {
  if (a>0) return a;
  return (-a);
}

void app_main(void)
{

		ESP_LOGI(TAG,"On the Air.");
    init_leds();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    set_all_pixels(16,0,0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    set_all_pixels(0,16,0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(&rc_recv, "RCrcv", 8192, NULL, 55, NULL,1);
    set_all_pixels(0,0,16);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    set_all_pixels(8,8,8);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //rmt_init();
     while (1) {
       uint16_t l;
        rtc_wdt_feed();
        //ESP_LOGI(TAG,"%u %d %d %d",recv,ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2]);
        //ESP_LOGI(TAG,"%u %u",recv,rc_test_val);
        unsigned int v1 = map_range(rc_val[0]);
        unsigned int v2 = map_range(rc_val[1]);
        unsigned int v3 = map_range(rc_val[2]);
        if (state != STATE_INIT) {
          if (v1<118) nextstate=STATE_LEFT;
          else if (v1>138) nextstate=STATE_RIGHT;
          else if (v2>138) nextstate=STATE_FWD;
          else if (v2<118) nextstate=STATE_REV;
          else if (state != STATE_IDLE) nextstate=STATE_STOP;
        }

        if ((state == STATE_STOP) && (state_time >= 16)) {
          nextstate = STATE_IDLE;
        }
        if ((state != nextstate) && (state_time > 5)) {
          state = nextstate;
          state_time = 0;
        }

        l = state_time % 16;
        if (l >= 8) {
          l=16-l;
        }
        l = (l*16)+8;
        ESP_LOGI(TAG,"RC Value= %lu %lu %lu = %u %u %u = level = %d : %s -> %s",rc_val[0],rc_val[1],rc_val[2],v1,v2,v3,l,statestr[nextstate],statestr[state]);
        //set_all_pixels(ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2],255);
        //set_all_pixels(0,0,0);
        //vTaskDelay(250 / portTICK_PERIOD_MS);
        switch (state) {
          case STATE_INIT:
            set_all_pixels(state_time <2 ? 255:0,state_time >=4 ? 255:0,state_time % 2 ? 0 : 255);
            if (state_time >= 10) {
              state = STATE_IDLE;
              nextstate = STATE_IDLE;
              state_time =0;
            }
            break;
          case STATE_LEFT:
            if (state_time % 2)
              set_leftright_pixels(32,32,0,0,(v2>138) ? l:0,0);
            else
              set_leftright_pixels(0,0,0,0,(v2>138) ? l:0,0);
            break;
          case STATE_RIGHT:
            if (state_time % 2)
              set_leftright_pixels(0,(v2>138) ? l:0,0,32,32,0);
            else
              set_leftright_pixels(0,(v2>138) ? l:0,0,0,0,0);
            break;
          case STATE_STOP:
            set_all_pixels(64-(state_time*4),0,0);
            break;
          case STATE_REV:
            set_all_pixels(32,32,32);
            break;
          case STATE_FWD:
            set_all_pixels(0,l,0);
            break;
          case STATE_IDLE:
            set_all_pixels(0,0,0);
            break;
          default:
            set_all_pixels(v1,v2,v3);
            break;
        }
        state_time++;
        vTaskDelay(100 / portTICK_PERIOD_MS);
     }
}

