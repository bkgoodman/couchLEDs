#include <stdio.h>
#include	<string.h>
#include	<esp_log.h>
#include	"esp_event.h"	//	for usleep
#include <esp_system.h>
#include "freertos/task.h"
#include "driver/uart.h"

//#include "config_rmt.h"
#include "neopixel.h"
#define NEOPIXEL_RMT_CHANNEL 0
#define NEOPIXEL_RMT_CHANNEL2 1
#define GPIO_OUTPUT_NEOPIXEL 23
#define GPIO_OUTPUT_NEOPIXEL2 22
#define NR_LED (18)
#define	NEOPIXEL_WS2812 1
#include "soc/rtc_wdt.h"

// Audio Board
// https://github.com/SnijderC/dyplayer 
// https://m.media-amazon.com/images/I/A1rcOBoeTaL.pdf

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
	vTaskDelay(10 / portTICK_PERIOD_MS);
	np_show(&px, NEOPIXEL_RMT_CHANNEL2);

}

void set_leftright_pixels(uint16_t rl, uint16_t gl, uint16_t bl,
      uint16_t rr, uint16_t gr, uint16_t br) {
	int i;
	vTaskDelay(10 / portTICK_PERIOD_MS);
	for (i=0;i<NR_LED;i++) {
		np_set_pixel_rgbw_level(&px, i , rl,gl,bl,0,255);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	for (i=0;i<NR_LED;i++) {
		np_set_pixel_rgbw_level(&px, i , rr,gr,br,0,255);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL2);
}


void idle_pixels(unsigned long state_time) {
	int i;
  int s;
  //unsigned long h;
  //float hue;
	for (i=0;i<NR_LED;i++) {
    //h = (state_time + i)%360;
    //hue = (float) h;
    //np_set_pixel_color_hsb(&px, i, hue, 1, (float) 0.5);
    //
    uint32_t c;
    s = (state_time + i) % 24;
    c = (0xff << s) | (0xff >> (24-s));
    np_set_pixel_color(&px, i, c);
	}
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	np_show(&px, NEOPIXEL_RMT_CHANNEL2);
}

#define RC_CHANNELS 6
const uart_port_t uart_num = UART_NUM_2;
unsigned short rc_gpio[RC_CHANNELS]={4,18,19,34,35,36};
unsigned short rc_gpio_binary_out[RC_CHANNELS]={33,32,27,26,25,21};
//unsigned short sound_gpio[RC_CHANNELS]={17,16,13}; // 16 and 17 used for UART2
unsigned long rc_val[RC_CHANNELS];
// Measure RC data
#define OLD_MIN 430
#define OLD_MAX 730
#define RC_MEDIAN (((OLD_MAX-OLD_MIN)/2)+OLD_MIN)
#define RC_BOTTOM_THIRD (((OLD_MAX-OLD_MIN)/3)+OLD_MIN)
#define RC_TOP_THIRD ((((OLD_MAX-OLD_MIN)/3)*2)+OLD_MIN)
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
  ESP_LOGE(TAG,"rc_val ptr is at %p size is %d",rc_val,sizeof(rc_val));
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
	rc = neopixel_init(GPIO_OUTPUT_NEOPIXEL2, NEOPIXEL_RMT_CHANNEL2);
	if (rc < 0)
		ESP_LOGE(TAG, "neopixel_init2 rc = %d", rc);

	px.pixels = (uint8_t *)pixels;
	px.pixel_count = NR_LED;
#ifdef	NEOPIXEL_WS2812
	strcpy(px.color_order, "RGB");
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
	np_show(&px, NEOPIXEL_RMT_CHANNEL2);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	np_set_pixel_rgbw_level(&px, 0 , 0,0,0,0,255);
	np_show(&px, NEOPIXEL_RMT_CHANNEL);
	np_show(&px, NEOPIXEL_RMT_CHANNEL2);
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

void bkg_uart_init() {
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      //.rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, -1, -1));
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, 256, 
                                          256, 0, (QueueHandle_t *) 0L, 0));
}

void bkg_uart_xmit(void *data, size_t len) {
  // Write data to UART.
  uart_write_bytes(uart_num, (const char*)data, len);
  // Wait for packet to be sent
  ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 250)); // wait timeout is 100 RTOS ticks (TickType_t)
}
uint8_t checksum(uint8_t *data, uint8_t len)
  {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
      sum = sum + data[i];
    }
    return sum;
  }

void stop_sound() {
  uint8_t vol[4] = {0xaa,0x10,0x00,0xba};
  bkg_uart_xmit(vol,4);
}
void play_sound(uint16_t sound) {
  uint8_t zero[5] = {0,0,0,0,0};
  uint8_t vol[5] = {0xaa,0x13,0x01,0xff,0};
  uint8_t buf[6] = {0xaa,0x07,0x02,0x00,0x00,0x00};
  //bkg_uart_xmit(zero,5);
  vol[4] = checksum(vol,4);
  bkg_uart_xmit(vol,5);

  buf[4] = sound;
  buf[5] = checksum(buf,5);
  bkg_uart_xmit(buf,6);
}

void app_main(void)
{

    bkg_uart_init();
    unsigned short mode = 0;
    unsigned long hornTime = 0;

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
    /*
    int i;
    for (i=1;i<10;i++) {
        ESP_LOGI(TAG,"Sound %d",i);
      play_sound(i);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    */
    play_sound(6);
    //rmt_init();
     while (1) {
       uint16_t l;
        rtc_wdt_feed();
        //ESP_LOGI(TAG,"%u %d %d %d",recv,ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2]);
        //ESP_LOGI(TAG,"%u %u",recv,rc_test_val);
        unsigned int v1 = map_range(rc_val[0]);
        unsigned int v2 = map_range(rc_val[1]);
        unsigned int v3 = map_range(rc_val[2]);
        unsigned int v4 = map_range(rc_val[3]);

        short newmode = 1;
        if (rc_val[2] > RC_TOP_THIRD) {
            newmode =2;
        }
        if (rc_val[2] < RC_BOTTOM_THIRD) newmode =0;

        if ((newmode == 1) && (mode != 1)) {
          play_sound(1);
        }
        else if ((mode == 1) && (newmode != 1)) {
          stop_sound();
        }

        if ((v3 > 200) && (hornTime >3)) {
          play_sound(5);
        }
        hornTime++;

        mode = newmode;

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
          /* Exit Stae */
          switch (state) {
            case STATE_REV:
              stop_sound();
              break;
            default:
              break;
          }
          state = nextstate;
          state_time = 0;
          // Enter State
          switch (state) {
            case STATE_REV:
              play_sound(3);
              break;
            default:
              break;
          }
        }

        l = state_time % 16;
        if (l >= 8) {
          l=16-l;
        }
        l = (l*16)+8;
        ESP_LOGI(TAG,"RC Value= %lu %lu %lu = %u %u %u = mode = %d : %s -> %s",
            rc_val[0],rc_val[1],rc_val[2],v1,v2,v3,mode,statestr[nextstate],statestr[state]);
        //set_all_pixels(ReceiverChannels[0],ReceiverChannels[1],ReceiverChannels[2],255);
        //set_all_pixels(0,0,0);
        //vTaskDelay(250 / portTICK_PERIOD_MS);
        //
        if (mode == 2) {
           set_all_pixels(state_time % 2 ? 255:0,0,0);
        }
        else if (mode == 1) {
                switch (state_time % 40) {
                  case 0:
                  case 20:
                    set_leftright_pixels(255,255,255,   0,0,0); 
                    break;
                  case 2:
                  case 4:
                  case 6:
                  case 8:
                    set_leftright_pixels(255,0,0,   0,0,0); 
                    break;
                  case 22:
                  case 24:
                  case 26:
                  case 28:
                    set_leftright_pixels(0,0,255,   0,0,0); 
                    break;
                  case 10:
                  case 30:
                    set_leftright_pixels(0,0,0,   255,255,255); 
                    break;
                  case 12:
                  case 14:
                  case 16:
                  case 18:
                    set_leftright_pixels(0,0,0,   0,0,255); 
                      break;
                  case 32:
                  case 34:
                  case 36:
                  case 38:
                    set_leftright_pixels(0,0,0,   255,0,0); 
                    break;
                  default:
                    set_leftright_pixels(0,0,0,   0,0,0); 
                    break;
              }
        } else 
          // Normal mode
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
              set_leftright_pixels(255,255,0,0,(v2>138) ? l:0,0);
            else
              set_leftright_pixels(0,0,0,0,(v2>138) ? l:0,0);
            break;
          case STATE_RIGHT:
            if (state_time % 2)
              set_leftright_pixels(0,(v2>138) ? l:0,0,255,255,0);
            else
              set_leftright_pixels(0,(v2>138) ? l:0,0,0,0,0);
            break;
          case STATE_STOP:
            set_all_pixels(256-(state_time*16),0,0);
            break;
          case STATE_REV:
            set_all_pixels(128,128,128);
            break;
          case STATE_FWD:
            set_all_pixels(0,l,0);
            break;
          case STATE_IDLE:
            if (state_time >= 100) {
              idle_pixels(state_time-100);
            } else 
              set_all_pixels(0,0,0);
            break;
          default:
            set_all_pixels(v1,v2,v3);
            break;
        }
        state_time++;
        vTaskDelay(((mode == 1)? 50: 100) / portTICK_PERIOD_MS);
     }
}

