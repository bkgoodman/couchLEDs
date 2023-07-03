#include "freertos/FreeRTOS.h"

#define CONFIG_WS2812_LED_TYPE_RGB 1

#if CONFIG_WS2812_LED_TYPE_RGB
#define BITS_PER_LED_CMD	24
#elif CONFIG_WS2812_LED_TYPE_RGBW
#define BITS_PER_LED_CMD	32
#endif

#define LED_BUFFER_ITEMS	(NUM_LEDS * BITS_PER_LED_CMD)

// These values are determined by measuring pulse timing with logic analyzer and adjusting to match datasheet. 
#define T0H	14 // 0 bit high time
#define T1H	52 // 1 bit high time
#define T0L	52 // low time for either bit
#define T1L	52

#define RECEIVER_CHANNELS_NUM 1
#define RECEIVER_CH_CENTER 128

extern volatile uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM];
void rmt_init(void);
#define NUM_LEDS	1
// Configure these based on your project needs using menuconfig ********
#define LED_RMT_TX_CHANNEL		0
#define LED_RMT_TX_GPIO			23
// ****************************************************

// This structure is used for indicating what the colors of each LED should be set to.
// There is a 32bit value for each LED. Only the lower 3 bytes are used and they hold the
// Red (byte 2), Green (byte 1), and Blue (byte 0) values to be set.
struct led_state {
    uint32_t leds[NUM_LEDS];
};


// Update the LEDs to the new state. Call as needed.
// This function will block the current task until the RMT peripheral is finished sending 
// the entire sequence.
esp_err_t rmt_write_leds(struct led_state new_state);

