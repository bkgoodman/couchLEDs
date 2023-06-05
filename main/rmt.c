// this file configure the RMT for 2 purposes:
// - Reading the pulse widths of the servo signal from the RC receiver
// - Reading the pulse widths of the hall effect sensors in the motor
//
#include "config_rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "config.h"
#include "esp_err.h"
#include "esp_check.h"

static rmt_item32_t led_data_buffer[LED_BUFFER_ITEMS];
#define RMT_TICK_PER_US 8
// determines how many clock cycles one "tick" is
// [1..255], source is generally 80MHz APB clk
#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)
// time before receiver goes idle
#define RMT_RX_MAX_US 3500


void rmt_init(void);
static const char* TAG = "RMT";
volatile unsigned xmit_intr_count=0;

volatile uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM] = {0};
const uint8_t XMIT_CHANNEL = 0;
const uint8_t RECEIVER_CHANNELS[RECEIVER_CHANNELS_NUM] = { 1, 2, 3 };
const uint8_t RECEIVER_GPIOS[RECEIVER_CHANNELS_NUM] = { 18, 17, 5 };

static void rmt_isr_handler(void* arg){
    // with reference to https://www.esp32.com/viewtopic.php?t=7116#p32383
    // but modified so that this ISR only checks chX_rx_end
    uint32_t intr_st = RMT.int_st.val;

    // see declaration of RMT.int_st:
    // takes the form of 
    // bit 0: ch0_tx_end
    // bit 1: ch0_rx_end
    // bit 2: ch0_err
    // bit 3: ch1_tx_end
    // bit 4: ch1_rx_end
    // ...
    // thus, check whether bit (channel*3 + 1) is set to identify
    // whether that channel has changed

    uint8_t i;
    for(i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
        uint8_t channel = RECEIVER_CHANNELS[i];
        uint32_t channel_mask = BIT(channel*3+1);

        if (!(intr_st & channel_mask)) continue;

        RMT.conf_ch[channel].conf1.rx_en = 0;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
        volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;
        if (item) {
            ReceiverChannels[i] = item->duration0;
        }

        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[channel].conf1.rx_en = 1;

        //clear RMT interrupt status.
        RMT.int_clr.val = channel_mask;
    }

    // https://github.com/espressif/esp-idf/blob/v3.3/components/soc/esp32/include/soc/rmt_struct.h
    uint32_t channel_mask = BIT(XMIT_CHANNEL*3);
    if (intr_st & BIT(channel_mask)) {
      xmit_intr_count++;
      RMT.int_clr.val = channel_mask;
    }
}

static void setup_rmt_data_buffer(struct led_state new_state)
{
  for (uint32_t led = 0; led < NUM_LEDS; led++) {
    uint32_t bits_to_send = new_state.leds[led];
    uint32_t mask = 1 << (BITS_PER_LED_CMD - 1);

    for (uint32_t bit = 0; bit < BITS_PER_LED_CMD; bit++) {
      uint32_t bit_is_set = bits_to_send & mask;
      led_data_buffer[(led * BITS_PER_LED_CMD) + bit] = bit_is_set ?
                                                      (rmt_item32_t){{{T1H, 1, T1L, 0}}} :
                                                      (rmt_item32_t){{{T0H, 1, T0L, 0}}};
      mask >>= 1;
    }
  }
}
esp_err_t rmt_write_leds(struct led_state new_state)
{
  setup_rmt_data_buffer(new_state);
  ESP_RETURN_ON_ERROR(rmt_write_items(LED_RMT_TX_CHANNEL, led_data_buffer, LED_BUFFER_ITEMS, false), TAG, "Failed to write items");
  ESP_RETURN_ON_ERROR(rmt_wait_tx_done(LED_RMT_TX_CHANNEL, portMAX_DELAY), TAG, "Failed to wait for RMT transmission to finish");

  return ESP_OK;
}

void rmt_init(void) {
    uint8_t i;

    rmt_config_t rmt_channels[RECEIVER_CHANNELS_NUM] = {};

    /*
    for (i = 0; i < RECEIVER_CHANNELS_NUM; i++) {
        ReceiverChannels[i] = RECEIVER_CH_CENTER;

        rmt_channels[i].channel = (rmt_channel_t) RECEIVER_CHANNELS[i];
        rmt_channels[i].gpio_num = (gpio_num_t) RECEIVER_GPIOS[i];
        rmt_channels[i].clk_div = RMT_RX_CLK_DIV;
        rmt_channels[i].mem_block_num = 1;
        rmt_channels[i].rmt_mode = RMT_MODE_RX;
        rmt_channels[i].rx_config.filter_en = true;
        rmt_channels[i].rx_config.filter_ticks_thresh = 100;
        rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;

        ESP_ERROR_CHECK(rmt_config(&rmt_channels[i]));
    ESP_LOGI(TAG, "Allocated RX RMT channel %d",rmt_channels[i].channel);
  ESP_LOGI(TAG,"dd");
        //rmt_set_rx_intr_en(rmt_channels[i].channel, true);
        //rmt_rx_start(rmt_channels[i].channel, 1);
    }
    */

    // Setup TX for neopixels
  rmt_config_t tx_config = {
    .rmt_mode = RMT_MODE_TX,
    .channel = XMIT_CHANNEL,
    .gpio_num = LED_RMT_TX_GPIO,
    .mem_block_num = 3,
    .tx_config.loop_en = false,
    .tx_config.carrier_en = false,
    .tx_config.idle_output_en = true,
    .tx_config.idle_level = 0,
    .clk_div = 2
  };

  ESP_RETURN_ON_ERROR(rmt_driver_install(0, 0, 0), TAG, "Failed to install RMT driver");
  ESP_LOGI(TAG,"000");
  ESP_ERROR_CHECK(rmt_config(&tx_config));
  ESP_LOGI(TAG,"aaa");
  ESP_ERROR_CHECK(rmt_set_tx_intr_en(XMIT_CHANNEL,true));
  ESP_LOGI(TAG,"bb");
  ESP_ERROR_CHECK(rmt_tx_start(XMIT_CHANNEL, true));
  ESP_LOGI(TAG,"cc");

  rmt_isr_register(rmt_isr_handler, NULL, 0, NULL);
  ESP_LOGI(TAG, "Init ISR on %d", xPortGetCoreID());
}
