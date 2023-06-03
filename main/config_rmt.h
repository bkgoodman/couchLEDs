#include "freertos/FreeRTOS.h"

#define RECEIVER_CHANNELS_NUM 3
#define RECEIVER_CH_CENTER 128

extern volatile uint16_t ReceiverChannels[RECEIVER_CHANNELS_NUM];
void rmt_init(void);
