
#ifndef APP_SPEECH_H
#define APP_SPEECH_H
#include "esp_log.h"
#include "esp_system.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>
#ifdef __cplusplus
extern "C"
{
#endif
extern esp_codec_dev_handle_t output_dev_;

extern i2s_chan_handle_t tx_handle_ ;
extern i2s_chan_handle_t rx_handle_ ;
void Codec_I2S_init(void);

#ifdef __cplusplus
}
#endif
#endif