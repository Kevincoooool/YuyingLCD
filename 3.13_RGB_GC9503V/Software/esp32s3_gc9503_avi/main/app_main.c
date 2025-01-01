/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lcd_gc9503.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "driver/spi_master.h"
#include "lv_demos.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_jpeg_common.h"
#include "esp_jpeg_dec.h"
#include "esp_jpeg_enc.h"
#include "avi_player.h"
#include "app_speech.h"
#include "ksdiy_lcd_port.h"
static const char *TAG = "example";
uint8_t *img_rgb565 = NULL; // MALLOC_CAP_SPIRAM
uint8_t *pbuffer = NULL;    // MALLOC_CAP_SPIRAM  MALLOC_CAP_DMA
size_t rgb_width = 0;
size_t rgb_height = 0;

static bool end_play = false;
uint32_t Rgbsize = 0;

static jpeg_pixel_format_t j_type = JPEG_PIXEL_FORMAT_RGB565_LE;
static jpeg_rotate_t j_rotation = JPEG_ROTATE_0D;

jpeg_error_t esp_jpeg_decode_one_picture(uint8_t *input_buf, int len, uint8_t **output_buf, int *out_len)
{
    uint8_t *out_buf = NULL;
    jpeg_error_t ret = JPEG_ERR_OK;
    jpeg_dec_io_t *jpeg_io = NULL;
    jpeg_dec_header_info_t *out_info = NULL;
    rgb_width = 0;
    rgb_height = 0;
    // Generate default configuration
    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = j_type;
    config.rotate = j_rotation;
    // config.scale.width       = 0;
    // config.scale.height      = 0;
    // config.clipper.width     = 0;
    // config.clipper.height    = 0;

    // Create jpeg_dec handle
    jpeg_dec_handle_t jpeg_dec = NULL;
    ret = jpeg_dec_open(&config, &jpeg_dec);
    if (ret != JPEG_ERR_OK)
    {
        return ret;
    }

    // Create io_callback handle
    jpeg_io = calloc(1, sizeof(jpeg_dec_io_t));
    if (jpeg_io == NULL)
    {
        ret = JPEG_ERR_NO_MEM;
        goto jpeg_dec_failed;
    }

    // Create out_info handle
    out_info = calloc(1, sizeof(jpeg_dec_header_info_t));
    if (out_info == NULL)
    {
        ret = JPEG_ERR_NO_MEM;
        goto jpeg_dec_failed;
    }

    // Set input buffer and buffer len to io_callback
    jpeg_io->inbuf = input_buf;
    jpeg_io->inbuf_len = len;

    // Parse jpeg picture header and get picture for user and decoder
    ret = jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info);
    if (ret != JPEG_ERR_OK)
    {
        goto jpeg_dec_failed;
    }
    rgb_width = out_info->width;
    rgb_height = out_info->height;
    ESP_LOGI(TAG, "img width:%d height:%d ", rgb_width, rgb_height);

    *out_len = out_info->width * out_info->height * 3;
    // Calloc out_put data buffer and update inbuf ptr and inbuf_len
    if (config.output_type == JPEG_PIXEL_FORMAT_RGB565_LE || config.output_type == JPEG_PIXEL_FORMAT_RGB565_BE || config.output_type == JPEG_PIXEL_FORMAT_CbYCrY)
    {
        *out_len = out_info->width * out_info->height * 2;
    }
    else if (config.output_type == JPEG_PIXEL_FORMAT_RGB888)
    {
        *out_len = out_info->width * out_info->height * 3;
    }
    else
    {
        ret = JPEG_ERR_INVALID_PARAM;
        goto jpeg_dec_failed;
    }
    out_buf = jpeg_calloc_align(*out_len, 16);
    if (out_buf == NULL)
    {
        ret = JPEG_ERR_NO_MEM;
        goto jpeg_dec_failed;
    }
    jpeg_io->outbuf = out_buf;
    *output_buf = out_buf;

    // Start decode jpeg
    ret = jpeg_dec_process(jpeg_dec, jpeg_io);
    if (ret != JPEG_ERR_OK)
    {
        goto jpeg_dec_failed;
    }

    // Decoder deinitialize
jpeg_dec_failed:
    jpeg_dec_close(jpeg_dec);
    if (jpeg_io)
    {
        free(jpeg_io);
    }
    if (out_info)
    {
        free(out_info);
    }
    return ret;
}

void video_write(frame_data_t *data, void *arg)
{
    int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    int min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    int free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Free internal: %u minimal internal: %u free_psram: %u", free_sram, min_free_sram, free_psram);
    // ESP_LOGI(TAG, "Video write: %d", data->data_bytes);
    /* Set src of image with file name */
    esp_jpeg_decode_one_picture(data->data, data->data_bytes, &img_rgb565, &Rgbsize); // 使用乐鑫adf的jpg解码 速度快三倍
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, rgb_width , rgb_height , img_rgb565);
    free(img_rgb565);
}

void audio_write(frame_data_t *data, void *arg)
{
    size_t bytes_write = 0;

    // ESP_LOGI(TAG, "Audio write: %d", data->data_bytes);
    // i2s_channel_write(tx_handle_, data->data, data->data_bytes, &bytes_write, 100);
    esp_codec_dev_write(output_dev_, data->data, data->data_bytes);

}

void audio_set_clock(uint32_t rate, uint32_t bits_cfg, uint32_t ch, void *arg)
{
    ESP_LOGI(TAG, "Audio set clock, rate %" PRIu32 ", bits %" PRIu32 ", ch %" PRIu32 "", rate, bits_cfg, ch);
}

void avi_play_end(void *arg)
{
    ESP_LOGI(TAG, "Play end");
    end_play = true;
    // fclose("/spiffs/output.avi");
    avi_player_play_from_file("/spiffs/output.avi");
}

/*显示spiffs的所有文件名*/
static void SPIFFS_Directory(char *path)
{
    DIR *dir = opendir(path);
    assert(dir != NULL);
    while (true)
    {
        struct dirent *pe = readdir(dir);
        if (!pe)
            break;
        ESP_LOGI(__FUNCTION__, "d_name=%s d_ino=%d d_type=%x", pe->d_name, pe->d_ino, pe->d_type);
    }
    closedir(dir);
}
//视频文件用以下命令转换：
//.\ffmpeg.exe -i .\badapple.avi -t 30 -vcodec mjpeg -vf scale=280:240 -r 14 -q:v 60 -acodec pcm_s16le -ar 16000 output.avi
/*
-t是时间
-r是帧率
-q:v 是jpg质量 数值越大输出的文件越小  数值越小 越清晰
*/
void app_main(void)
{
    ESP_LOGI(TAG, "Compile time: %s %s", __DATE__, __TIME__);

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    /*初始化spiffs用于存放字体文件或者图片文件或者网页文件*/
    ESP_LOGI(TAG, "Initializing SPIFFS");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 2,
        .format_if_mount_failed = true};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        else
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
    /*显示spiffs里的文件列表*/
    SPIFFS_Directory("/spiffs/");
    Codec_I2S_init();
    img_rgb565 = heap_caps_malloc(1024 * 600 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM); // MALLOC_CAP_SPIRAM

    ksdiy_lvgl_lcd_init();
    end_play = false;

    avi_player_config_t config = {
        .buffer_size = 50 * 1024,
        .audio_cb = audio_write,
        .video_cb = video_write,
        // .audio_set_clock_cb = audio_set_clock,
        .avi_play_end_cb = avi_play_end,
        .coreID = 1,

    };

    avi_player_init(config);
    avi_player_play_from_file("/spiffs/output.avi");

}
