/*
 * @Author: your name
 * @Date: 2021-10-13 17:24:29
 * @LastEditTime: 2021-10-29 17:52:44
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \s3_lvgl_v7\components\adc\bsp_adc.h
 */

#ifndef _KSDIY_LVGL_PORT_
#define _KSDIY_LVGL_PORT_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#ifdef __cplusplus
extern "C"
{
#endif
    extern esp_lcd_panel_handle_t panel_handle;

    void ksdiy_lvgl_lcd_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif