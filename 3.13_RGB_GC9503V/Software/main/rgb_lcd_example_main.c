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
static const char *TAG = "example";
lv_obj_t *img_cam; // 要显示图像
lv_img_dsc_t img_dsc = {
    .header.always_zero = 0,
    .header.w = 376,
    .header.h = 960,
    .data_size = 376 * 960 * 2,
    .header.cf = LV_IMG_CF_TRUE_COLOR,
    .data = NULL,
};
#define JPG_IMAGE_MAX_SIZE (450 * 1024)
#include "lvgl.h"

void lv_example_color_gradient(void)
{
    lv_obj_t *scr = lv_scr_act(); // 获取当前屏幕对象

    // 创建一个颜色数组，用于显示色阶块
    lv_color_t colors[] = {
        LV_COLOR_MAKE(0xFF, 0xFF, 0xFF), // 黑色
        LV_COLOR_MAKE(0xFF, 0x00, 0x00), // 红色
        LV_COLOR_MAKE(0x00, 0xFF, 0x00), // 绿色
        LV_COLOR_MAKE(0x00, 0x00, 0xFF), // 蓝色
        LV_COLOR_MAKE(0x00, 0x00, 0x00), // 白色
        // LV_COLOR_GRAY,   // 灰色
        // LV_COLOR_YELLOW, // 黄色
        // LV_COLOR_CYAN,   // 青色
        // LV_COLOR_MAGENTA // 品红
    };

    int color_count = sizeof(colors) / sizeof(colors[0]); // 计算颜色数量

    // 创建水平布局容器，按顺序排列色阶块
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, LV_HOR_RES, LV_VER_RES); // 宽度与高度为屏幕分辨率
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);  // 使用水平布局

    // 遍历颜色数组，生成色阶块
    for (int i = 0; i < color_count; i++)
    {
        lv_obj_t *color_block = lv_obj_create(cont);              // 创建一个色块
        lv_obj_set_size(color_block, LV_HOR_RES / 8, LV_VER_RES); // 平均宽度
        lv_obj_set_style_bg_color(color_block, colors[i], 0);     // 设置背景颜色
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT 4
#define EXAMPLE_PIN_NUM_HSYNC 6
#define EXAMPLE_PIN_NUM_VSYNC 5
#define EXAMPLE_PIN_NUM_DE 15
#define EXAMPLE_PIN_NUM_PCLK 7

// #define EXAMPLE_PIN_NUM_DATA0 47 // B0
// #define EXAMPLE_PIN_NUM_DATA1 21 // B1
// #define EXAMPLE_PIN_NUM_DATA2 14 // B2
// #define EXAMPLE_PIN_NUM_DATA3 13 // B3
// #define EXAMPLE_PIN_NUM_DATA4 12 // B4

// #define EXAMPLE_PIN_NUM_DATA5 11  // G0
// #define EXAMPLE_PIN_NUM_DATA6 10  // G1
// #define EXAMPLE_PIN_NUM_DATA7 9   // G2
// #define EXAMPLE_PIN_NUM_DATA8 46  // G3
// #define EXAMPLE_PIN_NUM_DATA9 3   // G4
// #define EXAMPLE_PIN_NUM_DATA10 20 // G5

// #define EXAMPLE_PIN_NUM_DATA11 19 // R0
// #define EXAMPLE_PIN_NUM_DATA12 8  // R1
// #define EXAMPLE_PIN_NUM_DATA13 18 // R2
// #define EXAMPLE_PIN_NUM_DATA14 17 // R3
// #define EXAMPLE_PIN_NUM_DATA15 16 // R4

#define EXAMPLE_PIN_NUM_DATA0 47 // B0
#define EXAMPLE_PIN_NUM_DATA1 21 // B1
#define EXAMPLE_PIN_NUM_DATA2 14 // B2
#define EXAMPLE_PIN_NUM_DATA3 13 // B3
#define EXAMPLE_PIN_NUM_DATA4 12 // B4

#define EXAMPLE_PIN_NUM_DATA5 11  // G0
#define EXAMPLE_PIN_NUM_DATA6 10  // G1
#define EXAMPLE_PIN_NUM_DATA7 9   // G2
#define EXAMPLE_PIN_NUM_DATA8 46  // G3
#define EXAMPLE_PIN_NUM_DATA9 3   // G4
#define EXAMPLE_PIN_NUM_DATA10 20 // G5

#define EXAMPLE_PIN_NUM_DATA11 19 // R0
#define EXAMPLE_PIN_NUM_DATA12 8  // R1
#define EXAMPLE_PIN_NUM_DATA13 18 // R2
#define EXAMPLE_PIN_NUM_DATA14 17 // R3
#define EXAMPLE_PIN_NUM_DATA15 16 // R4

#define EXAMPLE_PIN_NUM_DISP_EN -1

#define TEST_LCD_IO_SPI_CS_1 (GPIO_NUM_48)
#define TEST_LCD_IO_SPI_SCL_1 (GPIO_NUM_17)
#define TEST_LCD_IO_SPI_SDO_1 (GPIO_NUM_16)

// #define TEST_LCD_IO_SPI_SCL_1 (GPIO_NUM_17)
// #define TEST_LCD_IO_SPI_SDO_1 (GPIO_NUM_16)

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 376
#define EXAMPLE_LCD_V_RES 960

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB 2
#else
#define EXAMPLE_LCD_NUM_FB 1
#endif // CONFIG_EXAMPLE_DOUBLE_FB

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

static SemaphoreHandle_t lvgl_mux = NULL;

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE)
    {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver

    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

char file_name_with_path[128] = {0};

static void btn_event_cb(lv_event_t *event)
{
    lv_obj_t *img = (lv_obj_t *)event->user_data;
    const char *file_name = lv_list_get_btn_text(lv_obj_get_parent(event->target), event->target);
    ESP_LOGI(TAG, "btn_event_cb");
    memset(file_name_with_path, 0, sizeof(file_name_with_path));
    /* Get full file name with mount point and folder path */
    strcpy(file_name_with_path, "S:/spiffs/");
    strcat(file_name_with_path, file_name);

    /* Set src of image with file name */
    lv_img_set_src(img, file_name_with_path);

    /* Align object */
    lv_obj_align(img, LV_ALIGN_CENTER, 40, 0);

    /* Only for debug */
    ESP_LOGI(TAG, "Display image file : %s", file_name_with_path);
}

static void image_display(void)
{
    lv_obj_t *list = lv_list_create(lv_scr_act());
    lv_obj_set_size(list, 120, 240);
    lv_obj_set_style_border_width(list, 0, LV_STATE_DEFAULT);
    lv_obj_align(list, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *img = lv_img_create(lv_scr_act());

    /* Get file name in storage */
    struct dirent *p_dirent = NULL;
    DIR *p_dir_stream = opendir("/spiffs");

    /* Scan files in storage */
    while (true)
    {
        p_dirent = readdir(p_dir_stream);
        if (NULL != p_dirent)
        {
            if (p_dirent->d_type == DT_REG)
            {

                if (strstr(p_dirent->d_name, ".jpg") || strstr(p_dirent->d_name, ".JPG"))
                {

                    lv_obj_t *btn = lv_list_add_btn(list, NULL, p_dirent->d_name);
                    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, (void *)img);
                }
            }
        }
        else
        {
            closedir(p_dir_stream);
            break;
        }
    }
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
uint8_t *img_rgb565 = NULL; // MALLOC_CAP_SPIRAM
uint8_t *pbuffer = NULL;    // MALLOC_CAP_SPIRAM  MALLOC_CAP_DMA
size_t rgb_width = 0;
size_t rgb_height = 0;

static jpeg_pixel_format_t j_type = JPEG_PIXEL_FORMAT_RGB565_BE;
static jpeg_rotate_t j_rotation = JPEG_ROTATE_0D;

jpeg_error_t esp_jpeg_decode_one_picture(uint8_t *input_buf, int len, uint8_t **output_buf, int *out_len)
{
    uint8_t *out_buf = NULL;
    jpeg_error_t ret = JPEG_ERR_OK;
    jpeg_dec_io_t *jpeg_io = NULL;
    jpeg_dec_header_info_t *out_info = NULL;

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
void app_main(void)
{
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
    /* malloc a buffer for RGB565 data */

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif
#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = TEST_LCD_IO_SPI_CS_1,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = TEST_LCD_IO_SPI_SCL_1,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = TEST_LCD_IO_SPI_SDO_1,
        .io_expander = NULL,
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    (esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t rgb_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .bits_per_pixel = 16,

        .dma_burst_size = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .clk_src = LCD_CLK_SRC_PLL240M,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = GC9503_480_480_PANEL_60HZ_RGB_TIMING(),
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };

    ESP_LOGI(TAG, "Initialize RGB LCD panel");

    gc9503_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .flags = {
            .mirror_by_cmd = 0,
            .auto_del_panel_io = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    (esp_lcd_new_panel_gc9503(io_handle, &panel_config, &panel_handle));
    (esp_lcd_panel_reset(panel_handle));
    (esp_lcd_panel_init(panel_handle));
    // esp_lcd_panel_swap_xy(panel_handle, true);
    // (esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    assert(buf1);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 40);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);

#if CONFIG_BSP_DISPLAY_LVGL_ROTATION_90 || CONFIG_BSP_DISPLAY_LVGL_ROTATION_270
    disp_drv.hor_res = EXAMPLE_LCD_V_RES;
    disp_drv.ver_res = EXAMPLE_LCD_H_RES;
#else
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
#endif

    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Scatter Chart");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1))
    {
        // example_lvgl_demo_ui(disp);
        // lv_demo_music();
        // image_display();
        // lv_demo_widgets();
        // lv_demo_benchmark();
        // lv_main_page();
        // Release the mutex
        img_cam = lv_img_create(lv_scr_act());
        lv_obj_align(img_cam, LV_ALIGN_CENTER, 0, 0);
        // lv_example_color_gradient();
        example_lvgl_unlock();
    }

    /* malloc a buffer for RGB565 data */
    uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc((376 * 960 * 2), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(lcd_buffer != NULL);

    uint8_t *jpeg_buf = (uint8_t *)heap_caps_malloc(JPG_IMAGE_MAX_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(jpeg_buf != NULL);

    size_t i = 10;
    char file_name[64] = {0};
    uint32_t Rgbsize = 0;

    while (1)
    {

        if (i > 14)
            i = 10;
        sprintf(file_name, "/spiffs/img%03d.jpg", i);
        ESP_LOGI(TAG, "file_name: %s,  free_heap: %d", file_name, esp_get_free_heap_size());

        FILE *fd = fopen(file_name, "r");
        memset(jpeg_buf, 0, JPG_IMAGE_MAX_SIZE);
        int read_bytes = fread(jpeg_buf, 1, JPG_IMAGE_MAX_SIZE, fd);
        fclose(fd);
        esp_jpeg_decode_one_picture(jpeg_buf, read_bytes, &lcd_buffer, &Rgbsize); // 使用乐鑫adf的jpg解码 速度快三倍

        img_dsc.data = (uint8_t *)lcd_buffer;
        lv_img_set_src(img_cam, &img_dsc);
        // mjpegdraw(jpeg_buf, read_bytes, lcd_buffer, lcd_write_bitmap);
        ESP_LOGD(TAG, "file_name: %s, fd: %p, read_bytes: %d, free_heap: %d", file_name, fd, read_bytes, esp_get_free_heap_size());
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        i += 1;
    }

    free(jpeg_buf);
}
