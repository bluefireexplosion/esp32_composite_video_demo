#include "sdkconfig.h"
#include <stdio.h>
#include "esp_log.h"
#include <esp_err.h>
#include "nvs_flash.h"
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <driver/uart.h>
#include "esp_system.h"
#include "driver/rtc_io.h"
#include <driver/dac.h>
#include "esp_efuse.h"
#include "esp32/rom/lldesc.h"
#include <soc/rtc.h>
#include "video.h"
#include <esp_system.h>
#include "driver/i2s.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#if CONFIG_VIDEO_ENABLE_LVGL_SUPPORT
#include "lvgl_driver_video.h"
#endif
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/dac.h"
#include "mp3dec.h"
#include <string.h>
#define I2S_NUM I2S_NUM_0
#define AUDIO_DMA_BUF_LEN 1024   // size of each DMA buffer

extern const uint8_t _binary_portal_mp3_start[];  // start address
extern const uint8_t _binary_portal_mp3_end[];    // end address
#define AUDIO_SAMPLE_RATE 16000   // approx, your MP3 frame rate
#define AUDIO_TASK_STACK 8192
#define VIDEO_TASK_STACK 8192
#define AUDIO_BATCH 64

#define BLINK_GPIO 2   

static const char* TAG = "VIDEO";


#if CONFIG_VIDEO_ENABLE_LVGL_SUPPORT
void demo_pm5544(lv_obj_t* scr)
{
    char mode[14];
    video_get_mode_description(mode, sizeof(mode));

    LV_IMG_DECLARE(pm5544);
    lv_obj_t* img1 = lv_img_create(scr);
    lv_img_set_src(img1, &pm5544);
    lv_obj_set_align(img1, LV_ALIGN_CENTER);
    lv_obj_set_size(img1, pm5544.header.w, pm5544.header.h);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t* top_label = lv_label_create(scr);
    lv_obj_set_width(top_label, 55);
    lv_obj_set_style_text_color(top_label, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(top_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(top_label, LV_ALIGN_CENTER, 0, -83);

    lv_obj_t* mode_text = lv_label_create(scr);
    lv_obj_set_width(mode_text, 75);
    lv_obj_set_style_text_color(mode_text, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_align(mode_text, LV_ALIGN_CENTER, 0, 66);
    lv_obj_set_style_text_align(mode_text, LV_TEXT_ALIGN_CENTER, 0);

    char* t = strtok(mode, " ");
    lv_label_set_text(top_label, t);
    t = strtok(NULL, " ");
    lv_label_set_text(mode_text, t);
}

void demo_rhino(lv_obj_t* scr)
{
    LV_IMG_DECLARE(rhino);
    lv_obj_t* img1 = lv_img_create(scr);
    lv_img_set_src(img1, &rhino);
    lv_obj_align(img1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(img1, rhino.header.w, rhino.header.h);
}

static lv_obj_t* meter;
static void meter_set_value(void* indic, int32_t v)
{
    lv_meter_set_indicator_value(meter, indic, v);
}

void demo_meter(lv_obj_t* scr)
{
    meter = lv_meter_create(scr);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 200, 200);

    /* Add a scale first */
    lv_meter_scale_t* scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 8, 4, 15, lv_color_black(), 10);

    lv_meter_indicator_t* indic;

    /* Add a blue arc to the start */
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /* Make the tick lines blue at the start of the scale */
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /* Add a red arc to the end */
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /* Make the tick lines red at the end of the scale */
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /* Add a needle line indicator */
    indic = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);

    /* Create an animation to set the value */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, meter_set_value);
    lv_anim_set_var(&a, indic);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 2000);
    lv_anim_set_repeat_delay(&a, 100);
    lv_anim_set_playback_time(&a, 500);
    lv_anim_set_playback_delay(&a, 100);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

void create_label(lv_style_t* style, lv_obj_t* cont, const char* text, uint8_t row, uint8_t col)
{
    lv_obj_t* obj = lv_label_create(cont);
    lv_obj_set_grid_cell(obj, LV_GRID_ALIGN_STRETCH, col, 1, LV_GRID_ALIGN_STRETCH, row, 1);

    lv_label_set_text(obj, text);
    lv_obj_add_style(obj, style, 0);
}

const char* get_chip_name(esp_chip_info_t* chip_info)
{
    switch (chip_info->model)
    {
        case CHIP_ESP32:
            return "ESP32";
        case CHIP_ESP32S2:
            return "ESP32-S2";
        case CHIP_ESP32S3:
            return "ESP32-S3";
        case CHIP_ESP32C3:
            return "ESP32-C3";
        case CHIP_ESP32H2:
            return "ESP32-H2";
        default:
            return "UNKNOWN";
    }
}

void demo_system_info(lv_obj_t* scr)
{
    lv_obj_t* label1 = lv_label_create(scr);
    lv_obj_align(label1, LV_ALIGN_TOP_MID, 0, 0);
    lv_label_set_text(label1, "System Information");

    static lv_coord_t col_dsc[] = { 160, 80, LV_GRID_TEMPLATE_LAST };
    static lv_coord_t row_dsc[] = { 20, 20, 20, 20, 20, LV_GRID_TEMPLATE_LAST };

    lv_obj_t* cont = lv_obj_create(scr);
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 300, 170);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_radius(&style, 5);
    lv_style_set_border_width(&style, 1);
    lv_style_set_bg_opa(&style, LV_OPA_COVER);
    lv_style_set_bg_color(&style, lv_palette_lighten(LV_PALETTE_BLUE, 1));
    lv_style_set_border_color(&style, lv_color_black());
    lv_style_set_pad_left(&style, 5);
    lv_style_set_border_side(&style, LV_BORDER_SIDE_BOTTOM | LV_BORDER_SIDE_RIGHT);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    char value[32];
    const char* const labels[] = { "Chip", "Chip cores", "Bluetooth", "Silicon rev", "FLASH" };
    for (int row = 0; row < 5; row++)
    {
        create_label(&style, cont, labels[row], row, 0);

        switch (row)
        {
            case 0:
                snprintf(value, sizeof(value), "%s", get_chip_name(&chip_info));
                break;

            case 1:
                snprintf(value, sizeof(value), "%d", chip_info.cores);
                break;

            case 2:
                snprintf(value, sizeof(value), "%s %s",
                         (chip_info.features & BIT(0)) ? "BT" : "",
                         (chip_info.features & BIT(1)) ? "BLE" : "");
                break;

            case 3:
                snprintf(value, sizeof(value), "%d", chip_info.revision);
                break;

            case 4:
            {
                uint32_t flash_size;  
                esp_err_t ret = esp_flash_get_size(NULL, &flash_size);  
                if (ret == ESP_OK)
                {
                    snprintf(value, sizeof(value), "%lu MiB", flash_size / (1024 * 1024));
                }
                else
                {
                    snprintf(value, sizeof(value), "Unknown");
                }
                break;
            }
        }

        create_label(&style, cont, value, row, 1);
    }
}

#define MAX_DEMO_COUNT 4
lv_obj_t* screens[MAX_DEMO_COUNT];
static int screen_index = 0;

void swap_screen_timer_callback(lv_timer_t* timer)
{
    lv_scr_load_anim(screens[screen_index], LV_SCR_LOAD_ANIM_MOVE_LEFT, 1000, 1000, false);
    screen_index++;
    if (screen_index >= MAX_DEMO_COUNT)
    {
        screen_index = 0;
    }
}

void slides_demo(void)
{
    for (size_t i = 0; i < MAX_DEMO_COUNT; i++)
    {
        screens[i] = lv_obj_create(NULL);
    }

    size_t index = 0;
    demo_pm5544(screens[index++]);
    demo_system_info(screens[index++]);
    demo_meter(screens[index++]);
    demo_rhino(screens[index++]);

    assert(index <= MAX_DEMO_COUNT);

    lv_disp_load_scr(screens[screen_index++]);
    const uint32_t timeout_ms = 10000;
    lv_timer_create(swap_screen_timer_callback, timeout_ms, NULL);
}

void run_demo_single_slide(const GRAPHICS_MODE mode)
{
    ESP_LOGI(TAG, "Single Screen Demo");
    const int delta = 20;

    lv_video_disp_init(mode);

    lv_obj_t* scr = lv_obj_create(NULL);
    demo_pm5544(scr);
    lv_scr_load(scr);
}

void run_demo_slides(void)
{
#if LV_COLOR_DEPTH < 16
    const size_t pixel_count = 320 * 254;
    lv_color_t* lvgl_pixel_buffer = heap_caps_malloc(sizeof(lv_color_t) * pixel_count, MALLOC_CAP_8BIT);
    assert(lvgl_pixel_buffer);
    lv_video_disp_init_buf(PAL_320x256, lvgl_pixel_buffer, pixel_count);
#else
    //no memory for buffers
    ESP_LOGI(TAG, "Using direct framebuffer access. Expect tearing effect for animations.");
    lv_video_disp_init(PAL_320x200);
#endif
    ESP_LOGI(TAG, "Slides Demo");

    slides_demo();
}
#endif

static void setup_dac(void) {
    dac_output_enable(DAC_CHANNEL_2); // GPIO26
}

// Convert int16 PCM to 8-bit DAC
static inline uint8_t pcm16_to_dac8(int16_t sample) {
    return (sample >> 8) + 128;
}

/*void combo_task(void *arg) {
    ESP_LOGI(TAG, "Combined task started");
    setup_dac();

    HMP3Decoder hMP3Decoder = MP3InitDecoder();
    uint8_t *read_ptr = (uint8_t *)_binary_portal_mp3_start;
    int bytes_left = _binary_portal_mp3_end - _binary_portal_mp3_start;

    short *pcm_buf = heap_caps_malloc(1152*2*sizeof(short), MALLOC_CAP_8BIT);
    if (!pcm_buf) {
        ESP_LOGE(TAG, "Failed to allocate PCM buffer");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Running video start");
    run_demo_slides();
    ESP_LOGI(TAG, "Done starting video!");
    // Skip ID3 tag if present
    if (memcmp(read_ptr, "ID3", 3) == 0) {
        int tag_size = ((read_ptr[6] & 0x7F) << 21) |
                       ((read_ptr[7] & 0x7F) << 14) |
                       ((read_ptr[8] & 0x7F) << 7)  |
                       (read_ptr[9] & 0x7F);
        read_ptr += 10 + tag_size;
        bytes_left -= 10 + tag_size;
        ESP_LOGI(TAG, "Skipped ID3 tag (%d bytes)", tag_size);
    }

    MP3FrameInfo info;

    while (bytes_left > 0) {
        int err = MP3Decode(hMP3Decoder, &read_ptr, &bytes_left, pcm_buf, 0);
        if (err != 0) continue;

        MP3GetLastFrameInfo(hMP3Decoder, &info);
        for (int i = 0; i < info.outputSamps; i += info.nChans) {
            uint8_t val = (pcm_buf[i] >> 8) + 128;
            ESP_LOGI(TAG, "Outputting voltage! %u %u", val, bytes_left);
            dac_output_voltage(DAC_CHANNEL_2, val);
            esp_rom_delay_us(62); // crude 16 kHz
        }
        lv_task_handler();
    }

    free(pcm_buf);
    MP3FreeDecoder(hMP3Decoder);
    ESP_LOGI(TAG, "Audio finished");
    vTaskDelete(NULL);
}*/

// Core 1: audio task
void audio_task(void *arg) {
    setup_dac();
    HMP3Decoder hMP3Decoder = MP3InitDecoder();

    uint8_t *read_ptr = (uint8_t *)_binary_portal_mp3_start;
    int bytes_left = _binary_portal_mp3_end - _binary_portal_mp3_start;
    short *pcm_buf = heap_caps_malloc(1152*2*sizeof(short), MALLOC_CAP_8BIT);

    if (!pcm_buf) vTaskDelete(NULL);

    // Skip ID3 tag
    if (memcmp(read_ptr, "ID3", 3) == 0) {
        int tag_size = ((read_ptr[6] & 0x7F) << 21) |
                       ((read_ptr[7] & 0x7F) << 14) |
                       ((read_ptr[8] & 0x7F) << 7) |
                       (read_ptr[9] & 0x7F);
        read_ptr += 10 + tag_size;
        bytes_left -= 10 + tag_size;
    }

    MP3FrameInfo info;

    while (bytes_left > 0) {
        int err = MP3Decode(hMP3Decoder, &read_ptr, &bytes_left, pcm_buf, 0);
        if (err) continue;
        MP3GetLastFrameInfo(hMP3Decoder, &info);

        int sample_index = 0;
        while (sample_index < info.outputSamps) {
            int batch_end = sample_index + AUDIO_BATCH * info.nChans;
            if (batch_end > info.outputSamps) batch_end = info.outputSamps;

            for (int i = sample_index; i < batch_end; i += info.nChans) {
                dac_output_voltage(DAC_CHANNEL_2, (pcm_buf[i] >> 8) + 128);
            }

            sample_index = batch_end;
            vTaskDelay(pdMS_TO_TICKS(AUDIO_BATCH * 1000 / AUDIO_SAMPLE_RATE));
        }
    }

    free(pcm_buf);
    MP3FreeDecoder(hMP3Decoder);
    vTaskDelete(NULL);
}

void video_task(void *arg) {
    ESP_LOGI(TAG, "Starting video task");
    run_demo_slides();
    while (1) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 FPS
    }
}

void app_main(void)
{

    // Configure the GPIO
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    int i = 0;
    while (i < 1) {
        // Turn the LED on
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // Turn the LED off
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        i++;
    }
    
    ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_0, CONFIG_ESPTOOLPY_MONITOR_BAUD));

    //  esp_log_level_set("*", ESP_LOG_INFO);
     esp_log_level_set("*", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "Application start...");
    ESP_LOGD(TAG, "DEBUG Output enabled");

    ESP_LOGI(TAG, "CPU Speed %d MHz", CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
    assert(240==CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
#if VIDEO_DIAG_ENABLE_INTERRUPT_STATS
    ESP_LOGI(TAG, "Interrupt timing stats enabled");
#endif
    // Create audio task on core 1
    xTaskCreatePinnedToCore(audio_task, "audio_task", AUDIO_TASK_STACK, NULL, 5, NULL, 1);

    // Create video task on core 1 (or core 0 if audio needs priority)
    /*xTaskCreatePinnedToCore(
        video_task,
        "video_task",
        VIDEO_TASK_STACK,
        NULL,
        4,               // Lower priority than audio
        NULL,
        0
    );*/
    /*ESP_LOGI(TAG, "Running demo slides!");
    run_demo_slides();

    ESP_LOGI(TAG, "Running audio!");
    dac_output_enable(DAC_CHANNEL_2);  // GPIO26
    //video_test_ntsc(VIDEO_TEST_CHECKERS);

    HMP3Decoder hMP3Decoder = MP3InitDecoder();
    uint8_t *read_ptr = (uint8_t *)_binary_portal_mp3_start;
    int bytes_left = _binary_portal_mp3_end - _binary_portal_mp3_start;

    // Skip ID3v2 tag if present
    if (memcmp(read_ptr, "ID3", 3) == 0) {
        int tag_size = ((read_ptr[6] & 0x7F) << 21) |
                       ((read_ptr[7] & 0x7F) << 14) |
                       ((read_ptr[8] & 0x7F) << 7)  |
                       (read_ptr[9] & 0x7F);
        read_ptr += 10 + tag_size;
        bytes_left -= 10 + tag_size;
        //printf("Skipped ID3 tag (%d bytes)\n", tag_size);
    }

    short pcm[1152 * 2]; // stereo frame buffer
    while (bytes_left > 0) {
        int offset = MP3FindSyncWord(read_ptr, bytes_left);
        if (offset < 0) break; // no sync found
        read_ptr += offset;
        bytes_left -= offset;

        int err = MP3Decode(hMP3Decoder, &read_ptr, &bytes_left, pcm, 0);
        if (err) {
            printf("MP3Decode error %d, bytes_left=%d\n", err, bytes_left);
            continue;
        }

        MP3FrameInfo info;
        MP3GetLastFrameInfo(hMP3Decoder, &info);
        //printf("Decoded frame: %d Hz, %d channels, %d samples\n",
               //info.samprate, info.nChans, info.outputSamps);

        // Send PCM to DAC, I2S, etc. (scaled 0–255 for DAC)
        for (int i = 0; i < info.outputSamps; i++) {
            uint8_t val = (pcm[i] >> 8) + 128;
            dac_output_voltage(DAC_CHANNEL_2, val);
            esp_rom_delay_us(50); // crude ~8kHz
        }
    }

    MP3FreeDecoder(hMP3Decoder);

    dac_output_disable(DAC_CHANNEL_2);*/
    

#if CONFIG_VIDEO_ENABLE_LVGL_SUPPORT
    ESP_LOGI(TAG, "Running other demo slides!");
    //run_demo_slides();
    // run_demo_single_slide(NTSC_320x200);
#else

#if CONFIG_VIDEO_DIAG_DISPLAY_TEST_FUNC
    // The following works without LVGL
    video_test_pal(VIDEO_TEST_PM5544);
    // video_test_ntsc(VIDEO_TEST_PM5544);
#else
#pragma GCC error "Nothing to show. Enable LVGL support or demo functions in the library."
#endif    

#endif
}