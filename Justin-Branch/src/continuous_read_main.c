/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include <dirent.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_spiffs.h"
#include "driver/ledc.h"
#include "esp_err.h"

//Def for Buzzer
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (2) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

//Def for ADC
#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define EXAMPLE_READ_LEN                    256
#define ADC_MAX_VAL                         4095
#define ADC_REF_VAL                         1700 //3.3 Volts for comparison if harmful gas


static adc_channel_t channel[1] = {ADC_CHANNEL_3};
static TaskHandle_t s_task_handle;
static const char *TAG = "ADC UNIT";

//data logging into SPIFFS

static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", //determining path for NVM
        .partition_label = "spiffs", //labeling at SPIFFS
        .max_files = 5,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total: %d bytes, used: %d bytes", total, used);
    } else {
        ESP_LOGE(TAG, "SPIFFS info failed (%s)", esp_err_to_name(ret));
    }
}


static const char *TAG1 = "spiffs_list";

void list_spiffs_files(void)
{
    DIR *dir = opendir("/spiffs");
    if (dir == NULL) {
        ESP_LOGE(TAG1, "Failed to open directory");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG1, "Found file: %s", entry->d_name);
    }

    closedir(dir);
}

//PWM Buzzer
static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1000, //adc samples at 1000Hz
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void read_log_file(const char *file_path)
{
    FILE *f = fopen(file_path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open");
        return;
    }

    ESP_LOGI(TAG, "Reading log file: %s", file_path);

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        // Print each line to the console/log
        printf("%s", line);
    }

    fclose(f);
}

#define MAX_LOG_FILE_SIZE  (100 * 1024)  // 100 KB max log file size

static uint32_t raw_to_voltage(uint32_t raw)
{
    return (raw * ADC_REF_VAL) / ADC_MAX_VAL;
}


void log_adc_data_to_spiffs(adc_continuous_data_t *parsed_data, uint32_t num_samples)
{
    FILE *f = fopen("/spiffs/adc_log.txt", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    for (uint32_t i = 0; i < num_samples; i++) {
        if (parsed_data[i].valid) {
            uint32_t voltage_mv = raw_to_voltage(parsed_data[i].raw_data);
            if(voltage_mv >= 3300)
            {
                fprintf(f, "ADC%d, Channel: %d, Value: %" PRIu32 "\n",
                    parsed_data[i].unit + 1,
                    parsed_data[i].channel,
                    parsed_data[i].raw_data);
            }
        }
    }
    fclose(f);
}

void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    init_spiffs();
    s_task_handle = xTaskGetCurrentTaskHandle();

    //Buzzer
    

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    if (log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open log file for writing");
    }

    while (1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int j = 0;
        while (j < 1) {
            // Set the LEDC peripheral configuration
            example_ledc_init();
            int i = 1;
            // Set duty to 50%
            if(i == 1)
            {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            }
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

                adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
                uint32_t num_parsed_samples = 0;

                esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if (parse_ret == ESP_OK) {
                    for (int i = 0; i < num_parsed_samples; i++) {
                        if (parsed_data[i].valid) {
                            ESP_LOGI(TAG, "ADC%d, Channel: %d, Voltage: %"PRIu32,
                                     parsed_data[i].unit + 1,
                                     parsed_data[i].channel,
                                     parsed_data[i].raw_data);
                        } else {
                            ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
                                     parsed_data[i].unit + 1,
                                     parsed_data[i].channel,
                                     parsed_data[i].raw_data);
                        }
                    }
                    log_adc_data_to_spiffs(parsed_data, num_parsed_samples);
                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }

                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                vTaskDelay(pdMS_TO_TICKS(1000)); //wait 1 second
                //read_log_file("/spiffs/adc_log.txt");
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    read_log_file("/spiffs/adc_log.txt");

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
