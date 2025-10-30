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

#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "esp_zigbee_cluster.h"

#define TAG "MAIN"

// Zigbee endpoint
#define HA_ONOFF_SWITCH_ENDPOINT 0x01
#define ESP_MANUFACTURER_NAME "MyCompany"
#define ESP_MODEL_IDENTIFIER  "MySwitch"

// ADC pin
#define ADC_CHANNEL ADC1_CHANNEL_6 // example GPIO34

static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
}

static void read_adc_and_log(void *pvParameters)
{
    while (1) {
        int adc_raw = adc1_get_raw(ADC_CHANNEL);
        ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);

        FILE *f = fopen("/spiffs/adc_log.txt", "a");
        if (f != NULL) {
            fprintf(f, "ADC: %d\n", adc_raw);
            fclose(f);
        } else {
            ESP_LOGE(TAG, "Failed to open file for writing");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // every 2 seconds
    }
}

static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting Zigbee stack");

    /* Zigbee network configuration */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    ESP_ERROR_CHECK(esp_zb_init(&zb_nwk_cfg));

    /* Create endpoint for On/Off switch */
    esp_zb_ep_list_t *ep = esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, NULL);

    /* Create cluster list */
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster configuration */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_VERSION,
        .application_version = 1,
        .stack_version = 1,
        .hw_version = 1,
        .manufacturer_name = (uint8_t *)ESP_MANUFACTURER_NAME,
        .model_identifier = (uint8_t *)ESP_MODEL_IDENTIFIER,
    };
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list,
                                                         basic_attr_list,
                                                         ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* On/Off cluster configuration */
    esp_zb_on_off_attr_t onoff_attr = { 0 };
    esp_zb_attribute_list_t *onoff_attr_list = esp_zb_on_off_cluster_create(&onoff_attr);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list,
                                                          onoff_attr_list,
                                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Register endpoint */
    ESP_ERROR_CHECK(esp_zb_device_register(ep, cluster_list));

    /* Set network channel mask */
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Run Zigbee main loop */
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing NVS");
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Initializing SPIFFS");
    init_spiffs();

    ESP_LOGI(TAG, "Configuring ADC");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Start ADC logging task
    xTaskCreate(read_adc_and_log, "adc_log_task", 4096, NULL, 5, NULL);

    // Start Zigbee task
    xTaskCreate(esp_zb_task, "zigbee_task", 8192, NULL, 5, NULL);
}
