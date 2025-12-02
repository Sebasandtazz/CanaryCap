//Canary Cap Team 30 - ECE 49022

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include <dirent.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_spiffs.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_check.h"
#include "string.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_switch.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "led_strip.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "zb_alert.h"
#include "zb_cluster_manager.h"
#include "zb_mesh_network.h"
#include "zb_canary_cluster.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "esp_mac.h"
#include "dirent.h"
#include "stdlib.h"

/*
 * Build role selection:
 * Uncomment the following line to compile this firmware as a Zigbee ROUTER.
 * When commented out, the firmware builds as a COORDINATOR by default.
 */
#define ZB_BUILD_AS_ROUTER
#define ZB_PERMIT_JOIN_DURATION 600
/* Periodic reopen interval (seconds): how often coordinator re-opens network to allow joins */
#define ZB_REOPEN_INTERVAL_SECONDS 300

//Definitions for SPI and ADXL345
static const char *TAGSPI = "ADXL345";

#define I2C_MASTER_SCL_IO           11 // SCL on ESP32-C6
#define I2C_MASTER_SDA_IO           10 // SDA on ESP32-C6
#define LED_GPIO1                   19 //CONFIG_LED_GPIO
#define LED_GPIO2                   20 //CONFIG_LED_GPIO
#define LED_ONBOARD_GPIO            8  // Onboard RGB LED for ESP32-C6-DevKit

#define BUTTON_GPIO                 23  // Boot button on ESP32-C6-DevKit
#define BUZZER_GPIO                 18 //CONFIG_BUZZER_GPIO

static led_strip_handle_t s_led_strip = NULL;

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000  // Start with standard 100kHz I2C speed
#define I2C_MASTER_TIMEOUT_MS       5000    // Increased timeout for reliability

#define ADXL345_ADDR                0x53
#define ADXL345_DEVID_REG           0x00
#define ADXL345_POWER_CTL_REG       0x2D
#define ADXL345_DATA_FORMAT_REG     0x31
#define ADXL345_DATAX0_REG          0x32

#define ADXL345_DEVID_EXPECTED      0xE5
#define LSB_TO_G                    0.0039f  // 3.9 mg/LSB for ±2g range

#define THRESH_SEVERE_G             1.0f
#define THRESH_MODERATE_G           0.5f
#define MAX_LOG_FILE_SIZE           (100 * 1024)  // 100 KB max log file size

static TimerHandle_t major_impact_timer;
static bool major_impact_active = false;
static volatile bool major_impact_cancelled_flag = false;
static const char *TAG1 = "spiffs_list";

/* Button press tracking for factory reset */
static volatile uint32_t button_press_time = 0;
static volatile bool button_pressed = false;
static int impact_count = 0;

//Def for Buzzer
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (19) // Define the output GPIO
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
#define ADC_REF_VAL                         3300 // Reference in mV (approx 3.3V Vref)

//Def for zb
#if defined ZB_ED_ROLE
#error Define ZB_COORDINATOR_ROLE in idf.py menuconfig to compile light switch source code.
#endif
typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

static const char *TAGZB = "Zigbee";


static adc_channel_t channel[1] = {ADC_CHANNEL_3};  // GPIO3 on ESP32-C6 for gas sensor
static TaskHandle_t adc_task_handle = NULL;
static TaskHandle_t i2c_task_handle = NULL;
static TaskHandle_t s_task_handle = NULL;
static const char *TAGADC = "ADC UNIT";

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
        ESP_LOGE(TAGADC, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAGADC, "SPIFFS mounted: total: %d bytes, used: %d bytes", total, used);
    } else {
        ESP_LOGE(TAGADC, "SPIFFS info failed (%s)", esp_err_to_name(ret));
    }
}
// ============================================================================
// ADXL345 functions adapted for software I2C
// ============================================================================

static esp_err_t adxl345_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t adxl345_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = { reg_addr, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

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


void read_log_file(const char *file_path)
{
    FILE *f = fopen(file_path, "r");
    if (f == NULL) {
        ESP_LOGE(TAGSPI, "Failed to open");
        return;
    }

    ESP_LOGI(TAGSPI, "Reading log file: %s", file_path);

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        // Print each line to the console/log
        printf("%s", line);
    }

    fclose(f);
}

void log_data_to_spiffs(float mag, float x_g, float y_g, float z_g)
{
    if(gpio_get_level(BUTTON_GPIO)==0){
        FILE *f = fopen("/spiffs/i2c_log.txt", "w");
        impact_count=0;
        gpio_set_level(LED_GPIO2,1);
        fclose(f);
        return;
    }
    FILE *f = fopen("/spiffs/i2c_log.txt", "a");
    if (f == NULL) {
        ESP_LOGE(TAGSPI, "Failed to open file for writing");
        return;
    }

        fprintf(f, "Magnitude: %f, X-G: %f, Y-G: %f, Z-G: %f \n",
                mag,
                x_g,
                y_g,
                z_g);
    fclose(f);

    read_log_file("/spiffs/i2c_log.txt");
}


static void init_led(int gpio){
    gpio_config_t io_conf = {.pin_bit_mask=1ULL<<gpio,.mode=GPIO_MODE_OUTPUT,.pull_up_en=0,.pull_down_en=0,.intr_type=GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(gpio,1);  // Start OFF (active LOW)
}

static void init_onboard_led(void){
    led_strip_config_t led_strip_conf = {
        .max_leds = 1,
        .strip_gpio_num = LED_ONBOARD_GPIO,
    };
    led_strip_rmt_config_t rmt_conf = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &s_led_strip));
    /* Turn off LED initially */
    led_strip_set_pixel(s_led_strip, 0, 0, 0, 0);
    led_strip_refresh(s_led_strip);
}

static void set_onboard_led(uint8_t r, uint8_t g, uint8_t b){
    if (s_led_strip) {
        led_strip_set_pixel(s_led_strip, 0, r, g, b);
        led_strip_refresh(s_led_strip);
    }
}

static void flash_led(int GPIO){
    for(int i=0;i<10;i++){
        gpio_set_level(GPIO,0);  // ON (active LOW)
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO,1);  // OFF (active LOW)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Non-blocking LED blink task — start_led_blink/stop_led_blink
   allows the LED to flash in background while main loop collects data.
*/
static TaskHandle_t led_blink_task_handle = NULL;
static int led_blink_gpio = -1;

typedef struct {
    int gpio;
    uint32_t on_ms;
    uint32_t off_ms;
} led_blink_params_t;

static void led_blink_task(void *arg)
{
    led_blink_params_t params = *(led_blink_params_t *)arg;
    free(arg);

    while (1) {
        gpio_set_level(params.gpio, 0); // on (active low in this board)
        vTaskDelay(pdMS_TO_TICKS(params.on_ms));
        gpio_set_level(params.gpio, 1); // off
        vTaskDelay(pdMS_TO_TICKS(params.off_ms));
    }
}

static void start_led_blink(int gpio, uint32_t on_ms, uint32_t off_ms)
{
    if (led_blink_task_handle != NULL) return; // already running
    led_blink_gpio = gpio;
    led_blink_params_t *p = malloc(sizeof(led_blink_params_t));
    if (!p) return;
    p->gpio = gpio;
    p->on_ms = on_ms;
    p->off_ms = off_ms;
    xTaskCreate(led_blink_task, "led_blink", 2048, p, tskIDLE_PRIORITY + 1, &led_blink_task_handle);
}

static void stop_led_blink(void)
{
    if (led_blink_task_handle == NULL) return;
    vTaskDelete(led_blink_task_handle);
    led_blink_task_handle = NULL;
    if (led_blink_gpio >= 0) {
        gpio_set_level(led_blink_gpio, 1); // ensure LED off
        led_blink_gpio = -1;
    }
}


/* Buzzer using LEDC hardware PWM to avoid blocking the main task.
   We create a short one-shot FreeRTOS timer to stop the PWM after the
   requested duration so buzzer_beep_ms() is non-blocking.
*/

/* Select LEDC speed mode; fall back to low speed if symbol not defined for this target */
#if defined(LEDC_HIGH_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_HIGH_SPEED_MODE
#elif defined(LEDC_LOW_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#else
#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#endif

static void buzzer_stop_timer_cb(TimerHandle_t xTimer)
{
    ledc_set_duty(BUZZER_LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, LEDC_CHANNEL);
    ledc_stop(BUZZER_LEDC_MODE, LEDC_CHANNEL, 0);
    xTimerDelete(xTimer, 0);
}

static void buzzer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void buzzer_beep_ms(uint32_t freq_hz, uint32_t duration_ms)
{
    ledc_set_freq(BUZZER_LEDC_MODE, LEDC_TIMER, freq_hz);
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    ledc_set_duty(BUZZER_LEDC_MODE, LEDC_CHANNEL, max_duty / 2);
    ledc_update_duty(BUZZER_LEDC_MODE, LEDC_CHANNEL);

    TimerHandle_t t = xTimerCreate("bstop", pdMS_TO_TICKS(duration_ms), pdFALSE, NULL, buzzer_stop_timer_cb);
    if (t) {
        xTimerStart(t, 0);
    }
}

static const char* infer_impact_type(float ax,float ay,float az,float mag){
    
    if(mag>THRESH_SEVERE_G && az<-1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Fall";
    if(mag>THRESH_MODERATE_G && fabsf(ax)>0.5f && fabsf(ay)>0.5f && fabsf(az)>0.5f) return "Ceiling Collapse";
    if(mag>THRESH_SEVERE_G && az>1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Hard Landing";
    if(mag>THRESH_MODERATE_G && ax>0.5f) return "Front Impact";
    if(mag>THRESH_MODERATE_G && ax<-0.5f) return "Rear Impact";
    if(mag>THRESH_MODERATE_G && ay>0.5f) return "Right Side Impact";
    if(mag>THRESH_MODERATE_G && ay<-0.5f) return "Left Side Impact";
    
    if(mag>THRESH_MODERATE_G) return "blunt";
    return "none";
}

static void classify_impact(float x_g, float y_g, float z_g){
    float mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
    if(x_g > THRESH_SEVERE_G || x_g < -THRESH_SEVERE_G || y_g > THRESH_SEVERE_G || y_g < -THRESH_SEVERE_G || z_g > THRESH_SEVERE_G || z_g < -THRESH_SEVERE_G){
            ESP_LOGW(TAGSPI, "Major Impact Detected: Starting Timer!");
            if(!major_impact_active){
                major_impact_active=true;
                xTimerStart(major_impact_timer,0);
                log_data_to_spiffs(mag, x_g, y_g, z_g);
                /* notify Zigbee network (debounced) */
                //send_zigbee_alert_toggle_debounced(5000);
            }
            
            start_led_blink(LED_GPIO1, 100, 100);
            //buzzer_beep_ms(4000, 5000);

            
    }
    else if(x_g > THRESH_MODERATE_G || x_g < -THRESH_MODERATE_G || y_g > THRESH_MODERATE_G || y_g < -THRESH_MODERATE_G || z_g > THRESH_MODERATE_G || z_g < -THRESH_MODERATE_G){
            ESP_LOGW(TAGSPI, "Minor Impact!");
            log_data_to_spiffs(mag, x_g, y_g, z_g);
            impact_count++;
    }
    
      const char* impact_type = infer_impact_type(x_g,y_g,z_g,mag);
    if(strcmp(impact_type, "none")!=0){
        ESP_LOGE(TAGSPI, "Impact detected! Type: %s, Magnitude: %.3f g", impact_type, mag);
    }

    /* Determine severity and send via cluster manager */
    canary_impact_severity_t severity = CANARY_IMPACT_NONE;
    canary_impact_type_t cluster_type = CANARY_IMPACT_TYPE_UNKNOWN;
    
    if (mag > THRESH_SEVERE_G) {
        severity = CANARY_IMPACT_SEVERE;
    } else if (mag > THRESH_MODERATE_G) {
        severity = CANARY_IMPACT_MAJOR;
    } else {
        severity = CANARY_IMPACT_MINOR;
    }
    
    /* Map impact type string to cluster enum */
    if (strcmp(impact_type, "Fall") == 0) cluster_type = CANARY_IMPACT_TYPE_FALL;
    else if (strcmp(impact_type, "Ceiling Collapse") == 0) cluster_type = CANARY_IMPACT_TYPE_CEILING_COLLAPSE;
    else if (strcmp(impact_type, "Hard Landing") == 0) cluster_type = CANARY_IMPACT_TYPE_HARD_LANDING;
    else if (strcmp(impact_type, "Front Impact") == 0) cluster_type = CANARY_IMPACT_TYPE_FRONT;
    else if (strcmp(impact_type, "Rear Impact") == 0) cluster_type = CANARY_IMPACT_TYPE_REAR;
    else if (strcmp(impact_type, "Right Side Impact") == 0) cluster_type = CANARY_IMPACT_TYPE_RIGHT_SIDE;
    else if (strcmp(impact_type, "Left Side Impact") == 0) cluster_type = CANARY_IMPACT_TYPE_LEFT_SIDE;
    else if (strcmp(impact_type, "blunt") == 0) cluster_type = CANARY_IMPACT_TYPE_BLUNT;
    
    /* Send impact alert via cluster manager */
    if (severity >= CANARY_IMPACT_MAJOR) {
        zb_cluster_send_impact_alert(severity, cluster_type, mag, x_g, y_g, z_g);
        
        /* Local alerts for severe impacts */
        if (severity == CANARY_IMPACT_SEVERE) {
            start_led_blink(LED_GPIO2, 150, 150);
            buzzer_beep_ms(3000, 3000);
        }
    }
   
    if(impact_count>=5 && mag>THRESH_MODERATE_G){
            ESP_LOGW(TAGSPI, "Multiple impacts detected (%d)! Go see a doctor!", impact_count);
            //impact_count=0;
            gpio_set_level(LED_GPIO2,0);
        }
    //float mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
}

/*
void log_to_spiffs(float mag){
fopen(
fwrite(
fclose(
}
*/
static void IRAM_ATTR button_handler(void* arg){
    /* ISR: Track button press/release for factory reset */
    uint32_t level = gpio_get_level(BUTTON_GPIO);
    
    if (level == 0) {
        /* Button pressed (active low) */
        button_pressed = true;
        button_press_time = xTaskGetTickCountFromISR();
    } else {
        /* Button released */
        button_pressed = false;
    }
    
    /* Legacy: Stop major impact timer if active */
    if (major_impact_active && level == 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t stopped = xTimerStopFromISR(major_impact_timer, &xHigherPriorityTaskWoken);
        if (stopped == pdPASS) {
            major_impact_active = false;
            major_impact_cancelled_flag = true;
        }
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void major_impact_timer_callback(TimerHandle_t xTimer){
    /* If cancellation was requested (set from ISR), skip executing the
       warning actions. The flag is volatile and may be set from the ISR.
    */
    if (major_impact_cancelled_flag) {
        /* Clear the flag here and don't perform the warning */
        major_impact_cancelled_flag = false;
        major_impact_active = false;
        stop_led_blink();
        return;
    }

    stop_led_blink();
    ESP_LOGW(TAGSPI, "Major impact! WARNING!");
    //flash_led(LED_GPIO);
    gpio_set_level(LED_GPIO1,0);
    major_impact_active = false;
}

//PWM Buzzer
// static void example_ledc_init(void)
// {
//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_MODE,
//         .duty_resolution  = LEDC_DUTY_RES,
//         .timer_num        = LEDC_TIMER,
//         .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel = {
//         .speed_mode     = LEDC_MODE,
//         .channel        = LEDC_CHANNEL,
//         .timer_sel      = LEDC_TIMER,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = LEDC_OUTPUT_IO,
//         .duty           = 0, // Set duty to 0%
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
// }


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);

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

        ESP_LOGI(TAGADC, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAGADC, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAGADC, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    // Register event callback so ADC task gets notified when data is ready
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    *out_handle = handle;
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
        ESP_LOGE(TAGADC, "Failed to open file for writing");
        return;
    }

    for (uint32_t i = 0; i < num_samples; i++) {
        if (parsed_data[i].valid) {
            uint32_t voltage_mv = raw_to_voltage(parsed_data[i].raw_data);
            if(voltage_mv >= 3300)
            {
                /* Send gas alert via cluster manager */
                zb_cluster_send_gas_alert(true, voltage_mv, parsed_data[i].raw_data);
                
                /* Local alert indicators */
                start_led_blink(LED_GPIO2, 200, 200);
                buzzer_beep_ms(2000, 3000);

                fprintf(f, "ADC%d, Channel: %d, Value: %" PRIu32 " (GAS ALERT)\n",
                    parsed_data[i].unit + 1,
                    parsed_data[i].channel,
                    parsed_data[i].raw_data);
            }
        }
    }
    fclose(f);
}

void adc_task(void *param)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    adc_continuous_handle_t handle = (adc_continuous_handle_t)param;


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
        ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) {
            ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

            adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t num_parsed_samples = 0;

            esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
            if (parse_ret == ESP_OK) {
                for (int i = 0; i < num_parsed_samples; i++) {
                    if (parsed_data[i].valid) {
                        uint32_t raw = parsed_data[i].raw_data;
                        uint32_t voltage_mv = raw_to_voltage(raw);
                        if(i % 32 == 0) { // Log every 32nd valid sample
                        ESP_LOGI(TAGADC, "ADC%d Ch%d raw=%"PRIu32" mV=%"PRIu32,
                                parsed_data[i].unit + 1,
                                parsed_data[i].channel,
                                raw,
                                voltage_mv);
                            } 
                        }
                        else {
                        ESP_LOGW(TAGADC, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
                                parsed_data[i].unit + 1,
                                parsed_data[i].channel,
                                parsed_data[i].raw_data);
                        }
                    }
                    log_adc_data_to_spiffs(parsed_data, num_parsed_samples);
            } 
            else {
                ESP_LOGE(TAGADC, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }

                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
            vTaskDelay(pdMS_TO_TICKS(1000)); //wait 1 second
                //read_log_file("/spiffs/adc_log.txt");
        } 
            else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
                }
    }
}

// --- Zigbee mesh network callbacks ---

static void mesh_state_change_callback(zb_mesh_state_t old_state, zb_mesh_state_t new_state)
{
    ESP_LOGW(TAGZB, "*** MESH STATE CHANGED: %d -> %d ***", old_state, new_state);
    
    if (new_state == ZB_MESH_STATE_JOINED) {
        /* Joined network - flash onboard LED and indicate success */
        ESP_LOGW(TAGZB, "==> DEVICE CONNECTED TO NETWORK <==");
        
        /* Flash onboard LED rapidly to indicate connection (green) */
        for (int i = 0; i < 5; i++) {
            set_onboard_led(0, 50, 0);  // Green ON
            vTaskDelay(pdMS_TO_TICKS(100));
            set_onboard_led(0, 0, 0);  // OFF
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        /* Keep onboard LED on while connected (solid green) */
        set_onboard_led(0, 20, 0);
        
        /* Other indicators */
        gpio_set_level(LED_GPIO2, 0);
        buzzer_beep_ms(2000, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(LED_GPIO2, 1);
        
    } else if (old_state == ZB_MESH_STATE_JOINED && new_state == ZB_MESH_STATE_DISCONNECTED) {
        /* Lost connection - turn off onboard LED and alert */
        ESP_LOGE(TAGZB, "==> DEVICE DISCONNECTED FROM NETWORK <==");
        
        /* Turn onboard LED red */
        set_onboard_led(20, 0, 0);
        
        /* Alert with blinking and beep */
        start_led_blink(LED_GPIO2, 300, 300);
        buzzer_beep_ms(2000, 500);
        
    } else if (new_state == ZB_MESH_STATE_FORMING) {
        ESP_LOGI(TAGZB, "Forming network...");
    } else if (new_state == ZB_MESH_STATE_JOINING) {
        ESP_LOGI(TAGZB, "Joining network...");
    }
}

static void mesh_device_event_callback(bool joined, uint16_t device_addr)
{
    if (joined) {
        ESP_LOGW(TAGZB, "*** NEW DEVICE JOINED NETWORK: 0x%04x ***", device_addr);
        
        /* Flash onboard LED to indicate new device (blue) */
        for (int i = 0; i < 3; i++) {
            set_onboard_led(0, 0, 50);  // Blue ON
            vTaskDelay(pdMS_TO_TICKS(150));
            set_onboard_led(0, 20, 0);  // Back to green
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        
        /* Quick beep */
        buzzer_beep_ms(2500, 150);
        
    } else {
        ESP_LOGE(TAGZB, "*** DEVICE LEFT NETWORK: 0x%04x ***", device_addr);
        
        /* Flash onboard LED slowly to indicate device left (red) */
        for (int i = 0; i < 2; i++) {
            set_onboard_led(30, 0, 0);  // Red ON
            vTaskDelay(pdMS_TO_TICKS(300));
            set_onboard_led(0, 20, 0);  // Back to green
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        /* Warning beep */
        buzzer_beep_ms(1500, 300);
    }
}

static void network_status_received_callback(const canary_network_status_msg_t *msg, uint16_t src_addr)
{
    if (msg->state == CANARY_NETWORK_CONNECTED) {
        ESP_LOGW(TAGZB, "*** REMOTE DEVICE CONNECTED: 0x%04x (PAN:0x%02x%02x CH:%d) ***",
                 src_addr, msg->pan_id_high, msg->pan_id_low, msg->channel);
    } else if (msg->state == CANARY_NETWORK_DISCONNECTED) {
        ESP_LOGE(TAGZB, "*** REMOTE DEVICE DISCONNECTED: 0x%04x ***", src_addr);
        
        /* Visual alert for remote disconnect */
        gpio_set_level(LED_GPIO1, 0);
        buzzer_beep_ms(1500, 200);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(LED_GPIO1, 1);
    } else {
        ESP_LOGI(TAGZB, "Network status from 0x%04x: %s",
                 src_addr, zb_cluster_network_state_to_string(msg->state));
    }
}

static void impact_alert_received_callback(const canary_impact_alert_msg_t *msg, uint16_t src_addr)
{
    float magnitude = msg->magnitude / 1000.0f;
    ESP_LOGW(TAGZB, "IMPACT ALERT from 0x%04x: %s (severity=%s, mag=%.2fg)",
             src_addr,
             zb_cluster_impact_type_to_string(msg->type),
             zb_cluster_impact_severity_to_string(msg->severity),
             magnitude);
    
    /* Visual/audio alert for remote impacts */
    if (msg->severity >= CANARY_IMPACT_MAJOR) {
        gpio_set_level(LED_GPIO1, 0); // Turn on LED
        buzzer_beep_ms(3000, 1000);
    }
}

static void gas_alert_received_callback(const canary_gas_alert_msg_t *msg, uint16_t src_addr)
{
    ESP_LOGW(TAGZB, "GAS ALERT from 0x%04x: detected=%d voltage=%dmV",
             src_addr, msg->detected, msg->voltage_mv);
    
    /* Visual/audio alert for remote gas detection */
    if (msg->detected) {
        gpio_set_level(LED_GPIO2, 0); // Turn on LED
        buzzer_beep_ms(2500, 1500);
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* Determine role based on build flag */
#ifdef ZB_BUILD_AS_ROUTER
    zb_mesh_role_t role = ZB_MESH_ROLE_ROUTER;
#else
    zb_mesh_role_t role = ZB_MESH_ROLE_COORDINATOR;
#endif

    /* Initialize mesh network */
    ESP_ERROR_CHECK(zb_mesh_init(role));
    
    /* Initialize cluster manager */
    ESP_ERROR_CHECK(zb_cluster_manager_init(role == ZB_MESH_ROLE_COORDINATOR));
    
    /* Register callbacks */
    zb_mesh_register_state_callback(mesh_state_change_callback);
    zb_mesh_register_device_callback(mesh_device_event_callback);
    zb_cluster_register_network_status_callback(network_status_received_callback);
    zb_cluster_register_impact_alert_callback(impact_alert_received_callback);
    zb_cluster_register_gas_alert_callback(gas_alert_received_callback);

    /* Configure Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = (role == ZB_MESH_ROLE_COORDINATOR) ? 
                       ESP_ZB_DEVICE_TYPE_COORDINATOR : ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = ZB_MESH_MAX_CHILDREN,
        },
    };

    ESP_LOGI(TAGZB, "Starting Zigbee mesh, role=%s", 
             (role == ZB_MESH_ROLE_COORDINATOR) ? "COORDINATOR" : "ROUTER");
    
    esp_zb_init(&zb_nwk_cfg);

    /* Create endpoint with on/off switch cluster for compatibility */
    esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *ep_list = esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);
    
    /* TODO: Add custom clusters to endpoint here if needed for attribute reporting */
    
    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ZB_MESH_PRIMARY_CHANNEL_MASK);
    
    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    /* Run main Zigbee stack loop - this handles all events */
    esp_zb_stack_main_loop();

}

/* Zigbee app signal handler - delegates to mesh network handler */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    /* Forward all signals to mesh network handler */
    zb_mesh_handle_signal(signal_struct);
}

/* Heartbeat task to show connection status */
static void button_monitor_task(void *pvParameters)
{
    const TickType_t hold_time_ticks = pdMS_TO_TICKS(3000);  // 3 seconds
    bool factory_reset_triggered = false;
    uint32_t last_debug_tick = 0;
    
    ESP_LOGI(TAGZB, "Button monitor task started - GPIO %d", BUTTON_GPIO);
    
    while (1) {
        /* Debug: Print button state every 2 seconds */
        uint32_t now = xTaskGetTickCount();
        if ((now - last_debug_tick) >= pdMS_TO_TICKS(2000)) {
            uint32_t level = gpio_get_level(BUTTON_GPIO);
            ESP_LOGI(TAGZB, "[DEBUG] Button GPIO %d level: %lu, pressed: %d", BUTTON_GPIO, level, button_pressed);
            last_debug_tick = now;
        }
        
        if (button_pressed && !factory_reset_triggered) {
            uint32_t hold_duration = xTaskGetTickCount() - button_press_time;
            
            ESP_LOGI(TAGZB, "Button held for %lu ms", (hold_duration * 1000) / configTICK_RATE_HZ);
            
            if (hold_duration >= hold_time_ticks) {
                /* Button held for 3 seconds - trigger factory reset */
                factory_reset_triggered = true;
                
                ESP_LOGW(TAGZB, "=== FACTORY RESET TRIGGERED ===");
                ESP_LOGW(TAGZB, "Button held for 3 seconds - erasing Zigbee network config...");
                
                /* Flash LED red rapidly to indicate factory reset */
                for (int i = 0; i < 10; i++) {
                    set_onboard_led(50, 0, 0);  // Red
                    vTaskDelay(pdMS_TO_TICKS(100));
                    set_onboard_led(0, 0, 0);   // Off
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                /* Perform factory reset */
                zb_mesh_factory_reset();
                
                ESP_LOGW(TAGZB, "Factory reset complete! Device will restart...");
                set_onboard_led(50, 0, 0);  // Solid red before restart
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                /* Restart ESP */
                esp_restart();
            }
        } else if (!button_pressed) {
            /* Reset flag when button is released */
            factory_reset_triggered = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
}

static void heartbeat_task(void *pvParameters)
{
    int pulse_count = 0;
    while (1) {
        bool is_joined = zb_mesh_is_joined();
        uint16_t short_addr = esp_zb_get_short_address();
        
        if (is_joined && short_addr != 0xFFFF) {
            /* Connected - pulse onboard LED every 2 seconds */
            ESP_LOGI(TAGZB, "[HEARTBEAT] Connected to network - Short Addr: 0x%04x", short_addr);
            
            /* Quick pulse to show we're alive */
            set_onboard_led(0, 50, 0);  // Bright green
            vTaskDelay(pdMS_TO_TICKS(100));
            set_onboard_led(0, 20, 0);  // Dim green
            
            pulse_count++;
            if (pulse_count % 5 == 0) {  // Every 10 seconds
                ESP_LOGW(TAGZB, "========== NETWORK STATUS: CONNECTED (0x%04x) ==========", short_addr);
            }
        } else {
            /* Not connected - show searching pattern */
            ESP_LOGW(TAGZB, "[HEARTBEAT] Not connected - Short Addr: 0x%04x (searching...)", short_addr);
            
            /* Slow blink to show searching (yellow) */
            set_onboard_led(30, 15, 0);  // Yellow ON
            vTaskDelay(pdMS_TO_TICKS(200));
            set_onboard_led(0, 0, 0);  // OFF
            vTaskDelay(pdMS_TO_TICKS(800));
            
            pulse_count = 0;
            continue;  // Skip the normal delay
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second heartbeat
    }
}


static void i2c_task(void *arg)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)arg;
    uint8_t data[6];

    while (1) {
        // Read 6 bytes of acceleration data
        ESP_ERROR_CHECK(adxl345_register_read(dev_handle, ADXL345_DATAX0_REG, data, 6));
        
        // ADXL345 outputs axis data in little-endian format by default.
        // If sensor configuration changes, update the axis data parsing accordingly.
        //gpio_set_level(LED_GPIO,1);
    
        int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
        int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
        int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

        float x_g = x_raw * LSB_TO_G;
        float y_g = y_raw * LSB_TO_G;
        float z_g = z_raw * LSB_TO_G -1.0f;//gravity 
        
        
        //ESP_LOGI(TAGSPI, "X=%.3f g, Y=%.3f g, Z=%.3f g", x_g, y_g, z_g);
        classify_impact(x_g, y_g, z_g);
        

        /* If the button cancelled a major impact warning in the ISR,
           perform the non-ISR-safe logging here in task context. */
        if (major_impact_cancelled_flag) {
            major_impact_cancelled_flag = false;
            ESP_LOGI(TAGSPI, "Major impact warning cancelled by button!");
            stop_led_blink();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    init_spiffs();
    /* use global adc_task_handle (declared above) */

    esp_err_t retzb = nvs_flash_init();
    if (retzb == ESP_ERR_NVS_NO_FREE_PAGES || retzb == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or needs erasing
        ESP_ERROR_CHECK(nvs_flash_erase());
        retzb = nvs_flash_init();
    }
    ESP_ERROR_CHECK(retzb);

    // uint8_t data[6];
    // uint8_t devid = 0;
    // i2c_master_bus_handle_t bus_handle;
    // i2c_master_dev_handle_t dev_handle;

    // // Initialize I2C and attach ADXL345
    // i2c_master_init(&bus_handle, &dev_handle);
    // ESP_LOGI(TAGSPI, "I2C initialized successfully");

    // // Verify ADXL345 identity
    // esp_err_t ret = adxl345_register_read(dev_handle, ADXL345_DEVID_REG, &devid, 1);
    // if (ret == ESP_OK) {
    //     ESP_LOGI(TAGSPI, "ADXL345 DEVID = 0x%02X", devid);
    //     if (devid != ADXL345_DEVID_EXPECTED) {
    //         ESP_LOGE(TAGSPI, "ADXL345 not detected! Expected 0xE5.");
    //         // continue without accelerometer
    //     }
    // } else {
    //     ESP_LOGE(TAGSPI, "Failed to communicate with ADXL345 at 0x%02X", ADXL345_ADDR);
    // }

    // // Enable measurement mode
    // ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle, ADXL345_POWER_CTL_REG, 0x08));

    // // Set data format: Full resolution, ±2g range (0x08)
    // ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle, ADXL345_DATA_FORMAT_REG, 0x08));

    init_led(LED_GPIO1);
    init_led(LED_GPIO2);
    init_onboard_led();  // Initialize onboard RGB LED for ESP32-C6
    buzzer_init();
    
    ESP_LOGI(TAGZB, "Onboard RGB LED initialized on GPIO %d", LED_ONBOARD_GPIO);

    // Configure button input
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // enable pull-up
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE, // both edges for press/release detection
    };
    gpio_config(&io_conf);

    // Install ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_handler, NULL);

    // Create major impact timer (10 sec)
    major_impact_timer = xTimerCreate("MajorImpactTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, major_impact_timer_callback);

    xTaskCreate(i2c_task, "i2c_task", 4096, dev_handle, 5, NULL);

    /* ADC and I2C tasks enabled */
    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    /* Create the ADC task and store its handle before registering callbacks
       or starting the ADC. This prevents the ADC ISR from attempting to
       notify a NULL task handle if conversions occur immediately. */
    xTaskCreate(adc_task, "adc_task", 4096, handle, 5, &adc_task_handle);
    s_task_handle = adc_task_handle;

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    
    ESP_LOGI(TAGZB, "ADC and I2C tasks enabled");

    /* Configure Zigbee platform (radio/host) before starting the Zigbee task */
    {
        esp_zb_platform_config_t zb_config = {
            .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
            .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
        };
        ESP_ERROR_CHECK(esp_zb_platform_config(&zb_config));
    }

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    
    /* Start heartbeat task to show connection status */
    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 3, NULL);
    ESP_LOGI(TAGZB, "Heartbeat task started - will show connection status every 2 seconds");
    
    /* Start button monitor task for factory reset */
    xTaskCreate(button_monitor_task, "button_monitor", 2048, NULL, 3, NULL);
    ESP_LOGI(TAGZB, "Button monitor started - hold boot button for 3 seconds to factory reset");

    // FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // if (log_file == NULL) {
    //     ESP_LOGE(TAG, "Failed to open log file for writing");
    // }
    //FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // Removed unused log file opening and closing to prevent resource leaks.
}