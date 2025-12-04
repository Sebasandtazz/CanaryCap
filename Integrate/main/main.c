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
#include "zcl/esp_zigbee_zcl_command.h"
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
#define ZB_PERMIT_JOIN_DURATION 1200  //seconds
/* Periodic reopen interval (seconds): how often coordinator re-opens network to allow joins */
#define ZB_REOPEN_INTERVAL_SECONDS 30 // 3 minutes

//Definitions for SPI and ADXL345
static const char *TAGSPI = "ADXL345";

#define I2C_MASTER_SCL_IO           11 // SCL on ESP32-C6
#define I2C_MASTER_SDA_IO           10 // SDA on ESP32-C6
#define LED_GPIO1                   19 //CONFIG_LED_GPIO
#define LED_GPIO2                   20 //CONFIG_LED_GPIO

#define BUTTON_GPIO                 23  // Boot button for impact cancel and Zigbee reset
#define SPIFFS_CLEAR_BUTTON_GPIO    8   // Boot button for SPIFFS clear
#define BUZZER_GPIO                 18 //CONFIG_BUZZER_GPIO
#define HEARTBEAT_LED_GPIO          19 //CONFIG_HEARTBEAT_LED_GPIO
#define HEALTH_LED_GPIO            20 //CONFIG_HEALTHY_LED_GPIO
#define STATUS_LED_GPIO            21 //CONFIG_STATUS_LED_GPIO
#define PENDING_LED_GPIO           22 //CONFIG_PENDING_LED_GPIO

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

/* H3LIS331DL High-G Accelerometer definitions */
#define H3LIS331DL_ADDR          0x18
#define H3LIS331DL_WHO_AM_I_REG   0x0F
#define H3LIS331DL_CTRL_REG1    0x20
#define H3LIS331DL_OUT_X_L    0x28
#define H3LIS331DL_OUT_Y_L    0x2A
#define H3LIS331DL_OUT_Z_L    0x2C
#define H3LIS331DL_WHO_AM_I_EXPECTED 0x32
#define H3LIS331DL_CTRL_REG4       0x23

/* H3LIS331DL datasheet values:
    - Full scale: ±100 g
    - Sensitivity: 780 mg / digit (i.e. 0.78 g per LSB)
    - Zero-g offset accuracy: ±1.5 g
    - Acceleration noise density (ODR 50Hz): 50 mg/√Hz (approx)
    These values are used to convert raw counts to g.
*/
#define H3LIS_SENSITIVITY_MG       780.0f
#define H3LIS_LSB_TO_G             (H3LIS_SENSITIVITY_MG / 1000.0f) /* 0.78 g/LSB */
/* ADXL345 configured to ±16g full-resolution */
#define ADXL_MAX_G                  16.0f
/* Hand over when ADXL reaches this fraction of its max (configurable) */
#define ADXL_HANDOVER_RATIO         0.90f

#define THRESH_SEVERE_G             4.0f
#define THRESH_MODERATE_G           2.0f
#define MAX_LOG_FILE_SIZE           (100 * 1024)  // 100 KB max log file size

static TimerHandle_t major_impact_timer;
static bool major_impact_active = false;
static volatile bool major_impact_cancelled_flag = false;
static const char *TAG1 = "spiffs_list";

/* Button press tracking for factory reset */
static volatile uint32_t button_press_time = 0;
static volatile bool button_pressed = false;
static int impact_count = 0;

/* Baseline accelerometer values for adaptive thresholding */
static float baseline_x = 0.0f;
static float baseline_y = 0.0f;
static float baseline_z = 0.0f;
static float baseline_mag = 0.0f;

/* Button hold timer for SPIFFS clear (requires 5 second hold) */
static TimerHandle_t button_hold_timer = NULL;
static bool button_held = false;

//Def for Buzzer
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (18) // Buzzer GPIO (matches BUZZER_GPIO)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (2700) // Resonant frequency - try 2700Hz for max volume

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

/* Global flags for alert cancellation */
static volatile bool severe_alert_active = false;
static volatile bool network_disconnected_alert_active = false;

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

static esp_err_t h3lis_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    uint8_t addr = reg_addr;
    /* Some devices require setting MSB for auto-increment during multi-byte read */
    if (len > 1) addr = reg_addr | 0x80;
    return i2c_master_transmit_receive(dev_handle, &addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t h3lis_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = { reg_addr, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void h3lis_init(i2c_master_dev_handle_t dev_handle)
{
    uint8_t who = 0;
    if (h3lis_register_read(dev_handle, H3LIS331DL_WHO_AM_I_REG, &who, 1) == ESP_OK) {
        ESP_LOGI(TAGSPI, "H3LIS WHO_AM_I = 0x%02X", who);
        if (who != H3LIS331DL_WHO_AM_I_EXPECTED) {
            ESP_LOGW(TAGSPI, "Unexpected H3LIS WHO_AM_I (expected 0x%02X)", H3LIS331DL_WHO_AM_I_EXPECTED);
        }
    } else {
        ESP_LOGW(TAGSPI, "Failed to read H3LIS WHO_AM_I");
    }

    /* Configure H3LIS: enable X/Y/Z, set ODR to a high rate and normal mode.
       0x27: common value to enable axes and set ODR (adjust if needed).
       Also set CTRL_REG4 to select full-scale ±100g if required by the device.
    */
    if (h3lis_register_write_byte(dev_handle, H3LIS331DL_CTRL_REG1, 0x27) != ESP_OK) {
        ESP_LOGW(TAGSPI, "Failed to write H3LIS CTRL_REG1");
    } else {
        ESP_LOGI(TAGSPI, "H3LIS CTRL_REG1 configured");
    }

    /* Set full-scale to ±100g: value below sets FS bits for ±100g on many H3LIS331DL versions.
       If your module requires a different value for ±100g, adjust H3LIS_FS_VAL accordingly. */
    /* H3LIS FS register value for ±100g. This should be set to the value
       specified by the H3LIS331DL datasheet for your chosen FS setting.
       If you want me to set the exact datasheet value I can fetch it and
       update this constant; for now this is a placeholder that you can
       change after confirming with the datasheet. */
     //CTRL_REG4 value provided by user (0x23) — sets FS and related bits per module config 
    const uint8_t H3LIS_CTRL_REG4_FS_100G = 0x23; /* user provided */
    if (h3lis_register_write_byte(dev_handle, H3LIS331DL_CTRL_REG4, H3LIS_CTRL_REG4_FS_100G) != ESP_OK) {
        ESP_LOGW(TAGSPI, "Failed to write H3LIS CTRL_REG4 (FS)");
    } else {
        ESP_LOGI(TAGSPI, "H3LIS CTRL_REG4 (FS) configured (val=0x%02X)", H3LIS_CTRL_REG4_FS_100G);
    }
    ESP_LOGI(TAGSPI, "H3LIS configured: FS=±100g, sensitivity=%.1f mg/LSB (%.3f g/LSB), zero-g offset accuracy=±1.5 g", H3LIS_SENSITIVITY_MG, H3LIS_LSB_TO_G);
}

/* Helper: read ADXL345 and return X/Y/Z in g (Z with gravity offset like rest of code)
   Returns true on success. */
static bool read_adxl_xyz(i2c_master_dev_handle_t dev_handle, float *xg, float *yg, float *zg){
    uint8_t data[6];
    if (adxl345_register_read(dev_handle, ADXL345_DATAX0_REG, data, 6) != ESP_OK) return false;
    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);
    *xg = x_raw * LSB_TO_G;
    *yg = y_raw * LSB_TO_G;
    *zg = z_raw * LSB_TO_G - 1.0f; // same gravity offset used elsewhere
    return true;
}

/* Helper: read H3LIS331DL and return X/Y/Z in g (apply same gravity offset to Z).
   Returns true on success. */
static bool read_h3lis_xyz(i2c_master_dev_handle_t dev_handle, float *xg, float *yg, float *zg)
{
    uint8_t data[6];
     //H3LIS registers OUT_X_L .. OUT_Z_H are contiguous in many modes; read 6 bytes starting at OUT_X_L 
    if (h3lis_register_read(dev_handle, H3LIS331DL_OUT_X_L, data, 6) != ESP_OK) return false;
    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);
    *xg = x_raw * H3LIS_LSB_TO_G;
    *yg = y_raw * H3LIS_LSB_TO_G;
    *zg = z_raw * H3LIS_LSB_TO_G - 1.0f; 
    return true;
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle_adxl, i2c_master_dev_handle_t *dev_handle_h3lis)
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
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle_adxl));

    /* H3LIS support disabled - do not add H3LIS device */
    /* dev_config.device_address = H3LIS331DL_ADDR; */
    /* ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle_h3lis)); */
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

/* Timer callback: clears SPIFFS if button was held for full 5 seconds */
static void button_hold_timer_callback(TimerHandle_t xTimer)
{
    
    if (gpio_get_level(SPIFFS_CLEAR_BUTTON_GPIO) == 0) {
        FILE *f = fopen("/spiffs/i2c_log.txt", "w");
        impact_count = 0;
        gpio_set_level(LED_GPIO2, 1);
        fclose(f);
        ESP_LOGI(TAGSPI, "Log file cleared by 5-second GPIO8 button hold.");
        button_held = false;
    } else {
        ESP_LOGI(TAGSPI, "GPIO8 button released before 5 seconds; clear cancelled.");
        button_held = false;
    }
}

void log_data_to_spiffs(float mag, float x_g, float y_g, float z_g)
{
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

/* Collect baseline accelerometer readings for duration_ms sampling every interval_ms.
   This blocks for the duration (intended to run at startup) and fills baseline globals.
*/
static void collect_baseline(i2c_master_dev_handle_t dev_handle, uint32_t duration_ms, uint32_t interval_ms)
{
    const int max_samples = (duration_ms + interval_ms - 1) / interval_ms;
    if (max_samples <= 0) return;

    float sx = 0.0f, sy = 0.0f, sz = 0.0f, sm = 0.0f;
    int samples = 0;

    ESP_LOGI(TAGSPI, "Collecting baseline for %u ms (%d samples)...", duration_ms, max_samples);
    for (int i = 0; i < max_samples; ++i) {
        float x_g, y_g, z_g;
        if (read_adxl_xyz(dev_handle, &x_g, &y_g, &z_g)) {
            float mag = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
            sx += fabsf(x_g);
            sy += fabsf(y_g);
            sz += fabsf(z_g);
            sm += mag;
            samples++;
        } else {
            ESP_LOGW(TAGSPI, "Baseline sample %d failed", i);
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    if(samples > 0){
        baseline_x = sx / samples;
        baseline_y = sy / samples;
        baseline_z = sz / samples;
        baseline_mag = sm / samples;
        ESP_LOGI(TAGSPI, "Baseline collected: X=%.3fg Y=%.3fg Z=%.3fg MAG=%.3fg (n=%d)", baseline_x, baseline_y, baseline_z, baseline_mag, samples);
    } 
    else {
        ESP_LOGW(TAGSPI, "No baseline samples collected");
    }
}

static const char* infer_impact_type(float ax,float ay,float az,float mag){
    
    if(mag>THRESH_SEVERE_G && az<(-1.0f) && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Fall";
    if(mag>THRESH_MODERATE_G && fabsf(ax)>(5*baseline_x) && fabsf(ay)>(5*baseline_y) && fabsf(az)>(5*baseline_z)) return "Ceiling Collapse";
    if(mag>THRESH_SEVERE_G && az>1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Hard Landing";
    if(mag>THRESH_MODERATE_G && ax>(5*baseline_x)) return "Front Impact";
    if(mag>THRESH_MODERATE_G && ax<(-5*baseline_x)) return "Rear Impact";
    if(mag>THRESH_MODERATE_G && ay>(5*baseline_y)) return "Right Side Impact";
    if(mag>THRESH_MODERATE_G && ay<(-5*baseline_y)) return "Left Side Impact";
    
    if(mag>THRESH_MODERATE_G) return "blunt";
    return "none";
}

/* Forward declaration for severe impact alert task */
static void severe_impact_alert_task(void *param);

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
        if (zb_mesh_is_joined()) {
            ESP_LOGW(TAGZB, "Sending impact alert over Zigbee: type=%s severity=%s mag=%.2fg",
                     zb_cluster_impact_type_to_string(cluster_type),
                     zb_cluster_impact_severity_to_string(severity),
                     mag);
            zb_cluster_send_impact_alert(severity, cluster_type, mag, x_g, y_g, z_g);
        } else {
            ESP_LOGE(TAGZB, "Impact alert NOT sent: device not joined to Zigbee network");
        }
        
        /* Local alerts for severe impacts - cancellable with button */
        if (severity == CANARY_IMPACT_SEVERE) {
            severe_alert_active = true;
            ESP_LOGW(TAGZB, "LOCAL SEVERE IMPACT - Press GPIO23 button to cancel alert");
            xTaskCreate(severe_impact_alert_task, "severe_alert", 2048, NULL, 5, NULL);
        } else if (severity == CANARY_IMPACT_MAJOR) {
            /* Major impact: 3-second local indication */
            int64_t start_time = esp_timer_get_time();
            int64_t duration_us = 3000000;  // 3 seconds
            
            while ((esp_timer_get_time() - start_time) < duration_us) {
                gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
                buzzer_beep_ms(LEDC_FREQUENCY, 150);  // Resonant frequency for max volume
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            gpio_set_level(HEALTH_LED_GPIO, 1);  // Ensure OFF
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

/* GPIO 8 button ISR - SPIFFS clear */
static void IRAM_ATTR spiffs_clear_button_handler(void* arg){
    /* Start the 5-second hold timer for SPIFFS clear */
    uint32_t level = gpio_get_level(SPIFFS_CLEAR_BUTTON_GPIO);
    
    if (level == 0) {
        /* Button pressed */
        if (!button_held && button_hold_timer != NULL) {
            button_held = true;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTimerStartFromISR(button_hold_timer, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    } else {
        /* Button released */
        if (button_held && button_hold_timer != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTimerStopFromISR(button_hold_timer, &xHigherPriorityTaskWoken);
            button_held = false;
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

static void IRAM_ATTR button_handler(void* arg){
    /* ISR: GPIO 23 button - handle both major and severe impact cancellation */
    
    /* Track button press/release for factory reset */
    uint32_t level = gpio_get_level(BUTTON_GPIO);
    
    /* Cancel severe alert on button press */
    if (level == 0 && severe_alert_active) {
        severe_alert_active = false;
        ESP_EARLY_LOGI("ISR", "Severe alert cancelled by button press");
    }
    
    /* Cancel major impact timer */
    if (major_impact_active) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t stopped = xTimerStopFromISR(major_impact_timer, &xHigherPriorityTaskWoken);
        if (stopped == pdPASS) {
            /* Only mark cancelled if the timer was actually stopped */
            major_impact_active = false;
            major_impact_cancelled_flag = true;
        }
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    
    if (level == 0) {
        /* Button pressed (active low) */
        button_pressed = true;
        button_press_time = xTaskGetTickCountFromISR();
    } else {
        /* Button released */
        button_pressed = false;
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

        // ESP_LOGI(TAGADC, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        // ESP_LOGI(TAGADC, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        // ESP_LOGI(TAGADC, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
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
            if(voltage_mv >= 2700)  // 2.7V threshold for gas detection
            {
                /* Send gas alert via cluster manager */
                zb_cluster_send_gas_alert(true, voltage_mv, parsed_data[i].raw_data);
                
                /* Local alert indicators */
                start_led_blink(LED_GPIO2, 200, 200);
                buzzer_beep_ms(LEDC_FREQUENCY, 3000);

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
            // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

            adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t num_parsed_samples = 0;

            esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
            if (parse_ret == ESP_OK) {
                for (int i = 0; i < num_parsed_samples; i++) {
                    if (parsed_data[i].valid) {
                        uint32_t raw = parsed_data[i].raw_data;
                        uint32_t voltage_mv = raw_to_voltage(raw);
                        
                        /* Check for gas detection at 2.7V threshold */
                        if (voltage_mv >= 2700) {
                            ESP_LOGW(TAGADC, "GAS DETECTED! Voltage: %ld mV", voltage_mv);
                            /* Send gas alert over Zigbee */
                            if (zb_mesh_is_joined()) {
                                zb_cluster_send_gas_alert(true, voltage_mv, raw);
                            }
                        }
                        
                        if(i % 32 == 0) { // Log every 32nd valid sample
                        // ESP_LOGI(TAGADC, "ADC%d Ch%d raw=%"PRIu32" mV=%"PRIu32,
                        //         parsed_data[i].unit + 1,
                        //         parsed_data[i].channel,
                        //         raw,
                        //         voltage_mv);
                            } 
                        }
                        else {
                        // ESP_LOGW(TAGADC, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
                        //         parsed_data[i].unit + 1,
                        //         parsed_data[i].channel,
                        //         parsed_data[i].raw_data);
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

// --- Task functions for continuous alerts ---

static void network_disconnect_alert_task(void *param)
{
    ESP_LOGE(TAGZB, "NETWORK DISCONNECTED - Flashing all LEDs until rejoin");
    while (network_disconnected_alert_active) {
        /* Flash all 4 LEDs */
        gpio_set_level(HEARTBEAT_LED_GPIO, 0);  // ON
        gpio_set_level(HEALTH_LED_GPIO, 0);     // ON
        gpio_set_level(STATUS_LED_GPIO, 0);     // ON
        gpio_set_level(PENDING_LED_GPIO, 0);    // ON
        buzzer_beep_ms(3000, 300);
        vTaskDelay(pdMS_TO_TICKS(300));
        
        gpio_set_level(HEARTBEAT_LED_GPIO, 1);  // OFF
        gpio_set_level(HEALTH_LED_GPIO, 1);     // OFF
        gpio_set_level(STATUS_LED_GPIO, 1);     // OFF
        gpio_set_level(PENDING_LED_GPIO, 1);    // OFF
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    /* Cleanup when reconnected */
    gpio_set_level(HEARTBEAT_LED_GPIO, 1);
    gpio_set_level(HEALTH_LED_GPIO, 1);
    gpio_set_level(STATUS_LED_GPIO, 1);
    gpio_set_level(PENDING_LED_GPIO, 1);
    ESP_LOGI(TAGZB, "Network reconnected - stopping disconnect alert");
    vTaskDelete(NULL);
}

static void severe_impact_alert_task(void *param)
{
    while (severe_alert_active) {
        gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
        gpio_set_level(PENDING_LED_GPIO, 0); // ON
        buzzer_beep_ms(LEDC_FREQUENCY, 200);  // Resonant frequency for max volume
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
        gpio_set_level(PENDING_LED_GPIO, 1); // OFF
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    /* Cleanup when cancelled */
    gpio_set_level(HEALTH_LED_GPIO, 1);
    gpio_set_level(PENDING_LED_GPIO, 1);
    ESP_LOGI(TAGZB, "Severe alert cancelled");
    vTaskDelete(NULL);
}

// --- Zigbee mesh network callbacks ---

static void mesh_state_change_callback(zb_mesh_state_t old_state, zb_mesh_state_t new_state)
{
    ESP_LOGW(TAGZB, "*** MESH STATE CHANGED: %d -> %d ***", old_state, new_state);
    
    if (new_state == ZB_MESH_STATE_JOINED) {
        /* Joined network - flash STATUS LED and indicate success */
        ESP_LOGW(TAGZB, "==> DEVICE CONNECTED TO NETWORK <==");
        
        /* Cancel network disconnection alert if it was active */
        if (network_disconnected_alert_active) {
            network_disconnected_alert_active = false;
            ESP_LOGI(TAGZB, "Network reconnected - cancelling disconnect alert");
        }
        
        /* Flash STATUS LED rapidly to indicate connection */
        for (int i = 0; i < 5; i++) {
            gpio_set_level(STATUS_LED_GPIO, 0);  // ON
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 1);  // OFF
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        /* Keep STATUS LED on while connected (solid) */
        gpio_set_level(STATUS_LED_GPIO, 0);  // ON
        
        /* Stop any blinking that was happening */
        stop_led_blink();
        
        /* Success beep */
        buzzer_beep_ms(LEDC_FREQUENCY, 200);
        
    } else if (old_state == ZB_MESH_STATE_JOINED && new_state == ZB_MESH_STATE_DISCONNECTED) {
        /* Lost connection - flash ALL LEDs and play buzzer continuously until rejoin */
        ESP_LOGE(TAGZB, "==> DEVICE DISCONNECTED FROM NETWORK <==");
        
        /* Activate continuous network disconnection alert */
        network_disconnected_alert_active = true;
        
        /* Spawn task for continuous disconnect alert */
        xTaskCreate(network_disconnect_alert_task, "disconnect_alert", 2048, NULL, 5, NULL);
        
        /* Start blinking STATUS LED */
        start_led_blink(STATUS_LED_GPIO, 500, 500);
        
    } else if (new_state == ZB_MESH_STATE_FORMING) {
        ESP_LOGI(TAGZB, "Forming network...");
        /* Flash STATUS LED while forming */
        start_led_blink(STATUS_LED_GPIO, 200, 200);
    } else if (new_state == ZB_MESH_STATE_JOINING) {
        ESP_LOGI(TAGZB, "Joining network...");
        /* Flash STATUS LED while joining */
        start_led_blink(STATUS_LED_GPIO, 200, 200);
    }
}

static void mesh_device_event_callback(bool joined, uint16_t device_addr)
{
    if (joined) {
        ESP_LOGW(TAGZB, "*** NEW DEVICE JOINED NETWORK: 0x%04x ***", device_addr);
        
        /* Flash HEALTH LED to indicate new device */
        for (int i = 0; i < 3; i++) {
            gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        
        /* Quick beep */
        buzzer_beep_ms(LEDC_FREQUENCY, 150);
        
    } else {
        ESP_LOGE(TAGZB, "*** DEVICE LEFT NETWORK: 0x%04x ***", device_addr);
        
        /* Flash HEALTH LED slowly to indicate device left */
        for (int i = 0; i < 2; i++) {
            gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        /* Warning beep */
        buzzer_beep_ms(LEDC_FREQUENCY, 300);
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
        buzzer_beep_ms(LEDC_FREQUENCY, 200);
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
    ESP_LOGE(TAGZB, "==== RECEIVED IMPACT ALERT ==== SRC=0x%04x TYPE=%s SEVERITY=%s MAG=%.2fg ====",
             src_addr,
             zb_cluster_impact_type_to_string(msg->type),
             zb_cluster_impact_severity_to_string(msg->severity),
             magnitude);
    
    /* Visual/audio alert based on severity - remote alerts are NOT cancellable */
    if (msg->severity == CANARY_IMPACT_SEVERE) {
        /* SEVERE: Continuous flash for remote indication (runs in callback, not cancellable) */
        ESP_LOGE(TAGZB, "REMOTE SEVERE IMPACT from 0x%04x - Showing continuous alert", src_addr);
        
        /* Show continuous alert for 10 seconds, then stop automatically */
        int64_t start_time = esp_timer_get_time();
        int64_t duration_us = 10000000;  // 10 seconds
        
        while ((esp_timer_get_time() - start_time) < duration_us) {
            gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
            gpio_set_level(PENDING_LED_GPIO, 0); // ON
            buzzer_beep_ms(LEDC_FREQUENCY, 200);  // Resonant frequency for max volume
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
            gpio_set_level(PENDING_LED_GPIO, 1); // OFF
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        gpio_set_level(HEALTH_LED_GPIO, 1);
        gpio_set_level(PENDING_LED_GPIO, 1);
        
    } else if (msg->severity == CANARY_IMPACT_MAJOR) {
        /* MAJOR: Flash HEALTH LED for 3 seconds with full volume buzzer */
        int64_t start_time = esp_timer_get_time();
        int64_t duration_us = 3000000;  // 3 seconds
        
        while ((esp_timer_get_time() - start_time) < duration_us) {
            gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
            buzzer_beep_ms(LEDC_FREQUENCY, 150);  // Resonant frequency for max volume
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        gpio_set_level(HEALTH_LED_GPIO, 1);  // Ensure OFF
    }
}

static void gas_alert_received_callback(const canary_gas_alert_msg_t *msg, uint16_t src_addr)
{
    ESP_LOGE(TAGZB, "==== RECEIVED GAS ALERT ==== SRC=0x%04x DETECTED=%d VOLTAGE=%dmV ====",
             src_addr, msg->detected, msg->voltage_mv);
    
    /* Visual/audio alert for remote gas detection */
    if (msg->detected) {
        gpio_set_level(LED_GPIO2, 0); // Turn on LED
        buzzer_beep_ms(LEDC_FREQUENCY, 1500);
        // brief pulse pattern so it's noticeable
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO2, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO2, 0);
    }
}

/* Zigbee core action handler for custom cluster commands */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    
    /* Log ALL action handler callbacks to diagnose receive path */
    ESP_LOGE(TAGZB, "!!!! [ACTION_HANDLER] CALLBACK FIRED - ID: 0x%x !!!!", callback_id);
    
    /* Loud banner for any custom cluster traffic */
    if (callback_id == ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID || 
        callback_id == ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_RESP_CB_ID) {
        ESP_LOGE(TAGZB, "================ CUSTOM CLUSTER CALLBACK ================ id=0x%x", callback_id);
    }
    
    switch (callback_id) {
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_RESP_CB_ID:
        {
            ESP_LOGE(TAGZB, "[ACTION] CUSTOM CLUSTER COMMAND RECEIVED!");
            esp_zb_zcl_custom_cluster_command_message_t *msg = (esp_zb_zcl_custom_cluster_command_message_t *)message;
            if (msg && msg->data.value) {
                uint16_t src_addr = msg->info.src_address.u.short_addr;
                ESP_LOGE(TAGZB, "[ACTION] SRC=0x%04x CLUSTER=0x%04x SIZE=%d",
                         src_addr, msg->info.cluster, msg->data.size);
                
                if (msg->info.cluster == ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT) {
                    canary_impact_alert_msg_t *alert = (canary_impact_alert_msg_t*)msg->data.value;
                    float mag = alert->magnitude / 1000.0f;
                    float x = alert->x_axis / 1000.0f;
                    float y = alert->y_axis / 1000.0f;
                    float z = alert->z_axis / 1000.0f;
                    
                    ESP_LOGE(TAGZB, "==== REMOTE IMPACT ALERT ==== SRC=0x%04x TYPE=%s SEVERITY=%s MAG=%.2fg XYZ=(%.2f,%.2f,%.2f) ====",
                             src_addr,
                             zb_cluster_impact_type_to_string((canary_impact_type_t)alert->type),
                             zb_cluster_impact_severity_to_string((canary_impact_severity_t)alert->severity),
                             mag, x, y, z);
                    
                    /* Invoke the registered callback */
                    impact_alert_received_callback(alert, src_addr);
                    
                    /* Flash HEALTH LED to indicate remote impact */
                    for (int i = 0; i < 5; i++) {
                        gpio_set_level(HEALTH_LED_GPIO, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(HEALTH_LED_GPIO, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    buzzer_beep_ms(LEDC_FREQUENCY, 300);
                    
                } else if (msg->info.cluster == ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT) {
                    canary_gas_alert_msg_t *alert = (canary_gas_alert_msg_t*)msg->data.value;
                    
                    ESP_LOGE(TAGZB, "==== REMOTE GAS ALERT ==== SRC=0x%04x DETECTED=%d VOLTAGE=%dmV RAW=%d ====",
                             src_addr,
                             alert->detected,
                             alert->voltage_mv,
                             alert->raw_value);
                    
                    /* Invoke the registered callback */
                    gas_alert_received_callback(alert, src_addr);
                    
                    /* Flash PENDING LED for gas alert */
                    for (int i = 0; i < 5; i++) {
                        gpio_set_level(PENDING_LED_GPIO, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(PENDING_LED_GPIO, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    buzzer_beep_ms(LEDC_FREQUENCY, 400);
                }
            }
        }
        break;
    default:
        break;
    }
    return ret;
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
    
    /* Add custom Canary clusters to the endpoint for impact/gas alerts */
    esp_zb_cluster_list_t *cluster_list = esp_zb_ep_list_get_ep(ep_list, HA_ONOFF_SWITCH_ENDPOINT);
    if (cluster_list) {
        /* Create custom clusters with BOTH client (send) and server (receive) roles */
        
        /* Impact alerts - client side for sending */
        esp_zb_attribute_list_t *impact_cluster_client = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, impact_cluster_client, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        
        /* Impact alerts - server side for receiving */
        esp_zb_attribute_list_t *impact_cluster_server = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, impact_cluster_server, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        
        /* Gas alerts - client side for sending */
        esp_zb_attribute_list_t *gas_cluster_client = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, gas_cluster_client, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        
        /* Gas alerts - server side for receiving */
        esp_zb_attribute_list_t *gas_cluster_server = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, gas_cluster_server, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        
        /* Network status - client side for sending */
        esp_zb_attribute_list_t *network_cluster_client = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, network_cluster_client, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        
        /* Network status - server side for receiving */
        esp_zb_attribute_list_t *network_cluster_server = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, network_cluster_server, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        
        ESP_LOGE(TAGZB, "!!! CUSTOM CANARY CLUSTERS REGISTERED !!! endpoint=%d, impact=0x%04x, gas=0x%04x",
                 HA_ONOFF_SWITCH_ENDPOINT, ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT, ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT);
    } else {
        ESP_LOGE(TAGZB, "Failed to get cluster list for endpoint");
    }
    
    esp_zb_device_register(ep_list);
    /* Register core action handler to receive custom cluster commands */
    esp_zb_core_action_handler_register(zb_action_handler);

    /* Note: esp_zb_zcl_custom_cluster_handlers_update is intended for attribute
       validation/write callbacks and standard clusters. Our custom clusters
       use the core action handler path for incoming commands. */

    esp_zb_set_primary_network_channel_set(ZB_MESH_PRIMARY_CHANNEL_MASK);
    
    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    /* Run main Zigbee stack loop - this handles all events */
    esp_zb_stack_main_loop();

}

/* Zigbee app signal handler - delegates to mesh network handler */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    /* Log ALL incoming signals to diagnose if frames are reaching us */
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    uint8_t signal_type = *p_sg_p;
    
    /* Verbose logging for any signal activity */
    if (signal_type != ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS && 
        signal_type != ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP) {
        ESP_LOGE(TAGZB, "[SIGNAL_HANDLER] Signal Type: 0x%x, Status: %s", 
                 signal_type, esp_err_to_name(err_status));
    }
    
    /* Forward all signals to mesh network handler */
    zb_mesh_handle_signal(signal_struct);
}

/* Heartbeat task to show connection status */
static void button_monitor_task(void *pvParameters)
{
    const TickType_t hold_time_ticks = pdMS_TO_TICKS(5000);  // 5 seconds for factory reset
    bool factory_reset_triggered = false;
    bool short_press_handled = false;
    uint32_t last_debug_tick = 0;
    
    ESP_LOGI(TAGZB, "Button monitor task started - GPIO %d", BUTTON_GPIO);
    ESP_LOGI(TAGZB, "GPIO23: Single press cancels severe alert, hold 5 sec for factory reset");
    ESP_LOGI(TAGZB, "GPIO8: Hold 5 sec to clear SPIFFS logs");
    
    while (1) {
        /* Get button level */
        uint32_t level = gpio_get_level(BUTTON_GPIO);
        uint32_t now = xTaskGetTickCount();
        
        /* Debug: Print button state every 2 seconds */
        if ((now - last_debug_tick) >= pdMS_TO_TICKS(2000)) {
            //ESP_LOGI(TAGZB, "[DEBUG] Button GPIO %d level: %lu, pressed: %d", BUTTON_GPIO, level, button_pressed);
            last_debug_tick = now;
        }
        
        /* Check for button press to cancel severe alert */
        if (!button_pressed && !short_press_handled) {
            if (level == 0 && severe_alert_active) {  // Button just pressed and severe alert is active
                severe_alert_active = false;
                short_press_handled = true;
                ESP_LOGW(TAGZB, "GPIO23 button pressed - SEVERE ALERT CANCELLED");
                vTaskDelay(pdMS_TO_TICKS(500));  // Debounce
            }
        }
        
        /* Reset short press flag when button is released */
        if (!button_pressed) {
            short_press_handled = false;
        }
        
        if (button_pressed && !factory_reset_triggered) {
            uint32_t hold_duration = xTaskGetTickCount() - button_press_time;
            
            ESP_LOGI(TAGZB, "Button held for %lu ms", (hold_duration * 1000) / configTICK_RATE_HZ);
            
            if (hold_duration >= hold_time_ticks) {
                /* Button held for 5 seconds - trigger factory reset */
                factory_reset_triggered = true;
                
                ESP_LOGW(TAGZB, "=== FACTORY RESET TRIGGERED ===");
                ESP_LOGW(TAGZB, "GPIO23 held for 5 seconds - erasing Zigbee network config...");
                
                /* Flash all LEDs rapidly to indicate factory reset */
                for (int i = 0; i < 10; i++) {
                    gpio_set_level(STATUS_LED_GPIO, 0);  // ON
                    gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
                    gpio_set_level(HEARTBEAT_LED_GPIO, 0);  // ON
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_set_level(STATUS_LED_GPIO, 1);  // OFF
                    gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
                    gpio_set_level(HEARTBEAT_LED_GPIO, 1);  // OFF
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                /* Perform factory reset */
                zb_mesh_factory_reset();
                
                ESP_LOGW(TAGZB, "Factory reset complete! Device will restart...");
                gpio_set_level(STATUS_LED_GPIO, 0);  // Keep STATUS LED on before restart
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
    int beep_counter = 0;
    while (1) {
        bool is_joined = zb_mesh_is_joined();
        uint16_t short_addr = esp_zb_get_short_address();
        
        if (is_joined && short_addr != 0xFFFF) {
            /* Connected - blink HEARTBEAT LED every 2 seconds */
            // ESP_LOGI(TAGZB, "[HEARTBEAT] Connected to network - Short Addr: 0x%04x", short_addr);
            
            /* Quick pulse on HEARTBEAT LED to show we're alive */
            gpio_set_level(HEARTBEAT_LED_GPIO, 0);  // ON
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(HEARTBEAT_LED_GPIO, 1);  // OFF
            
            pulse_count++;
            if (pulse_count % 5 == 0) {  // Every 10 seconds
                // ESP_LOGW(TAGZB, "========== NETWORK STATUS: CONNECTED (0x%04x) ==========", short_addr);
            }
            
            vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second heartbeat
            
        } else {
            /* Not connected - STATUS LED should be blinking (handled by mesh callback) */
            /* Sound buzzer periodically to alert disconnection */
            // ESP_LOGW(TAGZB, "[HEARTBEAT] Not connected - Short Addr: 0x%04x (searching...)", short_addr);
            
            beep_counter++;
            if (beep_counter >= 3) {  // Beep every 3 seconds when disconnected
                buzzer_beep_ms(2000, 200);
                beep_counter = 0;
            }
            
            pulse_count = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second when disconnected
        }
    }
}


static void i2c_task(void *arg)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)arg;

    while (1) {
        // Read ADXL345 using helper function
        float x_g = 0.0f, y_g = 0.0f, z_g = 0.0f;
        if (!read_adxl_xyz(dev_handle, &x_g, &y_g, &z_g)) {
            ESP_LOGW(TAGSPI, "ADXL read failed; skipping sample");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float mag = sqrtf(x_g*x_g + y_g*y_g + z_g*z_g);

        /* If ADXL indicates a high-G event above 1.5g, query the H3LIS
           for a more accurate high-G measurement and use that for
           classification when available. */
        /* Hand over to H3LIS when ADXL reaches its maximum measurable g (ADXL_MAX_G).
           This ensures we rely on the high-G sensor only when ADXL is beyond its range. */
        /* Hand over to H3LIS when ADXL reaches 90% of its maximum measurable g.
           This gives an early handover to the high-g sensor. */
       
        /*if (mag >= (ADXL_HANDOVER_RATIO * ADXL_MAX_G)) {
            ESP_LOGI(TAGSPI, "H3LIS support disabled at compile-time; using ADXL only (mag=%.3f)", mag);
        }
        */

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

    uint8_t devid = 0;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle_adxl;
    i2c_master_dev_handle_t dev_handle_h3lis;
    
    // Initialize I2C and attach ADXL345 and H3LIS331DL
    i2c_master_init(&bus_handle, &dev_handle_adxl, &dev_handle_h3lis);
    ESP_LOGI(TAGSPI, "I2C initialized successfully");

    // Verify ADXL345 identity
    esp_err_t ret = adxl345_register_read(dev_handle_adxl, ADXL345_DEVID_REG, &devid, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAGSPI, "ADXL345 DEVID = 0x%02X", devid);
        if (devid != ADXL345_DEVID_EXPECTED) {
            ESP_LOGE(TAGSPI, "ADXL345 not detected! Expected 0xE5.");
            // continue without accelerometer
        }
    } else {
        ESP_LOGE(TAGSPI, "Failed to communicate with ADXL345 at 0x%02X", ADXL345_ADDR);
    }

    // Enable measurement mode
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle_adxl, ADXL345_POWER_CTL_REG, 0x08));

    // Set data format: Full resolution, ±16g range (0x0B for full-res ±16g)
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle_adxl, ADXL345_DATA_FORMAT_REG, 0x0B));

    init_led(LED_GPIO1);
    init_led(LED_GPIO2);
    init_led(HEARTBEAT_LED_GPIO);
    init_led(HEALTH_LED_GPIO);
    init_led(STATUS_LED_GPIO);
    init_led(PENDING_LED_GPIO);

    //gpio_set_level(HEALTH_LED_GPIO, 0);  // OFF
    //gpio_set_level(PENDING_LED_GPIO, 0);  // OFF
    
    // Collect baseline accelerometer values (5 seconds at 100ms intervals)
    collect_baseline(dev_handle_adxl, 5000, 100);
    
    /* H3LIS support disabled: skip h3lis_init(dev_handle_h3lis); */
    buzzer_init();
    
    ESP_LOGI(TAGZB, "Allocated LEDs initialized: HEARTBEAT=%d, HEALTH=%d, STATUS=%d, PENDING=%d",
             HEARTBEAT_LED_GPIO, HEALTH_LED_GPIO, STATUS_LED_GPIO, PENDING_LED_GPIO);

    // Configure GPIO 23 button input (severe alert cancel & factory reset)
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // enable pull-up
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE, // both edges for press/release detection
    };
    gpio_config(&io_conf);

    // Configure GPIO 8 button input (SPIFFS clear)
    gpio_config_t io_conf_gpio8 = {
        .pin_bit_mask = 1ULL << SPIFFS_CLEAR_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // enable pull-up
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE, // both edges for press/release detection
    };
    gpio_config(&io_conf_gpio8);

    // Install ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_handler, NULL);
    gpio_isr_handler_add(SPIFFS_CLEAR_BUTTON_GPIO, spiffs_clear_button_handler, NULL);

    // Create major impact timer (10 sec)
    major_impact_timer = xTimerCreate("MajorImpactTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, major_impact_timer_callback);
    
    // Create button hold timer (5 sec for SPIFFS clear)
    button_hold_timer = xTimerCreate("ButtonHoldTimer", pdMS_TO_TICKS(5000), pdFALSE, NULL, button_hold_timer_callback);

    // Create I2C task for accelerometer monitoring
    xTaskCreate(i2c_task, "i2c_task", 4096, dev_handle_adxl, 5, &i2c_task_handle);

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
    ESP_LOGI(TAGZB, "Button monitor started - hold boot button for 10 seconds to factory reset");

    // FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // if (log_file == NULL) {
    //     ESP_LOGE(TAG, "Failed to open log file for writing");
    // }
    //FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // Removed unused log file opening and closing to prevent resource leaks.
}