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
#include "zb_device_registry.h"
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
/* Keep network open for initial joins, then close to save resources */
#define ZB_PERMIT_JOIN_DURATION 240  // 4 minutes - sufficient for initial device joins
/* NOTE: Network does NOT need to stay open for routing to work!
 * RxOnWhenIdle=true is sufficient for message routing. */

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

/* Double-press detection for soft leave on SPIFFS button */
static volatile uint32_t spiffs_button_last_press_time = 0;
static volatile uint8_t spiffs_button_press_count = 0;
#define DOUBLE_PRESS_TIMEOUT_MS 500  // 500ms window for double press

//Def for Buzzer
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (18) // Buzzer GPIO (matches BUZZER_GPIO)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Resonant frequency

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

/* Gas detection state tracking */
static volatile bool gas_detected = false;
static volatile bool gas_alert_active = false;
static int64_t gas_first_detected_time = 0;   // When gas was first detected in current episode
static int64_t gas_last_clear_time = 0;        // When gas was last clear (below threshold)
static int64_t gas_last_zigbee_alert_time = 0; // When we last sent a Zigbee alert
static uint32_t gas_current_voltage_mv = 0;    // Latest voltage reading
static uint32_t gas_current_raw = 0;           // Latest raw ADC reading
#define GAS_THRESHOLD_MV 2700                   // 2.7V threshold for MQ-7 gas detection
#define GAS_CLEAR_DURATION_MS (3 * 60 * 1000)  // 3 minutes clear before allowing new alert
#define GAS_CHECK_INTERVAL_MS (60 * 1000)       // 60 seconds between checks when in gas
static TaskHandle_t gas_alert_task_handle = NULL;

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

/* Forward declarations for alert tasks */
static void severe_impact_alert_task(void *param);
static void gas_alert_task(void *param);

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
        /* Local alerts for severe impacts - cancellable with button */
        if (severity == CANARY_IMPACT_SEVERE) {
            severe_alert_active = true;
            ESP_LOGW(TAGZB, "LOCAL SEVERE IMPACT - Press GPIO23 button within 10s to cancel");
            
            /* Allocate impact data to pass to task */
            typedef struct {
                canary_impact_severity_t severity;
                canary_impact_type_t type;
                float mag;
                float x_g;
                float y_g;
                float z_g;
            } impact_data_t;
            
            impact_data_t *data = (impact_data_t *)malloc(sizeof(impact_data_t));
            if (data) {
                data->severity = severity;
                data->type = cluster_type;
                data->mag = mag;
                data->x_g = x_g;
                data->y_g = y_g;
                data->z_g = z_g;
                /* Don't send Zigbee message yet - let the severe_impact_alert_task handle it after timer */
                xTaskCreate(severe_impact_alert_task, "severe_alert", 4096, data, 7, NULL);
            }
        } else if (severity == CANARY_IMPACT_MAJOR) {
            /* MAJOR impacts send immediately */
            if (zb_mesh_is_joined()) {
                ESP_LOGW(TAGZB, "Sending MAJOR impact alert over Zigbee: type=%s mag=%.2fg",
                         zb_cluster_impact_type_to_string(cluster_type), mag);
                zb_cluster_send_impact_alert(severity, cluster_type, mag, x_g, y_g, z_g);
            } else {
                ESP_LOGE(TAGZB, "Impact alert NOT sent: device not joined to Zigbee network");
            }
        }
        
        if (severity == CANARY_IMPACT_MAJOR) {
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

/* GPIO 8 button ISR - SPIFFS clear (hold 5s) or soft leave (double press) */
static void IRAM_ATTR spiffs_clear_button_handler(void* arg){
    uint32_t level = gpio_get_level(SPIFFS_CLEAR_BUTTON_GPIO);
    uint32_t now = xTaskGetTickCountFromISR();
    
    if (level == 0) {
        /* Button pressed */
        uint32_t time_since_last = (now - spiffs_button_last_press_time) * 1000 / configTICK_RATE_HZ;
        
        if (time_since_last < DOUBLE_PRESS_TIMEOUT_MS) {
            /* Double press detected - trigger soft leave */
            spiffs_button_press_count = 2;
            ESP_EARLY_LOGI("BUTTON", "Double press detected - triggering soft leave");
        } else {
            /* First press - start counting */
            spiffs_button_press_count = 1;
        }
        
        spiffs_button_last_press_time = now;
        
        /* Start the 5-second hold timer for SPIFFS clear */
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

    ESP_LOGI(TAGADC, "ADC task started - continuous gas monitoring active");

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
                        
                        /* Update current readings */
                        gas_current_voltage_mv = voltage_mv;
                        gas_current_raw = raw;
                        
                        int64_t now = esp_timer_get_time();
                        
                        /* Check for gas detection at 2.7V threshold */
                        if (voltage_mv >= GAS_THRESHOLD_MV) {
                            /* Gas detected */
                            if (!gas_detected) {
                                /* FIRST detection in this episode */
                                gas_detected = true;
                                gas_first_detected_time = now;
                                
                                /* Determine if we should send Zigbee alert */
                                bool should_alert = false;
                                if (gas_last_zigbee_alert_time == 0) {
                                    /* First ever detection */
                                    should_alert = true;
                                } else {
                                    /* Check if we've been clear for 3 minutes */
                                    int64_t time_since_last_clear_ms = (now - gas_last_clear_time) / 1000LL;
                                    if (time_since_last_clear_ms >= GAS_CLEAR_DURATION_MS) {
                                        should_alert = true;
                                    }
                                }
                                
                                if (should_alert) {
                                    ESP_LOGE(TAGADC, "*** GAS DETECTED - SENDING ZIGBEE ALERT *** Voltage: %ld mV", voltage_mv);
                                    /* Send ONE Zigbee alert for this episode */
                                    if (zb_mesh_is_joined()) {
                                        zb_cluster_send_gas_alert(true, voltage_mv, raw);
                                        gas_last_zigbee_alert_time = now;
                                    }
                                } else {
                                    ESP_LOGW(TAGADC, "Gas detected but not sending Zigbee (within 3min cooldown) - Voltage: %ld mV", voltage_mv);
                                }
                                
                                /* Start continuous local alert (buzzer + LED) */
                                if (!gas_alert_active && gas_alert_task_handle == NULL) {
                                    gas_alert_active = true;
                                    xTaskCreate(gas_alert_task, "gas_alert_task", 4096, NULL, 5, &gas_alert_task_handle);
                                }
                            }
                            /* else: already detected, local alerts continue in separate task */
                        } else {
                            /* Below threshold - gas clear */
                            if (gas_detected) {
                                /* Transitioning from detected to clear */
                                gas_detected = false;
                                gas_alert_active = false;  // Signal alert task to stop
                                gas_last_clear_time = now;
                                int64_t duration_ms = (now - gas_first_detected_time) / 1000LL;
                                ESP_LOGI(TAGADC, "Gas cleared - was detected for %lld ms", duration_ms);
                                
                                /* Turn LEDs back on (solid) after gas clears */
                                gpio_set_level(STATUS_LED_GPIO, 0);  // ON
                                gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
                            }
                        }
                        
                        if(i % 32 == 0) { // Log every 32nd valid sample
                            // ESP_LOGI(TAGADC, "ADC%d Ch%d raw=%"PRIu32" mV=%"PRIu32,
                            //         parsed_data[i].unit + 1,
                            //         parsed_data[i].channel,
                            //         raw,
                            //         voltage_mv);
                        } else {
                            // ESP_LOGW(TAGADC, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
                            //         parsed_data[i].unit + 1,
                            //         parsed_data[i].channel,
                            //         parsed_data[i].raw_data);
                        }
                    }
                }
                log_adc_data_to_spiffs(parsed_data, num_parsed_samples);
            } else {
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

/* Gas alert task - handles continuous local alerts (buzzer + LED)
 * while checking every 60s if still in gas environment */
static void gas_alert_task(void *param)
{
    ESP_LOGE(TAGADC, "GAS ALERT TASK STARTED - Alternating STATUS/HEALTH LEDs with indefinite buzzer");
    int64_t last_check_time = esp_timer_get_time();
    bool status_led_on = true;  // Track which LED is currently on
    
    while (gas_alert_active) {
        /* Alternate between STATUS and HEALTH LEDs with continuous buzzer */
        if (status_led_on) {
            gpio_set_level(STATUS_LED_GPIO, 0);   // STATUS ON
            gpio_set_level(HEALTH_LED_GPIO, 1);   // HEALTH OFF
        } else {
            gpio_set_level(STATUS_LED_GPIO, 1);   // STATUS OFF
            gpio_set_level(HEALTH_LED_GPIO, 0);   // HEALTH ON
        }
        status_led_on = !status_led_on;  // Toggle for next iteration
        
        buzzer_beep_ms(LEDC_FREQUENCY, 300);  // Continuous beeping
        vTaskDelay(pdMS_TO_TICKS(300));
        
        /* Every 60 seconds, check if we should send a Zigbee update */
        int64_t now = esp_timer_get_time();
        if ((now - last_check_time) >= (GAS_CHECK_INTERVAL_MS * 1000LL)) {
            last_check_time = now;
            if (gas_detected) {
                ESP_LOGW(TAGADC, "Gas still detected after 60s check - voltage: %ld mV", gas_current_voltage_mv);
                /* Just log locally, don't flood Zigbee network */
            } else {
                ESP_LOGI(TAGADC, "Gas cleared during periodic check");
                break;  // Exit alert loop
            }
        }
    }
    
    /* Clean up when alert stops - restore LEDs to normal state (both ON if connected) */
    if (zb_mesh_is_joined()) {
        gpio_set_level(HEALTH_LED_GPIO, 0);  // ON (normal state when connected)
        gpio_set_level(STATUS_LED_GPIO, 0);  // ON (normal state when connected)
    } else {
        gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
        gpio_set_level(STATUS_LED_GPIO, 1);  // OFF
    }
    ESP_LOGI(TAGADC, "Gas alert task ending");
    gas_alert_task_handle = NULL;
    vTaskDelete(NULL);
}

static void network_disconnect_alert_task(void *param)
{
    ESP_LOGE(TAGZB, "NETWORK DISCONNECTED/DEVICE LEFT - Flashing PENDING+HEALTH+STATUS until reconnection");
    while (network_disconnected_alert_active) {
        /* Flash PENDING, HEALTH, and STATUS LEDs with constant buzzer */
        gpio_set_level(HEALTH_LED_GPIO, 0);     // ON
        gpio_set_level(STATUS_LED_GPIO, 0);     // ON
        gpio_set_level(PENDING_LED_GPIO, 0);    // ON
        buzzer_beep_ms(LEDC_FREQUENCY, 300);
        vTaskDelay(pdMS_TO_TICKS(300));
        
        gpio_set_level(HEALTH_LED_GPIO, 1);     // OFF
        gpio_set_level(STATUS_LED_GPIO, 1);     // OFF
        gpio_set_level(PENDING_LED_GPIO, 1);    // OFF
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    /* Cleanup when reconnected */
    gpio_set_level(HEALTH_LED_GPIO, 1);
    gpio_set_level(STATUS_LED_GPIO, 1);
    gpio_set_level(PENDING_LED_GPIO, 1);
    ESP_LOGI(TAGZB, "Network reconnected - stopping disconnect alert");
    vTaskDelete(NULL);
}

static void severe_impact_alert_task(void *param)
{
    /* param contains impact data: severity, type, mag, x, y, z */
    typedef struct {
        canary_impact_severity_t severity;
        canary_impact_type_t type;
        float mag;
        float x_g;
        float y_g;
        float z_g;
    } impact_data_t;
    
    impact_data_t *data = (impact_data_t *)param;
    
    /* Alert for 10 seconds with button cancel option */
    int64_t start_time = esp_timer_get_time();
    int64_t duration_us = 10000000LL;  // 10 seconds
    
    while (severe_alert_active && (esp_timer_get_time() - start_time) < duration_us) {
        gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
        gpio_set_level(PENDING_LED_GPIO, 0); // ON
        buzzer_beep_ms(LEDC_FREQUENCY, 200);  // Resonant frequency for max volume
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
        gpio_set_level(PENDING_LED_GPIO, 1); // OFF
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    /* Cleanup LEDs */
    gpio_set_level(HEALTH_LED_GPIO, 1);
    gpio_set_level(PENDING_LED_GPIO, 1);
    
    /* If still active after 10s, send Zigbee alert */
    if (severe_alert_active) {
        ESP_LOGW(TAGZB, "Sending SEVERE impact alert over Zigbee (not cancelled): type=%s mag=%.2fg",
                 zb_cluster_impact_type_to_string(data->type), data->mag);
        if (zb_mesh_is_joined()) {
            zb_cluster_send_impact_alert(data->severity, data->type, data->mag, 
                                        data->x_g, data->y_g, data->z_g);
        }
        severe_alert_active = false;
    } else {
        ESP_LOGI(TAGZB, "Severe alert cancelled - NOT sending Zigbee message");
    }
    
    /* Free allocated impact data */
    if (data) {
        free(data);
    }
    
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
        
        /* Also turn on HEALTH LED to indicate normal operation */
        gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
        
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
        xTaskCreate(network_disconnect_alert_task, "disconnect_alert", 2048, NULL, 7, NULL);
        
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
        
        /* Activate continuous device leave alert - flash PENDING+HEALTH+STATUS with constant buzzer */
        network_disconnected_alert_active = true;
        xTaskCreate(network_disconnect_alert_task, "device_leave_alert", 2048, NULL, 7, NULL);
    }
}

static void heartbeat_received_callback(const canary_heartbeat_msg_t *msg, uint16_t src_addr)
{
    ESP_LOGD(TAGZB, "Heartbeat from 0x%04x: seq=%lu, uptime=%lu sec, battery=%u%%",
             src_addr, msg->seq_num, msg->uptime_sec, msg->battery_level);
    
    /* Update device registry with heartbeat timestamp */
    zb_registry_update_heartbeat(src_addr, msg->seq_num);
    
    /* Flash HEARTBEAT LED when heartbeat is received from network */
    gpio_set_level(HEARTBEAT_LED_GPIO, 0);  // ON
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(HEARTBEAT_LED_GPIO, 1);  // OFF
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
    
    /* Additional logging for EVERY callback type to catch anything */
    ESP_LOGE(TAGZB, "[ACTION_HANDLER] Type names: REQ=0x%x RESP=0x%x REPORT=0x%x",
             ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID,
             ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_RESP_CB_ID,
             ESP_ZB_CORE_REPORT_ATTR_CB_ID);
    
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        {
            /* Handle attribute reports - some devices send custom clusters this way */
            ESP_LOGW(TAGZB, "[ACTION] ATTRIBUTE REPORT RECEIVED - checking if it's our custom cluster");
            esp_zb_zcl_report_attr_message_t *report = (esp_zb_zcl_report_attr_message_t *)message;
            if (report) {
                uint16_t src_addr = report->src_address.u.short_addr;
                uint16_t cluster_id = report->cluster;
                ESP_LOGW(TAGZB, "[ACTION] REPORT: SRC=0x%04x CLUSTER=0x%04x", src_addr, cluster_id);
                
                /* For now, just log it - we'll process through the normal custom cluster path */
            }
        }
        break;
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_RESP_CB_ID:
        {
            ESP_LOGE(TAGZB, "[ACTION] CUSTOM CLUSTER COMMAND RECEIVED!");
            esp_zb_zcl_custom_cluster_command_message_t *msg = (esp_zb_zcl_custom_cluster_command_message_t *)message;
            if (msg && msg->data.value) {
                uint16_t src_addr = msg->info.src_address.u.short_addr;
                ESP_LOGE(TAGZB, "[ACTION] SRC=0x%04x CLUSTER=0x%04x SIZE=%d",
                         src_addr, msg->info.cluster, msg->data.size);
                
                /* Auto-add source device to registry if not already present */
                zb_device_info_t check_device;
                if (src_addr != esp_zb_get_short_address() && src_addr != 0xFFFF && src_addr != 0xFFFE) {
                    if (zb_registry_get_device(src_addr, &check_device) != ESP_OK) {
                        ESP_LOGW(TAGZB, "Source device 0x%04x not in registry - adding it now", src_addr);
                        esp_err_t add_result = zb_registry_add_device(src_addr, msg->info.src_endpoint);
                        if (add_result == ESP_OK) {
                            uint8_t total_devices = zb_registry_get_count();
                            ESP_LOGI(TAGZB, "✓ Added device 0x%04x (endpoint=%d) - registry now has %d devices", 
                                     src_addr, msg->info.src_endpoint, total_devices);
                        } else {
                            ESP_LOGE(TAGZB, "✗ Failed to add device 0x%04x to registry: %s", 
                                     src_addr, esp_err_to_name(add_result));
                        }
                    } else {
                        ESP_LOGD(TAGZB, "Device 0x%04x already in registry", src_addr);
                    }
                }
                
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
                    
                } else if (msg->info.cluster == ZB_ZCL_CLUSTER_ID_CANARY_HEARTBEAT) {
                    canary_heartbeat_msg_t *hb = (canary_heartbeat_msg_t*)msg->data.value;
                    
                    ESP_LOGD(TAGZB, "==== HEARTBEAT ==== SRC=0x%04x SEQ=%lu UPTIME=%lu BATTERY=%u%% ====",
                             src_addr, hb->seq_num, hb->uptime_sec, hb->battery_level);
                    
                    /* Invoke the registered callback */
                    heartbeat_received_callback(hb, src_addr);
                    
                } else if (msg->info.cluster == ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS) {
                    canary_network_status_msg_t *status = (canary_network_status_msg_t*)msg->data.value;
                    
                    ESP_LOGI(TAGZB, "==== NETWORK STATUS ==== SRC=0x%04x STATE=%s ====",
                             src_addr, zb_cluster_network_state_to_string(status->state));
                    
                    /* Invoke the registered callback */
                    network_status_received_callback(status, src_addr);
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
    
    /* Initialize device registry */
    ESP_ERROR_CHECK(zb_registry_init());
    
    /* Initialize cluster manager */
    ESP_ERROR_CHECK(zb_cluster_manager_init(role == ZB_MESH_ROLE_COORDINATOR));
    
    /* Register callbacks */
    zb_mesh_register_state_callback(mesh_state_change_callback);
    zb_mesh_register_device_callback(mesh_device_event_callback);
    zb_cluster_register_network_status_callback(network_status_received_callback);
    zb_cluster_register_impact_alert_callback(impact_alert_received_callback);
    zb_cluster_register_gas_alert_callback(gas_alert_received_callback);
    zb_cluster_register_heartbeat_callback(heartbeat_received_callback);

    /* Set overall network size to support multiple routers (must be called before esp_zb_init) */
    esp_zb_overall_network_size_set(20);  /* Support up to 20 devices in network */
    ESP_LOGI(TAGZB, "Network size set to 20 devices (routers + end devices)");
    
    /* Configure Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = (role == ZB_MESH_ROLE_COORDINATOR) ? 
                       ESP_ZB_DEVICE_TYPE_COORDINATOR : ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = ZB_MESH_MAX_CHILDREN,  /* Max children per router/coordinator */
        },
    };
    
    ESP_LOGI(TAGZB, "Network config: max_children=%d", ZB_MESH_MAX_CHILDREN);

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
        
        /* Heartbeat - client side for sending */
        esp_zb_attribute_list_t *heartbeat_cluster_client = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_HEARTBEAT);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, heartbeat_cluster_client, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        
        /* Heartbeat - server side for receiving */
        esp_zb_attribute_list_t *heartbeat_cluster_server = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_CANARY_HEARTBEAT);
        esp_zb_cluster_list_add_custom_cluster(cluster_list, heartbeat_cluster_server, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        
        ESP_LOGE(TAGZB, "!!! CUSTOM CANARY CLUSTERS REGISTERED !!! endpoint=%d, impact=0x%04x, gas=0x%04x, heartbeat=0x%04x, network=0x%04x",
                 HA_ONOFF_SWITCH_ENDPOINT, ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT, ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT,
                 ZB_ZCL_CLUSTER_ID_CANARY_HEARTBEAT, ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS);
    } else {
        ESP_LOGE(TAGZB, "Failed to get cluster list for endpoint");
    }
    
    esp_zb_device_register(ep_list);
    
    /* CRITICAL: Enable RxOnWhenIdle for routers to receive broadcasts */
    esp_zb_set_rx_on_when_idle(true);
    ESP_LOGI(TAGZB, "RxOnWhenIdle enabled - device will listen for broadcasts");
    
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
    const TickType_t factory_reset_time_ticks = pdMS_TO_TICKS(5000);  // 5 seconds for factory reset
    bool factory_reset_triggered = false;
    bool short_press_handled = false;
    uint32_t last_debug_tick = 0;
    
    ESP_LOGI(TAGZB, "Button monitor task started - GPIO %d", BUTTON_GPIO);
    ESP_LOGI(TAGZB, "GPIO23: Single press cancels severe alert");
    ESP_LOGI(TAGZB, "GPIO23: Hold 5 sec for factory reset");
    ESP_LOGI(TAGZB, "GPIO8: Double-press for soft leave (test mode)");
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
            
            uint32_t hold_ms = (hold_duration * 1000) / configTICK_RATE_HZ;
            
            if (hold_duration >= factory_reset_time_ticks) {
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
            
            /* Show progress every second during hold */
            if (hold_ms / 1000 != (hold_ms - 100) / 1000) {
                ESP_LOGI(TAGZB, "Button held for %lu seconds...", hold_ms / 1000);
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
    uint32_t heartbeat_seq = 0;
    uint32_t boot_time_sec = 0;
    int pulse_count = 0;
    int beep_counter = 0;
    
    while (1) {
        bool is_joined = zb_mesh_is_joined();
        uint16_t short_addr = esp_zb_get_short_address();
        
        if (is_joined && short_addr != 0xFFFF) {
            /* Connected - HEARTBEAT LED only flashes when heartbeat received (no local pulse) */
            
            pulse_count++;
            boot_time_sec += 2;  // Increment uptime (2 seconds per loop)
            
            /* Send heartbeat message every 30 seconds (15 pulses) */
            if (pulse_count >= 15) {
                heartbeat_seq++;
                esp_err_t ret = zb_cluster_send_heartbeat(heartbeat_seq, boot_time_sec, 255);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAGZB, "Heartbeat sent: seq=%lu, uptime=%lu sec", heartbeat_seq, boot_time_sec);
                    
                    /* Flash HEARTBEAT LED when sending heartbeat */
                    gpio_set_level(HEARTBEAT_LED_GPIO, 0);  // ON
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_set_level(HEARTBEAT_LED_GPIO, 1);  // OFF
                } else {
                    ESP_LOGW(TAGZB, "Failed to send heartbeat");
                }
                pulse_count = 0;
            }
            
            vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second heartbeat
            
        } else {
            /* Not connected - STATUS LED should be blinking (handled by mesh callback) */
            /* Sound buzzer periodically to alert disconnection */
            
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

/* Device timeout checker task (for coordinators only) */
static void device_timeout_checker_task(void *pvParameters)
{
    uint16_t timed_out_devices[ZB_REGISTRY_MAX_DEVICES];
    uint8_t timeout_count;
    
    ESP_LOGI(TAGZB, "Device timeout checker task started (coordinator mode)");
    ESP_LOGI(TAGZB, "Checking for device timeouts every 30 seconds (timeout threshold: %d sec)", ZB_DEVICE_TIMEOUT_SEC);
    
    while (1) {
        /* Check for device timeouts every 30 seconds */
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        /* Only check if we're joined to network */
        if (!zb_mesh_is_joined()) {
            continue;
        }
        
        ESP_LOGD(TAGZB, "Running timeout check... (devices in registry: %d)", zb_registry_get_count());
        
        /* Check for timed out devices */
        esp_err_t ret = zb_registry_check_timeouts(timed_out_devices, ZB_REGISTRY_MAX_DEVICES, &timeout_count);
        
        if (ret == ESP_OK && timeout_count > 0) {
            ESP_LOGE(TAGZB, "=== %d DEVICE(S) TIMED OUT (NO HEARTBEAT) ===", timeout_count);
            
            for (int i = 0; i < timeout_count; i++) {
                ESP_LOGE(TAGZB, "*** DEVICE TIMEOUT: 0x%04x (no heartbeat for %d sec) ***",
                         timed_out_devices[i], ZB_DEVICE_TIMEOUT_SEC);
                
                /* Flash HEALTH LED to indicate device timeout */
                for (int j = 0; j < 3; j++) {
                    gpio_set_level(HEALTH_LED_GPIO, 0);  // ON
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(HEALTH_LED_GPIO, 1);  // OFF
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                
                /* Alert tone */
                buzzer_beep_ms(LEDC_FREQUENCY, 500);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}

/* Soft leave monitor task - watches for double press on SPIFFS button */
static void soft_leave_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAGZB, "Soft leave monitor task started - double-press GPIO8 to soft leave");
    
    while (1) {
        /* Check every 100ms */
        vTaskDelay(pdMS_TO_TICKS(100));
        
        /* Check if double press was detected */
        if (spiffs_button_press_count >= 2) {
            /* Reset counter */
            spiffs_button_press_count = 0;
            
            ESP_LOGW(TAGZB, "=== SOFT LEAVE TRIGGERED (DOUBLE PRESS) ===");
            ESP_LOGW(TAGZB, "GPIO8 double-pressed - leaving network gracefully...");
            
            /* Flash PENDING LED to indicate soft leave via SPIFFS button */
            for (int i = 0; i < 5; i++) {
                gpio_set_level(PENDING_LED_GPIO, 0);  // ON
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(PENDING_LED_GPIO, 1);  // OFF
                vTaskDelay(pdMS_TO_TICKS(150));
            }
            
            /* Perform soft leave - sends disconnect notification and leaves without factory reset */
            esp_err_t ret = zb_mesh_soft_leave(true);  // true = attempt to rejoin
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAGZB, "Soft leave initiated - device will attempt to rejoin");
            } else {
                ESP_LOGE(TAGZB, "Soft leave failed: %s", esp_err_to_name(ret));
            }
            
            /* Wait a bit before allowing another soft leave */
            vTaskDelay(pdMS_TO_TICKS(3000));
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

    //COMMENTED OUT - ADC initialization (no gas sensor monitoring locally)
    //ADC and I2C tasks enabled
    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    xTaskCreate(adc_task, "adc_task", 8192, handle, 3, &adc_task_handle);

    s_task_handle = adc_task_handle;

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    
    ESP_LOGI(TAGZB, "ADC and I2C tasks enabled");
    

    ESP_LOGW(TAGZB, "============================================================");
    ESP_LOGW(TAGZB, "  ZIGBEE MESH NODE - Impact sensor enabled");
    ESP_LOGW(TAGZB, "  This device will:");
    ESP_LOGW(TAGZB, "    - Monitor accelerometer for impacts (ADXL345)");
    ESP_LOGW(TAGZB, "    - Send impact alerts over Zigbee mesh");
    ESP_LOGW(TAGZB, "    - Receive remote alerts from other nodes");
    ESP_LOGW(TAGZB, "============================================================");

    /* Configure Zigbee platform (radio/host) before starting the Zigbee task */
    {
        esp_zb_platform_config_t zb_config = {
            .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
            .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
        };
        
        /* Optimize radio config for better range and reliability */
        zb_config.radio_config.radio_mode = ZB_RADIO_MODE_NATIVE;
        
        /* Optimize host config for responsiveness */
        /* Increase host connection timeout to prevent premature disconnections */
        
        ESP_ERROR_CHECK(esp_zb_platform_config(&zb_config));
    }
    
    /* Set maximum TX power for better signal strength (20 dBm for ESP32-C6) */
    esp_zb_set_tx_power(20);
    ESP_LOGI(TAGZB, "Zigbee TX power set to maximum (20 dBm) for better range");

    /* Increased stack size to prevent stack overflow with multiple custom clusters */
    xTaskCreate(esp_zb_task, "Zigbee_main", 12288, NULL, 7, NULL);
    
    /* Start heartbeat task to show connection status and send heartbeat messages */
    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 7, NULL);
    ESP_LOGI(TAGZB, "Heartbeat task started - sends heartbeat every 30 seconds");
    
    /* Start device timeout checker for coordinators */
#ifndef ZB_BUILD_AS_ROUTER
    /* Only start timeout checker if building as coordinator */
    xTaskCreate(device_timeout_checker_task, "device_timeout", 2048, NULL, 7, NULL);
    ESP_LOGI(TAGZB, "Device timeout checker started (coordinator mode)");
#endif
    
    /* Start soft leave monitor task - double-press GPIO8 to soft leave */
    xTaskCreate(soft_leave_monitor_task, "soft_leave", 2048, NULL, 7, NULL);
    ESP_LOGI(TAGZB, "Soft leave monitor started - double-press GPIO8 to soft leave");
    
    /* Start button monitor task for factory reset */
    xTaskCreate(button_monitor_task, "button_monitor", 2048, NULL, 3, NULL);
    ESP_LOGI(TAGZB, "Button monitor started - hold GPIO23 5s for factory reset");

    // FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // if (log_file == NULL) {
    //     ESP_LOGE(TAG, "Failed to open log file for writing");
    // }
    //FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // Removed unused log file opening and closing to prevent resource leaks.
}