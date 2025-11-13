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
#include "zb_alert.h"
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

//Definitions for SPI and ADXL345
static const char *TAGSPI = "ADXL345";

#define I2C_MASTER_SCL_IO           9 // SCL on ESP32-C6 (GPIO1) - using bit-bang
#define I2C_MASTER_SDA_IO           8 // SDA on ESP32-C6 (GPIO0) - using bit-bang
#define LED_GPIO1                   18 //CONFIG_LED_GPIO
#define LED_GPIO2                   13 //CONFIG_LED_GPIO

#define BUTTON_GPIO                 15 //CONFIG_BUTTON_GPIO
#define BUZZER_GPIO                 17 //CONFIG_BUZZER_GPIO

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

static const char *TAGZB = "ESP_ZB_ON_OFF_SWITCH";


static adc_channel_t channel[1] = {ADC_CHANNEL_6};  // GPIO11 on ESP32-C6
static TaskHandle_t s_task_handle;
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
// Software I2C (Bit-Banging) Implementation for GPIO0 and GPIO1
// ============================================================================

#define I2C_DELAY_US 50  // Delay in microseconds for I2C timing (increased for weak pull-ups)

typedef struct {
    int sda_pin;
    int scl_pin;
} soft_i2c_t;

static soft_i2c_t soft_i2c = {
    .sda_pin = I2C_MASTER_SDA_IO,
    .scl_pin = I2C_MASTER_SCL_IO,
};

// Initialize GPIO pins for software I2C (open-drain mode)
static void soft_i2c_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << soft_i2c.sda_pin) | (1ULL << soft_i2c.scl_pin),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open-drain for I2C
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Ensure pull-ups are enabled at pad level
    
    // Release lines (set high) and wait for pull-ups
    gpio_set_level(soft_i2c.sda_pin, 1);
    gpio_set_level(soft_i2c.scl_pin, 1);
        esp_rom_delay_us(200);  // Give pull-ups time to work
    
    ESP_LOGI(TAGSPI, "Software I2C initialized on GPIO%d (SCL), GPIO%d (SDA)", 
             soft_i2c.scl_pin, soft_i2c.sda_pin);
}

// Diagnostic function to test I2C bus
static void soft_i2c_diagnose(void)
{
    ESP_LOGI(TAGSPI, "=== I2C Bus Diagnostic ===");
    
    // Check if lines are high (good)
    int sda_level = gpio_get_level(soft_i2c.sda_pin);
    int scl_level = gpio_get_level(soft_i2c.scl_pin);
    ESP_LOGI(TAGSPI, "Bus levels - SDA: %d, SCL: %d (both should be 1)", sda_level, scl_level);
    
    if (sda_level == 0) ESP_LOGW(TAGSPI, "ERROR: SDA stuck low! Check pull-ups");
    if (scl_level == 0) ESP_LOGW(TAGSPI, "ERROR: SCL stuck low! Check pull-ups");
    
    // Try a START condition
    ESP_LOGI(TAGSPI, "Testing START condition...");
    gpio_set_level(soft_i2c.sda_pin, 1);
    gpio_set_level(soft_i2c.scl_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(soft_i2c.sda_pin, 0);  // SDA goes low
    esp_rom_delay_us(10);
    sda_level = gpio_get_level(soft_i2c.sda_pin);
    scl_level = gpio_get_level(soft_i2c.scl_pin);
    ESP_LOGI(TAGSPI, "After START - SDA: %d, SCL: %d (should be 0, 1)", sda_level, scl_level);
    
    // Try a STOP condition
    ESP_LOGI(TAGSPI, "Testing STOP condition...");
    gpio_set_level(soft_i2c.scl_pin, 0);
    esp_rom_delay_us(10);
    gpio_set_level(soft_i2c.scl_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(soft_i2c.sda_pin, 1);  // SDA goes high
    esp_rom_delay_us(10);
    sda_level = gpio_get_level(soft_i2c.sda_pin);
    scl_level = gpio_get_level(soft_i2c.scl_pin);
    ESP_LOGI(TAGSPI, "After STOP - SDA: %d, SCL: %d (should be 1, 1)", sda_level, scl_level);
    
    ESP_LOGI(TAGSPI, "=== End Diagnostic ===");
}

// Quick pin health check performed before we reconfigure pins for bit-bang.
// This reads the pins in a few configurations so we can compare MCU readings
// with your oscilloscope. Keep it very short to avoid changing board state.
static void soft_i2c_pin_health_check(void)
{
    ESP_LOGI(TAGSPI, "=== I2C Pin Health Check (pre-config) ===");

    // Read raw levels as the pins currently are (bootloader/ROM state)
    int raw_sda = gpio_get_level(I2C_MASTER_SDA_IO);
    int raw_scl = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAGSPI, "Raw levels before config - SDA(GPIO%d): %d, SCL(GPIO%d): %d",
             I2C_MASTER_SDA_IO, raw_sda, I2C_MASTER_SCL_IO, raw_scl);

    // Configure as inputs without pull to see passive level
    gpio_config_t io_in_none = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SDA_IO) | (1ULL << I2C_MASTER_SCL_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_in_none);
    esp_rom_delay_us(200);
    int in_none_sda = gpio_get_level(I2C_MASTER_SDA_IO);
    int in_none_scl = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAGSPI, "Input(no pull) - SDA: %d, SCL: %d", in_none_sda, in_none_scl);

    // Configure as inputs with internal pull-up
    gpio_config_t io_in_pu = io_in_none;
    io_in_pu.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_in_pu);
    esp_rom_delay_us(200);
    int in_pu_sda = gpio_get_level(I2C_MASTER_SDA_IO);
    int in_pu_scl = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAGSPI, "Input(with pull-up) - SDA: %d, SCL: %d", in_pu_sda, in_pu_scl);

    // Configure as open-drain outputs and drive high (release) then sample
    gpio_config_t io_od = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SDA_IO) | (1ULL << I2C_MASTER_SCL_IO),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_od);
    // Release lines
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    esp_rom_delay_us(200);
    int od_rel_sda = gpio_get_level(I2C_MASTER_SDA_IO);
    int od_rel_scl = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAGSPI, "OD release - SDA: %d, SCL: %d", od_rel_sda, od_rel_scl);

    // Drive lines low (simulate bus held low) and sample
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    gpio_set_level(I2C_MASTER_SCL_IO, 0);
    esp_rom_delay_us(200);
    int od_low_sda = gpio_get_level(I2C_MASTER_SDA_IO);
    int od_low_scl = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAGSPI, "OD drive low - SDA: %d, SCL: %d", od_low_sda, od_low_scl);

    ESP_LOGI(TAGSPI, "=== End Pin Health Check ===");
}

// Delay for I2C timing
static inline void soft_i2c_delay(void)
{
    esp_rom_delay_us(I2C_DELAY_US);
}

// Start condition: SDA goes low while SCL is high
static void soft_i2c_start(void)
{
    gpio_set_level(soft_i2c.sda_pin, 1);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.scl_pin, 1);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.sda_pin, 0);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.scl_pin, 0);
    soft_i2c_delay();
}

// Stop condition: SDA goes high while SCL is high
static void soft_i2c_stop(void)
{
    gpio_set_level(soft_i2c.sda_pin, 0);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.scl_pin, 1);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.sda_pin, 1);
    soft_i2c_delay();
}

// Send one bit on I2C bus
static void soft_i2c_send_bit(uint8_t bit)
{
    gpio_set_level(soft_i2c.sda_pin, bit ? 1 : 0);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.scl_pin, 1);
    soft_i2c_delay();
    gpio_set_level(soft_i2c.scl_pin, 0);
    soft_i2c_delay();
}

// Receive one bit from I2C bus
static uint8_t soft_i2c_recv_bit(void)
{
    uint8_t bit;
    gpio_set_level(soft_i2c.sda_pin, 1);  // Release line for slave to pull
    soft_i2c_delay();
    gpio_set_level(soft_i2c.scl_pin, 1);
    soft_i2c_delay();
    bit = gpio_get_level(soft_i2c.sda_pin);
    gpio_set_level(soft_i2c.scl_pin, 0);
    soft_i2c_delay();
    return bit;
}

// Send one byte on I2C bus, return true if ACK received
static bool soft_i2c_send_byte(uint8_t byte)
{
    // Send 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        soft_i2c_send_bit((byte >> i) & 1);
    }
    // Receive ACK bit (slave pulls SDA low)
    bool ack = (soft_i2c_recv_bit() == 0);
    return ack;
}

// Receive one byte from I2C bus, send ACK if ack_flag is true
static uint8_t soft_i2c_recv_byte(bool send_ack)
{
    uint8_t byte = 0;
    // Receive 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        uint8_t bit = soft_i2c_recv_bit();
        byte |= (bit << i);
        ESP_LOGV(TAGSPI, "I2C recv bit[%d]=%d, byte so far=0x%02X", 7-i, bit, byte);
    }
    // Send ACK or NAK
    soft_i2c_send_bit(send_ack ? 0 : 1);
    ESP_LOGV(TAGSPI, "I2C recv complete: 0x%02X (ACK=%d)", byte, send_ack);
    return byte;
}

// Send address and check for ACK
static bool soft_i2c_send_addr(uint8_t addr, uint8_t read_write)
{
    uint8_t addr_byte = (addr << 1) | (read_write & 1);
    return soft_i2c_send_byte(addr_byte);
}

// I2C write: address + data bytes
static esp_err_t soft_i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    soft_i2c_start();
    
    // Send address with write bit
    if (!soft_i2c_send_addr(addr, 0)) {
        ESP_LOGW(TAGSPI, "I2C: No ACK from slave at address 0x%02X", addr);
        soft_i2c_stop();
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Send data bytes
    for (size_t i = 0; i < len; i++) {
        if (!soft_i2c_send_byte(data[i])) {
            ESP_LOGW(TAGSPI, "I2C: No ACK after data byte %d", i);
            soft_i2c_stop();
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    
    soft_i2c_stop();
    return ESP_OK;
}

// I2C read: address + read data bytes
static esp_err_t soft_i2c_read(uint8_t addr, uint8_t *data, size_t len)
{
    soft_i2c_start();
    
    // Send address with read bit
    if (!soft_i2c_send_addr(addr, 1)) {
        ESP_LOGW(TAGSPI, "I2C: No ACK from slave at address 0x%02X (read)", addr);
        soft_i2c_stop();
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Receive data bytes
    for (size_t i = 0; i < len; i++) {
        bool send_ack = (i < len - 1);  // Send ACK for all but last byte
        data[i] = soft_i2c_recv_byte(send_ack);
    }
    
    soft_i2c_stop();
    return ESP_OK;
}

// I2C write + read (repeated start)
static esp_err_t soft_i2c_write_read(uint8_t addr, const uint8_t *write_data, size_t write_len,
                                       uint8_t *read_data, size_t read_len)
{
    soft_i2c_start();
    
    // Send address with write bit
    if (!soft_i2c_send_addr(addr, 0)) {
        ESP_LOGW(TAGSPI, "I2C: No ACK from slave at address 0x%02X (write_read)", addr);
        soft_i2c_stop();
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Send write data
    for (size_t i = 0; i < write_len; i++) {
        if (!soft_i2c_send_byte(write_data[i])) {
            ESP_LOGW(TAGSPI, "I2C: No ACK after write data byte %d", i);
            soft_i2c_stop();
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    
    // Repeated start
    soft_i2c_start();
    
    // Send address with read bit
    if (!soft_i2c_send_addr(addr, 1)) {
        ESP_LOGW(TAGSPI, "I2C: No ACK from slave at address 0x%02X (repeated read)", addr);
        soft_i2c_stop();
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Receive read data
    for (size_t i = 0; i < read_len; i++) {
        bool send_ack = (i < read_len - 1);
        read_data[i] = soft_i2c_recv_byte(send_ack);
    }
    
    soft_i2c_stop();
    return ESP_OK;
}

// ============================================================================
// ADXL345 functions adapted for software I2C
// ============================================================================

static esp_err_t adxl345_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    const int max_retries = 3;
    esp_err_t ret;

    uint8_t reg = reg_addr;
    if (len > 1) {
        reg |= 0x80; // ADXL345 multi-byte read flag (MSB)
    }

    for (int i = 0; i < max_retries; ++i) {
        // Use software I2C for bit-bang communication
        ret = soft_i2c_write_read(ADXL345_ADDR, &reg, 1, data, len);
        if (ret == ESP_OK) return ESP_OK;
        ESP_LOGW(TAGSPI, "adxl345_register_read retry %d failed: %s", i + 1, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ret;
}

/* Simple runtime probe: read DEVID and log result. Returns ESP_OK if device responds. */
static esp_err_t adxl_probe(i2c_master_dev_handle_t dev_handle)
{
    uint8_t devid = 0;
    esp_err_t ret = adxl345_register_read(dev_handle, ADXL345_DEVID_REG, &devid, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGSPI, "I2C probe failed communicating with ADXL345: %s", esp_err_to_name(ret));
        ESP_LOGE(TAGSPI, "Check connections: SCL=GPIO%d, SDA=GPIO%d, ADDR=0x%02X", 
                 I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, ADXL345_ADDR);
        return ret;
    }
    ESP_LOGI(TAGSPI, "ADXL345 probe successful: DEVID=0x%02X", devid);
    if (devid != ADXL345_DEVID_EXPECTED) {
        ESP_LOGW(TAGSPI, "Unexpected ADXL345 DEVID: got 0x%02X, expected 0x%02X", devid, ADXL345_DEVID_EXPECTED);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t adxl345_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = { reg_addr, data };
    const int max_retries = 3;
    esp_err_t ret;
    for (int i = 0; i < max_retries; ++i) {
        // Use software I2C for bit-bang communication
        ret = soft_i2c_write(ADXL345_ADDR, write_buf, sizeof(write_buf));
        if (ret == ESP_OK) return ESP_OK;
        ESP_LOGW(TAGSPI, "adxl345_register_write_byte retry %d failed: %s", i + 1, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ret;
}

static void i2c_bus_recovery(int scl_pin, int sda_pin)
{
    // Configure pins as GPIO output, open-drain
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << scl_pin) | (1ULL << sda_pin),
        .mode = GPIO_MODE_OUTPUT_OD,  // Open-drain
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Generate 9 clock pulses to recover from stuck slave
    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(scl_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Generate STOP condition
    gpio_set_level(sda_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(scl_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(scl_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(sda_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    ESP_LOGI(TAGSPI, "I2C bus recovery completed");
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .flags = {
            .disable_ack_check = false,
        },
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
    gpio_set_level(gpio,1);
}

static void flash_led(int GPIO){
    for(int i=0;i<10;i++){
        gpio_set_level(GPIO,0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(GPIO,1);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
//static const ledc_channel_t BUZZER_LEDC_CHANNEL = LEDC_CHANNEL_0;
//static const ledc_timer_bit_t BUZZER_DUTY_RES = LEDC_TIMER_8_BIT;
//static const ledc_timer_t BUZZER_LEDC_TIMER = LEDC_TIMER_0;

/* Select LEDC speed mode; fall back to 0 if symbol not defined for this target */
#if defined(LEDC_HIGH_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_HIGH_SPEED_MODE
#elif defined(LEDC_LOW_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#else
#define BUZZER_LEDC_MODE 0
#endif
/*
static void buzzer_stop_timer_cb(TimerHandle_t xTimer)
{
    
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    xTimerDelete(xTimer, 0);
}

static void buzzer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .duty_resolution  = BUZZER_DUTY_RES,
        .timer_num        = BUZZER_LEDC_TIMER,
        .freq_hz          = 2000, // default, will be updated per beep
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = BUZZER_GPIO,
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void buzzer_beep_ms(uint32_t freq_hz, uint32_t duration_ms)
{
    
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    uint32_t max_duty = (1 << BUZZER_DUTY_RES) - 1;
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, max_duty / 2);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);

    
    TimerHandle_t t = xTimerCreate("bstop", pdMS_TO_TICKS(duration_ms), pdFALSE, NULL, buzzer_stop_timer_cb);
    if (t) {
        xTimerStart(t, 0);
    }
}
*/

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
    /* ISR: avoid calling non-ISR safe APIs (like ESP_LOG) here.
       Stop the timer from ISR and set a flag so the main task can log.
    */
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
                /* notify Zigbee network (debounced) */
                //send_zigbee_alert_toggle_debounced(5000);

                fprintf(f, "ADC%d, Channel: %d, Value: %" PRIu32 "\n",
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
                        ESP_LOGI(TAGADC, "ADC%d Ch%d raw=%"PRIu32" mV=%"PRIu32,
                                parsed_data[i].unit + 1,
                                parsed_data[i].channel,
                                raw,
                                voltage_mv);
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

//begin zigbee switch code

static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        /* implemented light switch toggle functionality */
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAGZB, "Send 'on_off toggle' command");
    }
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited) {
        ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler),
                            ESP_FAIL, TAGZB, "Failed to initialize switch driver");
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAGZB, "Failed to start Zigbee bdb commissioning");
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAGZB, "Bound successfully!");
        if (user_ctx) {
            light_bulb_device_params_t *light = (light_bulb_device_params_t *)user_ctx;
            ESP_LOGI(TAGZB, "The light originating from address(0x%x) on endpoint(%d)", light->short_addr, light->endpoint);
            free(light);
        }
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAGZB, "Found light");
        esp_zb_zdo_bind_req_param_t bind_req;
        light_bulb_device_params_t *light = (light_bulb_device_params_t *)malloc(sizeof(light_bulb_device_params_t));
        light->endpoint = endpoint;
        light->short_addr = addr;
        esp_zb_ieee_address_by_short(light->short_addr, light->ieee_addr);
        esp_zb_get_long_address(bind_req.src_address);
        bind_req.src_endp = HA_ONOFF_SWITCH_ENDPOINT;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req.dst_address_u.addr_long, light->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req.dst_endp = endpoint;
        bind_req.req_dst_addr = esp_zb_get_short_address(); /* TODO: Send bind request to self */
        ESP_LOGI(TAGZB, "Try to bind On/Off");
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)light);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAGZB, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAGZB, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAGZB, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAGZB, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                esp_zb_bdb_open_network(180);
                ESP_LOGI(TAGZB, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAGZB, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAGZB, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAGZB, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAGZB, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAGZB, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        esp_zb_zdo_match_desc_req_param_t  cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        esp_zb_zdo_find_on_off_light(&cmd_req, user_find_cb, NULL);
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAGZB, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAGZB, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    default:
        ESP_LOGI(TAGZB, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    /* Create On/Off switch endpoint (Basic cluster is included automatically) */
    esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_ep_list_t *switch_ep = esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);

    /* Register endpoint and start Zigbee network */
    esp_zb_device_register(switch_ep);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    ESP_LOGI("HA_SWITCH", "Zigbee HA Switch running");

    /* Main loop */
    esp_zb_stack_main_loop();
}


static void i2c_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for system to stabilize
    // With software I2C bit-bang, we don't need the dev_handle
    // The arg parameter is unused in this case
    (void)arg;
    
    uint8_t data[6];

    while (1) {
    // Use software I2C to read ADXL345 data - pass NULL for dev_handle
    esp_err_t ret = adxl345_register_read(NULL, ADXL345_DATAX0_REG, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGW(TAGSPI, "Failed to read ADXL345 data: %s — will retry after delay", esp_err_to_name(ret));
        // optional: re-probe device once in a while
        vTaskDelay(pdMS_TO_TICKS(500));
        continue; // skip parsing, keep task alive
    }

    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

    float x_g = x_raw * LSB_TO_G;
    float y_g = y_raw * LSB_TO_G;
    float z_g = z_raw * LSB_TO_G - 1.0f; // gravity

    classify_impact(x_g, y_g, z_g);

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
    TaskHandle_t adc_task_handle = NULL;

    // Initialize software I2C (bit-banging on GPIO0 and GPIO1)
    // Quick pin health check before we reconfigure pins. This will log the
    // raw pin levels and try a couple of small experiments so you can compare
    // what the MCU reads vs your oscilloscope.
    soft_i2c_pin_health_check();

    // Initialize software I2C (bit-banging on GPIO0 and GPIO1)
    soft_i2c_init();
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAGSPI, "Software I2C initialized (bit-bang mode)");

    // Run I2C bus diagnostics
    soft_i2c_diagnose();

    // Probe ADXL345 device (with retries) - device handle is NULL for bit-bang
    // if (adxl_probe(NULL) != ESP_OK) {
    //     ESP_LOGE(TAGSPI, "ADXL345 probe failed; aborting startup");
    //     return;
    // }

    // Enable measurement mode
    ESP_ERROR_CHECK(adxl345_register_write_byte(NULL, ADXL345_POWER_CTL_REG, 0x08));

    // Set data format: Full resolution, ±2g range (0x08)
    ESP_ERROR_CHECK(adxl345_register_write_byte(NULL, ADXL345_DATA_FORMAT_REG, 0x08));
    init_led(LED_GPIO1);
    init_led(LED_GPIO2);
    //buzzer_init();

    // Configure button input
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // enable pull-up
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE, // falling edge
    };
    gpio_config(&io_conf);

    // Install ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_handler, NULL);

    // Create major impact timer (10 sec)
    major_impact_timer = xTimerCreate("MajorImpactTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, major_impact_timer_callback);

    // Pass NULL for i2c_task since we're using bit-bang I2C (no i2c_master_dev_handle)
    xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 5, NULL);
    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    /* NOTE: i2c_task already created above. Using software I2C bit-bang. */
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    //xTaskCreate(adc_task, "adc_task", 4096, handle, 5, &adc_task_handle);
    s_task_handle = adc_task_handle;  // used by ISR callback

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    //xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    //FILE *log_file = fopen("/spiffs/adc_log.txt", "a");
    // Removed unused log file opening and closing to prevent resource leaks.
}