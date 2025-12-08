/*
 * CanaryCap Zigbee Device Registry Implementation
 */

#include "zb_device_registry.h"
#include "zb_canary_cluster.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "zb_registry";

/* Internal state */
static zb_device_info_t device_registry[ZB_REGISTRY_MAX_DEVICES];
static uint8_t device_count = 0;
static SemaphoreHandle_t registry_mutex = NULL;
static bool discovery_in_progress = false;

/* Forward declarations */
static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx);
static void ieee_addr_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx);
static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx);

/* ============================================================================
 * Initialization
 * ============================================================================ */

esp_err_t zb_registry_init(void)
{
    if (registry_mutex != NULL) {
        ESP_LOGW(TAG, "Registry already initialized");
        return ESP_OK;
    }

    memset(device_registry, 0, sizeof(device_registry));
    device_count = 0;
    discovery_in_progress = false;

    registry_mutex = xSemaphoreCreateMutex();
    if (!registry_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Device registry initialized");
    return ESP_OK;
}

/* ============================================================================
 * Device Discovery
 * ============================================================================ */

esp_err_t zb_registry_start_discovery(void)
{
    if (discovery_in_progress) {
        ESP_LOGW(TAG, "Discovery already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting device discovery for custom clusters...");
    discovery_in_progress = true;
    
    /* Don't clear registry - keep existing devices and add new ones */

    /* Search for devices with our custom impact alert cluster */
    esp_zb_zdo_match_desc_req_param_t find_req;
    find_req.dst_nwk_addr = 0xFFFF;           // Broadcast to all devices
    find_req.addr_of_interest = 0xFFFD;       // All routers and coordinator (not sleepy end devices)
    find_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    
    /* Build cluster list for match descriptor */
    static uint16_t cluster_list[] = {
        ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT,
        ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT
    };
    
    /* Set input clusters (servers) and output clusters (clients) */
    find_req.num_in_clusters = 2;  // Looking for servers with these clusters
    find_req.num_out_clusters = 0;
    find_req.cluster_list = cluster_list;
    
    esp_zb_zdo_match_cluster(&find_req, user_find_cb, NULL);

    return ESP_OK;
}

/* ============================================================================
 * ZDO Callbacks
 * ============================================================================ */

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        uint16_t my_addr = esp_zb_get_short_address();
        
        /* Don't add ourselves */
        if (addr == my_addr) {
            ESP_LOGD(TAG, "Ignoring self in device discovery");
            return;
        }

        ESP_LOGI(TAG, "Found device: addr=0x%04x, endpoint=%d", addr, endpoint);
        
        /* Rate limiting: big delay between processing each discovered device */
        static int64_t last_process_time = 0;
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - last_process_time;
        if (elapsed < 500000LL) {  // Less than 500ms since last device
            int64_t wait_time = 500000LL - elapsed;
            vTaskDelay(pdMS_TO_TICKS(wait_time / 1000));
        }
        last_process_time = esp_timer_get_time();
        
        /* Check if device already exists in registry */
        zb_device_info_t existing_device;
        esp_err_t ret = zb_registry_get_device(addr, &existing_device);
        
        if (ret == ESP_OK) {
            /* Device already in registry - check if we need to update binding */
            if (existing_device.is_bound) {
                ESP_LOGI(TAG, "Device 0x%04x already discovered and bound", addr);
                return;
            }
            ESP_LOGI(TAG, "Device 0x%04x exists but not bound yet", addr);
        } else {
            /* New device - add to registry */
            ret = zb_registry_add_device(addr, endpoint);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to add device 0x%04x to registry", addr);
                return;
            }
        }

        /* Get IEEE address for binding */
        esp_zb_zdo_ieee_addr_req_param_t ieee_req;
        ieee_req.addr_of_interest = addr;
        ieee_req.dst_nwk_addr = addr;
        ieee_req.request_type = 0;  // Single device response
        ieee_req.start_index = 0;
        
        /* Additional delay before requesting IEEE address */
        vTaskDelay(pdMS_TO_TICKS(200));
        
        esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_addr_cb, (void *)(uintptr_t)addr);
    } else {
        ESP_LOGD(TAG, "Device discovery completed or no match found");
        discovery_in_progress = false;
    }
}

static void ieee_addr_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS && resp) {
        uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
        
        ESP_LOGI(TAG, "Got IEEE addr for 0x%04x: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 short_addr, resp->ieee_addr[7], resp->ieee_addr[6], resp->ieee_addr[5], 
                 resp->ieee_addr[4], resp->ieee_addr[3], resp->ieee_addr[2], 
                 resp->ieee_addr[1], resp->ieee_addr[0]);
        
        /* Update registry with IEEE address */
        zb_registry_update_ieee_addr(short_addr, resp->ieee_addr);
        
        /* Delay before binding to allow buffers to clear */
        vTaskDelay(pdMS_TO_TICKS(100));
        
        /* Get device info for binding */
        zb_device_info_t device;
        if (zb_registry_get_device(short_addr, &device) == ESP_OK) {
            /* Create binding from remote device to us for impact alerts */
            esp_zb_zdo_bind_req_param_t bind_req;
            memcpy(bind_req.src_address, device.ieee_addr, sizeof(esp_zb_ieee_addr_t));
            bind_req.src_endp = device.endpoint;
            bind_req.cluster_id = ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT;
            bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
            esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
            bind_req.dst_endp = HA_ONOFF_SWITCH_ENDPOINT;
            bind_req.req_dst_addr = device.short_addr;
            
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)(uintptr_t)short_addr);
            
            /* Delay before second bind to prevent buffer exhaustion */
            vTaskDelay(pdMS_TO_TICKS(200));
            
            /* Also bind gas alert cluster */
            bind_req.cluster_id = ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT;
            esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)(uintptr_t)short_addr);
        }
    }
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Successfully bound to device 0x%04x", short_addr);
        zb_registry_mark_bound(short_addr);
    } else {
        ESP_LOGW(TAG, "Binding failed for device 0x%04x: status=%d", short_addr, zdo_status);
    }
}

/* ============================================================================
 * Registry Management
 * ============================================================================ */

esp_err_t zb_registry_add_device(uint16_t short_addr, uint8_t endpoint)
{
    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Check if already exists */
    for (int i = 0; i < device_count; i++) {
        if (device_registry[i].short_addr == short_addr) {
            device_registry[i].endpoint = endpoint;
            xSemaphoreGive(registry_mutex);
            ESP_LOGD(TAG, "Updated existing device 0x%04x", short_addr);
            return ESP_OK;
        }
    }

    /* Add new device */
    if (device_count >= ZB_REGISTRY_MAX_DEVICES) {
        xSemaphoreGive(registry_mutex);
        ESP_LOGW(TAG, "Registry full, cannot add device 0x%04x", short_addr);
        return ESP_ERR_NO_MEM;
    }

    zb_device_info_t *dev = &device_registry[device_count];
    memset(dev, 0, sizeof(zb_device_info_t));
    dev->short_addr = short_addr;
    dev->endpoint = endpoint;
    dev->has_impact_cluster = true;  // Assumed from discovery
    dev->has_gas_cluster = true;     // Assumed from discovery
    dev->is_bound = false;
    
    device_count++;
    xSemaphoreGive(registry_mutex);
    
    ESP_LOGI(TAG, "Added device 0x%04x (total: %d)", short_addr, device_count);
    return ESP_OK;
}

esp_err_t zb_registry_update_ieee_addr(uint16_t short_addr, const esp_zb_ieee_addr_t ieee_addr)
{
    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < device_count; i++) {
        if (device_registry[i].short_addr == short_addr) {
            memcpy(device_registry[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
            xSemaphoreGive(registry_mutex);
            return ESP_OK;
        }
    }

    xSemaphoreGive(registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zb_registry_mark_bound(uint16_t short_addr)
{
    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < device_count; i++) {
        if (device_registry[i].short_addr == short_addr) {
            device_registry[i].is_bound = true;
            xSemaphoreGive(registry_mutex);
            return ESP_OK;
        }
    }

    xSemaphoreGive(registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zb_registry_get_devices(zb_device_info_t *devices, uint8_t max_devices, uint8_t *count)
{
    if (!devices || !count) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t copy_count = (device_count < max_devices) ? device_count : max_devices;
    memcpy(devices, device_registry, copy_count * sizeof(zb_device_info_t));
    *count = copy_count;

    xSemaphoreGive(registry_mutex);
    return ESP_OK;
}

esp_err_t zb_registry_get_device(uint16_t short_addr, zb_device_info_t *device)
{
    if (!device) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < device_count; i++) {
        if (device_registry[i].short_addr == short_addr) {
            memcpy(device, &device_registry[i], sizeof(zb_device_info_t));
            xSemaphoreGive(registry_mutex);
            return ESP_OK;
        }
    }

    xSemaphoreGive(registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zb_registry_remove_device(uint16_t short_addr)
{
    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < device_count; i++) {
        if (device_registry[i].short_addr == short_addr) {
            /* Shift remaining devices */
            for (int j = i; j < device_count - 1; j++) {
                memcpy(&device_registry[j], &device_registry[j + 1], sizeof(zb_device_info_t));
            }
            device_count--;
            xSemaphoreGive(registry_mutex);
            ESP_LOGI(TAG, "Removed device 0x%04x", short_addr);
            return ESP_OK;
        }
    }

    xSemaphoreGive(registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

void zb_registry_clear(void)
{
    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(device_registry, 0, sizeof(device_registry));
        device_count = 0;
        discovery_in_progress = false;
        xSemaphoreGive(registry_mutex);
        ESP_LOGI(TAG, "Registry cleared");
    }
}

uint8_t zb_registry_get_count(void)
{
    uint8_t count = 0;
    if (xSemaphoreTake(registry_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        count = device_count;
        xSemaphoreGive(registry_mutex);
    }
    return count;
}
