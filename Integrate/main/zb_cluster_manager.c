/*
 * CanaryCap Zigbee Cluster Manager Implementation
 */

#include "zb_cluster_manager.h"
#include "zb_mesh_network.h"
#include "zb_device_registry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "esp_zb_switch.h" /* for HA_ONOFF_SWITCH_ENDPOINT */
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "zb_cluster_mgr";

/* ============================================================================
 * Internal state
 * ============================================================================ */

static bool cluster_mgr_initialized = false;
static bool cluster_mgr_joined = false;
static bool is_coordinator_role = false;

/* Callback function pointers */
static zb_network_status_callback_t network_status_cb = NULL;
static zb_impact_alert_callback_t impact_alert_cb = NULL;
static zb_gas_alert_callback_t gas_alert_cb = NULL;
static zb_heartbeat_callback_t heartbeat_cb = NULL;

/* Mutex for thread-safe access */
static SemaphoreHandle_t cluster_mutex = NULL;

/* Attribute storage for custom clusters */
static struct {
    uint8_t state;
    uint16_t short_addr;
    uint32_t timestamp;
} network_status_attrs;

static struct {
    uint8_t severity;
    uint8_t type;
    int16_t magnitude;
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
    uint32_t timestamp;
} impact_alert_attrs;

static struct {
    uint8_t detected;
    uint16_t voltage_mv;
    uint16_t raw_value;
    uint32_t timestamp;
} gas_alert_attrs;

static struct {
    uint32_t seq_num;
    uint32_t uptime_sec;
    uint32_t timestamp;
    uint8_t battery_level;
} heartbeat_attrs;

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

static uint32_t get_timestamp_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

const char* zb_cluster_impact_type_to_string(canary_impact_type_t type)
{
    switch (type) {
        case CANARY_IMPACT_TYPE_UNKNOWN: return "Unknown";
        case CANARY_IMPACT_TYPE_FALL: return "Fall";
        case CANARY_IMPACT_TYPE_CEILING_COLLAPSE: return "Ceiling Collapse";
        case CANARY_IMPACT_TYPE_HARD_LANDING: return "Hard Landing";
        case CANARY_IMPACT_TYPE_FRONT: return "Front Impact";
        case CANARY_IMPACT_TYPE_REAR: return "Rear Impact";
        case CANARY_IMPACT_TYPE_RIGHT_SIDE: return "Right Side Impact";
        case CANARY_IMPACT_TYPE_LEFT_SIDE: return "Left Side Impact";
        case CANARY_IMPACT_TYPE_BLUNT: return "Blunt Impact";
        default: return "Invalid";
    }
}

const char* zb_cluster_network_state_to_string(canary_network_state_t state)
{
    switch (state) {
        case CANARY_NETWORK_DISCONNECTED: return "Disconnected";
        case CANARY_NETWORK_CONNECTED: return "Connected";
        case CANARY_NETWORK_REJOINING: return "Rejoining";
        default: return "Invalid";
    }
}

const char* zb_cluster_impact_severity_to_string(canary_impact_severity_t severity)
{
    switch (severity) {
        case CANARY_IMPACT_NONE: return "None";
        case CANARY_IMPACT_MINOR: return "Minor";
        case CANARY_IMPACT_MAJOR: return "Major";
        case CANARY_IMPACT_SEVERE: return "Severe";
        default: return "Invalid";
    }
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

esp_err_t zb_cluster_manager_init(bool is_coordinator)
{
    if (cluster_mgr_initialized) {
        ESP_LOGW(TAG, "Cluster manager already initialized");
        return ESP_OK;
    }

    is_coordinator_role = is_coordinator;
    
    /* Create mutex for thread safety */
    cluster_mutex = xSemaphoreCreateMutex();
    if (!cluster_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize attribute storage */
    memset(&network_status_attrs, 0, sizeof(network_status_attrs));
    memset(&impact_alert_attrs, 0, sizeof(impact_alert_attrs));
    memset(&gas_alert_attrs, 0, sizeof(gas_alert_attrs));

    cluster_mgr_initialized = true;
    
    ESP_LOGI(TAG, "Cluster manager initialized (role=%s)", 
             is_coordinator ? "COORDINATOR" : "ROUTER");
    
    return ESP_OK;
}

bool zb_cluster_manager_is_ready(void)
{
    return cluster_mgr_initialized && cluster_mgr_joined;
}

void zb_cluster_manager_set_joined(bool joined)
{
    if (xSemaphoreTake(cluster_mutex, portMAX_DELAY) == pdTRUE) {
        cluster_mgr_joined = joined;
        xSemaphoreGive(cluster_mutex);
    }
    
    ESP_LOGI(TAG, "Network status changed: %s", joined ? "JOINED" : "DISCONNECTED");
}

/* ============================================================================
 * Network Status Messages
 * ============================================================================ */

esp_err_t zb_cluster_send_network_status(canary_network_state_t state)
{
    if (!cluster_mgr_initialized) {
        ESP_LOGW(TAG, "Cluster manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!cluster_mgr_joined && state != CANARY_NETWORK_DISCONNECTED) {
        ESP_LOGD(TAG, "Not joined to network, skipping network status send");
        return ESP_ERR_INVALID_STATE;
    }

    canary_network_status_msg_t msg;
    msg.state = state;
    msg.short_addr = esp_zb_get_short_address();
    msg.timestamp = get_timestamp_ms();
    
    uint16_t pan_id = esp_zb_get_pan_id();
    msg.pan_id_high = (pan_id >> 8) & 0xFF;
    msg.pan_id_low = pan_id & 0xFF;
    msg.channel = esp_zb_get_current_channel();

    /* Build ZCL command with proper addressing */
    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req;
    memset(&cmd_req, 0, sizeof(cmd_req));
    
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.zcl_basic_cmd.dst_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    
    /* Try unicast to coordinator first if we're a router */
    if (!is_coordinator_role) {
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000; // Coordinator
    } else {
        /* Coordinator broadcasts to all */
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT;
        cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0xFFFF; // Broadcast
    }
    
    cmd_req.cluster_id = ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS;
    cmd_req.custom_cmd_id = ZB_ZCL_CMD_NETWORK_STATUS_REPORT_ID;
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd_req.data.type = ESP_ZB_ZCL_ATTR_TYPE_ARRAY;
    cmd_req.data.value = &msg;
    cmd_req.data.size = sizeof(msg);

    /* Small delay to prevent buffer exhaustion */
    vTaskDelay(pdMS_TO_TICKS(100));
    
    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t seq = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (seq != 0) {
        ESP_LOGI(TAG, "Network status sent: %s (seq=%d)", 
                 zb_cluster_network_state_to_string(state), seq);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send network status (seq=0)");
        return ESP_FAIL;
    }
}

esp_err_t zb_cluster_register_network_status_callback(zb_network_status_callback_t callback)
{
    if (xSemaphoreTake(cluster_mutex, portMAX_DELAY) == pdTRUE) {
        network_status_cb = callback;
        xSemaphoreGive(cluster_mutex);
        ESP_LOGI(TAG, "Network status callback registered");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/* ============================================================================
 * Impact Alert Messages
 * ============================================================================ */

esp_err_t zb_cluster_send_impact_alert(
    canary_impact_severity_t severity,
    canary_impact_type_t type,
    float magnitude,
    float x_g,
    float y_g,
    float z_g)
{
    if (!cluster_mgr_initialized) {
        ESP_LOGW(TAG, "Cluster manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t short_addr = esp_zb_get_short_address();
    if (!cluster_mgr_joined || short_addr == 0xFFFF || short_addr == 0xFFFE) {
        ESP_LOGW(TAG, "Not ready to send impact alert (joined=%d, addr=0x%04x)", 
                 cluster_mgr_joined, short_addr);
        return ESP_ERR_INVALID_STATE;
    }

    /* Debouncing: prevent spam */
    static int64_t last_impact_time = 0;
    int64_t now = esp_timer_get_time();
    if ((now - last_impact_time) < 2000000LL) { // 2 seconds
        ESP_LOGD(TAG, "Impact alert debounced");
        return ESP_OK;
    }
    last_impact_time = now;
    
    ESP_LOGI(TAG, "Preparing to send impact alert: %s (severity=%s, mag=%.2fg)",
             zb_cluster_impact_type_to_string(type),
             zb_cluster_impact_severity_to_string(severity),
             magnitude);

    canary_impact_alert_msg_t msg;
    msg.severity = severity;
    msg.type = type;
    msg.magnitude = (int16_t)(magnitude * 1000.0f);
    msg.x_axis = (int16_t)(x_g * 1000.0f);
    msg.y_axis = (int16_t)(y_g * 1000.0f);
    msg.z_axis = (int16_t)(z_g * 1000.0f);
    msg.timestamp = get_timestamp_ms();
    msg.source_addr = esp_zb_get_short_address();

    /* Log locally first */
    ESP_LOGW(TAG, "*** IMPACT ALERT (LOCAL) *** %s | severity=%s | mag=%.2fg | x=%.2fg y=%.2fg z=%.2fg | src=0x%04x",
             zb_cluster_impact_type_to_string(type),
             zb_cluster_impact_severity_to_string(severity),
             magnitude, x_g, y_g, z_g,
             esp_zb_get_short_address());
    
    /* Get discovered devices from registry */
    zb_device_info_t devices[ZB_REGISTRY_MAX_DEVICES];
    uint8_t device_count = 0;
    zb_registry_get_devices(devices, ZB_REGISTRY_MAX_DEVICES, &device_count);
    
    ESP_LOGI(TAG, "Sending impact alert to %d discovered devices + coordinator", device_count);
    
    /* Build ZCL command */
    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req;
    memset(&cmd_req, 0, sizeof(cmd_req));
    
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.zcl_basic_cmd.dst_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    cmd_req.cluster_id = ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT;
    cmd_req.custom_cmd_id = ZB_ZCL_CMD_IMPACT_ALERT_REPORT_ID;
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd_req.data.type = ESP_ZB_ZCL_ATTR_TYPE_ARRAY;
    cmd_req.data.value = (void *)&msg;
    cmd_req.data.size = sizeof(msg);
    
    int success_count = 0;
    
    /* Send to coordinator (always) */
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t seq = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();
    if (seq != 0) {
        ESP_LOGI(TAG, "Impact alert sent to coordinator 0x0000, seq=%d", seq);
        success_count++;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Increased delay
    
    /* Send to each discovered device (unicast) */
    for (int i = 0; i < device_count; i++) {
        if (devices[i].short_addr == esp_zb_get_short_address()) {
            continue;  // Don't send to ourselves
        }
        
        if (!devices[i].has_impact_cluster) {
            continue;  // Skip if device doesn't support impact cluster
        }
        
        cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = devices[i].short_addr;
        cmd_req.zcl_basic_cmd.dst_endpoint = devices[i].endpoint;
        
        esp_zb_lock_acquire(portMAX_DELAY);
        seq = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
        esp_zb_lock_release();
        
        if (seq != 0) {
            ESP_LOGI(TAG, "Impact alert sent to router 0x%04x, seq=%d", devices[i].short_addr, seq);
            success_count++;
        } else {
            ESP_LOGW(TAG, "Failed to send impact alert to router 0x%04x", devices[i].short_addr);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Increased delay to prevent buffer exhaustion
    }

    if (success_count > 0) {
        ESP_LOGI(TAG, "Impact alert sent to %d devices", success_count);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send impact alert to any device");
        return ESP_FAIL;
    }
}

esp_err_t zb_cluster_register_impact_alert_callback(zb_impact_alert_callback_t callback)
{
    if (xSemaphoreTake(cluster_mutex, portMAX_DELAY) == pdTRUE) {
        impact_alert_cb = callback;
        xSemaphoreGive(cluster_mutex);
        ESP_LOGI(TAG, "Impact alert callback registered");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/* ============================================================================
 * Gas Alert Messages
 * ============================================================================ */

esp_err_t zb_cluster_send_gas_alert(
    bool detected,
    uint32_t voltage_mv,
    uint32_t raw_value)
{
    if (!cluster_mgr_initialized) {
        ESP_LOGW(TAG, "Cluster manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!cluster_mgr_joined) {
        ESP_LOGW(TAG, "Not joined to network, cannot send gas alert");
        return ESP_ERR_INVALID_STATE;
    }

    /* Debouncing: prevent spam */
    static int64_t last_gas_time = 0;
    int64_t now = esp_timer_get_time();
    if ((now - last_gas_time) < 5000000LL) { // 5 seconds
        ESP_LOGD(TAG, "Gas alert debounced");
        return ESP_OK;
    }
    last_gas_time = now;

    canary_gas_alert_msg_t msg;
    msg.detected = detected ? CANARY_GAS_DETECTED : CANARY_GAS_NOT_DETECTED;
    msg.voltage_mv = (uint16_t)voltage_mv;
    msg.raw_value = (uint16_t)raw_value;
    msg.timestamp = get_timestamp_ms();
    msg.source_addr = esp_zb_get_short_address();

    /* Log locally first */
    ESP_LOGW(TAG, "*** GAS ALERT (LOCAL) *** detected=%d | voltage=%dmV | raw=%d | src=0x%04x",
             detected, voltage_mv, raw_value,
             esp_zb_get_short_address());
    
    /* Get discovered devices from registry */
    zb_device_info_t devices[ZB_REGISTRY_MAX_DEVICES];
    uint8_t device_count = 0;
    zb_registry_get_devices(devices, ZB_REGISTRY_MAX_DEVICES, &device_count);
    
    ESP_LOGI(TAG, "Sending gas alert to %d discovered devices + coordinator", device_count);
    
    /* Build ZCL command */
    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req;
    memset(&cmd_req, 0, sizeof(cmd_req));
    
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.zcl_basic_cmd.dst_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    cmd_req.cluster_id = ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT;
    cmd_req.custom_cmd_id = ZB_ZCL_CMD_GAS_ALERT_REPORT_ID;
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd_req.data.type = ESP_ZB_ZCL_ATTR_TYPE_ARRAY;
    cmd_req.data.value = (void *)&msg;
    cmd_req.data.size = sizeof(msg);
    
    int success_count = 0;
    
    /* Send to coordinator (always) */
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t seq = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();
    if (seq != 0) {
        ESP_LOGI(TAG, "Gas alert sent to coordinator 0x0000, seq=%d", seq);
        success_count++;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Increased delay
    
    /* Send to each discovered device (unicast) */
    for (int i = 0; i < device_count; i++) {
        if (devices[i].short_addr == esp_zb_get_short_address()) {
            continue;  // Don't send to ourselves
        }
        
        if (!devices[i].has_gas_cluster) {
            continue;  // Skip if device doesn't support gas cluster
        }
        
        cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = devices[i].short_addr;
        cmd_req.zcl_basic_cmd.dst_endpoint = devices[i].endpoint;
        
        esp_zb_lock_acquire(portMAX_DELAY);
        seq = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
        esp_zb_lock_release();
        
        if (seq != 0) {
            ESP_LOGI(TAG, "Gas alert sent to router 0x%04x, seq=%d", devices[i].short_addr, seq);
            success_count++;
        } else {
            ESP_LOGW(TAG, "Failed to send gas alert to router 0x%04x", devices[i].short_addr);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Increased delay to prevent buffer exhaustion
    }

    if (success_count > 0) {
        ESP_LOGI(TAG, "Gas alert sent to %d devices", success_count);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send gas alert to any device");
        return ESP_FAIL;
    }
}

esp_err_t zb_cluster_register_gas_alert_callback(zb_gas_alert_callback_t callback)
{
    if (xSemaphoreTake(cluster_mutex, portMAX_DELAY) == pdTRUE) {
        gas_alert_cb = callback;
        xSemaphoreGive(cluster_mutex);
        ESP_LOGI(TAG, "Gas alert callback registered");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/* ============================================================================
 * Heartbeat Messages
 * ============================================================================ */

esp_err_t zb_cluster_send_heartbeat(uint32_t seq_num, uint32_t uptime_sec, uint8_t battery_level)
{
    if (!cluster_mgr_initialized) {
        ESP_LOGE(TAG, "Cluster manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!cluster_mgr_joined) {
        ESP_LOGD(TAG, "Not joined to network, skipping heartbeat send");
        return ESP_ERR_INVALID_STATE;
    }
    
    canary_heartbeat_msg_t msg;
    msg.seq_num = seq_num;
    msg.uptime_sec = uptime_sec;
    msg.timestamp = get_timestamp_ms();
    msg.source_addr = esp_zb_get_short_address();
    msg.battery_level = battery_level;
    
    /* Update local attributes */
    heartbeat_attrs.seq_num = msg.seq_num;
    heartbeat_attrs.uptime_sec = msg.uptime_sec;
    heartbeat_attrs.timestamp = msg.timestamp;
    heartbeat_attrs.battery_level = msg.battery_level;
    
    /* Prepare custom cluster command request */
    esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {0};
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;  // Send to coordinator
    cmd_req.zcl_basic_cmd.dst_endpoint = CANARY_ENDPOINT;
    cmd_req.zcl_basic_cmd.src_endpoint = CANARY_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    cmd_req.cluster_id = ZB_ZCL_CLUSTER_ID_CANARY_HEARTBEAT;
    cmd_req.custom_cmd_id = ZB_ZCL_CMD_HEARTBEAT_PING_ID;
    cmd_req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    cmd_req.data.type = ESP_ZB_ZCL_ATTR_TYPE_ARRAY;
    cmd_req.data.value = (void *)&msg;
    cmd_req.data.size = sizeof(msg);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t seq = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();
    
    if (seq != 0) {
        ESP_LOGD(TAG, "Heartbeat sent: seq=%u, uptime=%lu, battery=%u%%", 
                 seq_num, uptime_sec, battery_level);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to send heartbeat (seq=0)");
        return ESP_FAIL;
    }
}

esp_err_t zb_cluster_register_heartbeat_callback(zb_heartbeat_callback_t callback)
{
    if (xSemaphoreTake(cluster_mutex, portMAX_DELAY) == pdTRUE) {
        heartbeat_cb = callback;
        xSemaphoreGive(cluster_mutex);
        ESP_LOGI(TAG, "Heartbeat callback registered");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}
