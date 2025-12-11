/*
 * CanaryCap Zigbee Mesh Network Implementation
 */

#include "zb_mesh_network.h"
#include "zb_cluster_manager.h"
#include "zb_device_registry.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "zb_mesh";

/* ============================================================================
 * Internal state
 * ============================================================================ */

static bool mesh_initialized = false;
static zb_mesh_role_t device_role = ZB_MESH_ROLE_ROUTER;
static zb_mesh_state_t current_state = ZB_MESH_STATE_UNINITIALIZED;
static SemaphoreHandle_t mesh_mutex = NULL;

/* Callbacks */
static zb_mesh_state_change_cb_t state_change_cb = NULL;
static zb_mesh_device_event_cb_t device_event_cb = NULL;
static uint16_t s_last_joined_addr = 0xFFFF;

/* No periodic task needed - broadcasts work immediately with proper configuration */

uint16_t zb_mesh_get_last_joined_addr(void)
{
    return s_last_joined_addr;
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

esp_err_t zb_mesh_init(zb_mesh_role_t role)
{
    if (mesh_initialized) {
        ESP_LOGW(TAG, "Mesh already initialized");
        return ESP_OK;
    }

    device_role = role;
    current_state = ZB_MESH_STATE_UNINITIALIZED;

    /* Create mutex */
    mesh_mutex = xSemaphoreCreateMutex();
    if (!mesh_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    mesh_initialized = true;
    
    ESP_LOGI(TAG, "Mesh network initialized as %s", 
             role == ZB_MESH_ROLE_COORDINATOR ? "COORDINATOR" : "ROUTER");
    
    return ESP_OK;
}

zb_mesh_role_t zb_mesh_get_role(void)
{
    return device_role;
}

bool zb_mesh_is_coordinator(void)
{
    return device_role == ZB_MESH_ROLE_COORDINATOR;
}

zb_mesh_state_t zb_mesh_get_state(void)
{
    zb_mesh_state_t state;
    if (xSemaphoreTake(mesh_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_state;
        xSemaphoreGive(mesh_mutex);
    } else {
        state = ZB_MESH_STATE_UNINITIALIZED;
    }
    return state;
}

bool zb_mesh_is_joined(void)
{
    return (zb_mesh_get_state() == ZB_MESH_STATE_JOINED);
}

/* ============================================================================
 * State Management
 * ============================================================================ */

void zb_mesh_update_state(zb_mesh_state_t new_state)
{
    if (xSemaphoreTake(mesh_mutex, portMAX_DELAY) == pdTRUE) {
        zb_mesh_state_t old_state = current_state;
        if (old_state != new_state) {
            current_state = new_state;
            xSemaphoreGive(mesh_mutex);
            
            ESP_LOGI(TAG, "State changed: %d -> %d", old_state, new_state);
            
            /* Notify cluster manager */
            if (new_state == ZB_MESH_STATE_JOINED) {
                zb_cluster_manager_set_joined(true);
            } else if (old_state == ZB_MESH_STATE_JOINED) {
                zb_cluster_manager_set_joined(false);
            }
            
            /* Call user callback */
            if (state_change_cb) {
                state_change_cb(old_state, new_state);
            }
        } else {
            xSemaphoreGive(mesh_mutex);
        }
    }
}

/* ============================================================================
 * Network Management
 * ============================================================================ */

esp_err_t zb_mesh_form_network(void)
{
    if (!mesh_initialized) {
        ESP_LOGE(TAG, "Mesh not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (device_role != ZB_MESH_ROLE_COORDINATOR) {
        ESP_LOGE(TAG, "Only coordinator can form network");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Starting network formation");
    zb_mesh_update_state(ZB_MESH_STATE_FORMING);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
    esp_zb_lock_release();

    return err;
}

esp_err_t zb_mesh_join_network(void)
{
    if (!mesh_initialized) {
        ESP_LOGE(TAG, "Mesh not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting network steering (join)");
    zb_mesh_update_state(ZB_MESH_STATE_JOINING);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    esp_zb_lock_release();

    return err;
}

esp_err_t zb_mesh_open_network(uint8_t duration_sec)
{
    if (!mesh_initialized) {
        ESP_LOGE(TAG, "Mesh not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!zb_mesh_is_joined()) {
        ESP_LOGW(TAG, "Not joined to network");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Opening network for %d seconds", duration_sec);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_bdb_open_network(duration_sec);
    esp_zb_lock_release();

    return err;
}

esp_err_t zb_mesh_leave_network(void)
{
    if (!mesh_initialized) {
        ESP_LOGE(TAG, "Mesh not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Leaving network - performing factory reset");
    
    /* Send disconnect notification before leaving */
    if (zb_mesh_is_joined()) {
        zb_cluster_send_network_status(CANARY_NETWORK_DISCONNECTED);
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow message to send
    }
    
    zb_mesh_update_state(ZB_MESH_STATE_DISCONNECTED);
    
    /* Factory reset is the proper way to leave the network in ESP-IDF */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_factory_reset();
    esp_zb_lock_release();

    return ESP_OK;
}

esp_err_t zb_mesh_soft_leave(bool rejoin)
{
    if (!mesh_initialized) {
        ESP_LOGE(TAG, "Mesh not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Soft leave: leaving network temporarily (rejoin=%d)", rejoin);
    
    /* Send disconnect notification before leaving */
    if (zb_mesh_is_joined()) {
        ESP_LOGI(TAG, "Sending disconnect notification to network");
        zb_cluster_send_network_status(CANARY_NETWORK_DISCONNECTED);
        vTaskDelay(pdMS_TO_TICKS(500)); // Allow message to propagate
    }
    
    /* Request to leave network using ZDO management leave */
    uint16_t self_addr = esp_zb_get_short_address();
    esp_zb_ieee_addr_t self_ieee;
    esp_zb_get_long_address(self_ieee);
    
    ESP_LOGI(TAG, "Requesting leave for device 0x%04x", self_addr);
    
    esp_zb_zdo_mgmt_leave_req_param_t leave_req;
    memset(&leave_req, 0, sizeof(leave_req));
    leave_req.dst_nwk_addr = self_addr;
    memcpy(leave_req.device_address, self_ieee, sizeof(esp_zb_ieee_addr_t));
    leave_req.remove_children = false;
    leave_req.rejoin = rejoin;
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zdo_device_leave_req(&leave_req, NULL, NULL);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Soft leave request sent");
    
    return ESP_OK;
}

esp_err_t zb_mesh_factory_reset(void)
{
    ESP_LOGW(TAG, "Performing factory reset");
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_factory_reset();
    esp_zb_lock_release();

    zb_mesh_update_state(ZB_MESH_STATE_UNINITIALIZED);

    return ESP_OK;
}

/* ============================================================================
 * Network Information
 * ============================================================================ */

esp_err_t zb_mesh_get_network_info(zb_mesh_network_info_t *info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(info, 0, sizeof(zb_mesh_network_info_t));

    info->short_addr = esp_zb_get_short_address();
    info->pan_id = esp_zb_get_pan_id();
    info->channel = esp_zb_get_current_channel();
    info->is_joined = zb_mesh_is_joined();
    info->role = device_role;
    info->state = zb_mesh_get_state();
    
    esp_zb_get_long_address(info->ieee_addr);
    esp_zb_get_extended_pan_id(info->extended_pan_id);

    return ESP_OK;
}

void zb_mesh_print_network_info(void)
{
    zb_mesh_network_info_t info;
    if (zb_mesh_get_network_info(&info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get network info");
        return;
    }

    ESP_LOGI(TAG, "=== Network Information ===");
    ESP_LOGI(TAG, "Role: %s", info.role == ZB_MESH_ROLE_COORDINATOR ? "COORDINATOR" : "ROUTER");
    ESP_LOGI(TAG, "State: %d", info.state);
    ESP_LOGI(TAG, "Joined: %s", info.is_joined ? "YES" : "NO");
    ESP_LOGI(TAG, "Short Address: 0x%04x", info.short_addr);
    ESP_LOGI(TAG, "PAN ID: 0x%04x", info.pan_id);
    ESP_LOGI(TAG, "Channel: %d", info.channel);
    ESP_LOGI(TAG, "IEEE Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             info.ieee_addr[7], info.ieee_addr[6], info.ieee_addr[5], info.ieee_addr[4],
             info.ieee_addr[3], info.ieee_addr[2], info.ieee_addr[1], info.ieee_addr[0]);
    ESP_LOGI(TAG, "Extended PAN: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             info.extended_pan_id[7], info.extended_pan_id[6], info.extended_pan_id[5], 
             info.extended_pan_id[4], info.extended_pan_id[3], info.extended_pan_id[2],
             info.extended_pan_id[1], info.extended_pan_id[0]);
    ESP_LOGI(TAG, "===========================");
}

/* ============================================================================
 * Callback Registration
 * ============================================================================ */

esp_err_t zb_mesh_register_state_callback(zb_mesh_state_change_cb_t callback)
{
    if (xSemaphoreTake(mesh_mutex, portMAX_DELAY) == pdTRUE) {
        state_change_cb = callback;
        xSemaphoreGive(mesh_mutex);
        ESP_LOGI(TAG, "State callback registered");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t zb_mesh_register_device_callback(zb_mesh_device_event_cb_t callback)
{
    if (xSemaphoreTake(mesh_mutex, portMAX_DELAY) == pdTRUE) {
        device_event_cb = callback;
        xSemaphoreGive(mesh_mutex);
        ESP_LOGI(TAG, "Device callback registered");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

/* ============================================================================
 * Signal Handler
 * ============================================================================ */

void zb_mesh_handle_signal(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    ESP_LOGI(TAG, "Signal: type=0x%x status=%s", sig_type, esp_err_to_name(err_status));

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device %s in %s mode",
                     sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "started" : "rebooted",
                     esp_zb_bdb_is_factory_new() ? "factory-reset" : "configured");

            if (esp_zb_bdb_is_factory_new()) {
                /* Factory new - start formation (coordinator) or steering (router) */
                if (device_role == ZB_MESH_ROLE_COORDINATOR) {
                    zb_mesh_form_network();
                } else {
                    zb_mesh_join_network();
                }
            } else {
                /* Already configured - try to rejoin */
                ESP_LOGI(TAG, "Rejoining existing network");
                zb_mesh_update_state(ZB_MESH_STATE_REJOINING);
                if (device_role == ZB_MESH_ROLE_COORDINATOR) {
                    /* Coordinator needs to start steering to become discoverable again */
                    ESP_LOGI(TAG, "Coordinator starting steering after reboot");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    /* Router needs to start steering to rejoin the network */
                    ESP_LOGI(TAG, "Router starting steering to rejoin network");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                }
            }
        } else {
            ESP_LOGW(TAG, "Device start failed: %s", esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Network formed successfully");
            ESP_LOGI(TAG, "Extended PAN: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
            ESP_LOGI(TAG, "PAN ID: 0x%04x, Channel: %d", 
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            
            /* Start steering so coordinator can be discovered and accept joins */
            /* Note: steering will automatically open network for permit_join */
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            
            zb_mesh_update_state(ZB_MESH_STATE_JOINED);
            /* Don't send network status here - wait for steering to complete to avoid buffer exhaustion */
        } else {
            ESP_LOGW(TAG, "Formation failed: %s", esp_err_to_name(err_status));
            zb_mesh_update_state(ZB_MESH_STATE_DISCONNECTED);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            if (device_role == ZB_MESH_ROLE_COORDINATOR) {
                ESP_LOGI(TAG, "Coordinator: Network steering completed");
                zb_mesh_update_state(ZB_MESH_STATE_JOINED);
                
                /* After steering, explicitly open network for joins.
                 * Steering doesn't automatically keep network open for the full permit-join duration.
                 * This is DIFFERENT from automatic reopening (which we removed) - this is the
                 * initial opening needed for routers to join. */
                ESP_LOGW(TAG, "Coordinator: Opening network for %d seconds to allow router joins", ZB_MESH_PERMIT_JOIN_DURATION);
                esp_err_t open_result = zb_mesh_open_network(ZB_MESH_PERMIT_JOIN_DURATION);
                if (open_result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to open network: %s", esp_err_to_name(open_result));
                } else {
                    ESP_LOGW(TAG, "Network is now OPEN - routers can join for next %d seconds", ZB_MESH_PERMIT_JOIN_DURATION);
                }
            } else {
                ESP_LOGI(TAG, "Router: Network steering completed - joined successfully");
                zb_mesh_update_state(ZB_MESH_STATE_JOINED);
            }
            ESP_LOGI(TAG, "Device can now route messages");
            
            /* Device discovery: Balance between stabilization and responsiveness.
             * Don't wait too long as this blocks the main Zigbee task during critical join window. */
            ESP_LOGI(TAG, "Network joined - waiting 5s before starting device discovery...");
            vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds - enough for stabilization
            
            ESP_LOGI(TAG, "Starting device discovery...");
            zb_registry_start_discovery();
            
            zb_mesh_print_network_info();
        } else {
            ESP_LOGW(TAG, "Steering failed: %s", esp_err_to_name(err_status));
            zb_mesh_update_state(ZB_MESH_STATE_DISCONNECTED);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        {
            esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = 
                (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            
            ESP_LOGE(TAG, "*** DEVICE ANNOUNCED: 0x%04x (IEEE: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x) ***",
                     dev_annce_params->device_short_addr,
                     dev_annce_params->ieee_addr[7], dev_annce_params->ieee_addr[6],
                     dev_annce_params->ieee_addr[5], dev_annce_params->ieee_addr[4],
                     dev_annce_params->ieee_addr[3], dev_annce_params->ieee_addr[2],
                     dev_annce_params->ieee_addr[1], dev_annce_params->ieee_addr[0]);
            
            s_last_joined_addr = dev_annce_params->device_short_addr;
            
            /* Network is already open - no need to reopen.
             * Excessive open_network calls cause buffer pool exhaustion. */
            
            /* When ANY device announces (including routers), re-run discovery.
             * CRITICAL: This ensures Router A discovers Router B when Router B joins later. */
            if (dev_annce_params->device_short_addr != esp_zb_get_short_address()) {
                ESP_LOGI(TAG, "Device 0x%04x announced - will rediscover after stabilization", 
                         dev_annce_params->device_short_addr);
                
                /* Wait longer for routers to fully initialize their endpoint and clusters.
                 * Routers need time to register custom clusters before responding to match descriptors. */
                vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds - critical for router cluster initialization
                
                /* Re-run discovery to find the newly joined device.
                 * This is THE KEY to router-to-router communication! */
                ESP_LOGI(TAG, "Re-running device discovery to find new device 0x%04x", 
                         dev_annce_params->device_short_addr);
                zb_registry_force_rediscovery();  // Force restart even if already in progress
            }
            
            if (device_event_cb) {
                device_event_cb(true, dev_annce_params->device_short_addr);
            }
        }
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            uint8_t duration = *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
            if (duration > 0) {
                ESP_LOGW(TAG, "==== NETWORK OPEN FOR JOINS: %d seconds remaining ====", duration);
            } else {
                ESP_LOGW(TAG, "==== NETWORK CLOSED TO NEW JOINS ====");
                ESP_LOGI(TAG, "Existing devices will continue routing normally (RxOnWhenIdle=true)");
                /* DO NOT automatically reopen! This causes buffer exhaustion and stack dumps.
                 * Routers will continue to route messages as long as RxOnWhenIdle=true,
                 * which is already set in main.c (esp_zb_set_rx_on_when_idle).
                 * Only manually reopen network when you want to allow new devices to join. */
            }
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Device left network");
        s_last_joined_addr = 0xFFFF;
        zb_mesh_update_state(ZB_MESH_STATE_DISCONNECTED);
        zb_cluster_send_network_status(CANARY_NETWORK_DISCONNECTED);
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE_INDICATION:
        {
            /* A child device has left the network */
            esp_zb_zdo_signal_leave_indication_params_t *leave_params = 
                (esp_zb_zdo_signal_leave_indication_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            
            if (leave_params) {
                ESP_LOGE(TAG, "*** CHILD DEVICE LEFT NETWORK: 0x%04x (rejoin=%d) ***", 
                         leave_params->short_addr, leave_params->rejoin);
                
                if (leave_params->rejoin) {
                    ESP_LOGI(TAG, "Device 0x%04x left with rejoin flag - will attempt to rejoin", 
                             leave_params->short_addr);
                } else {
                    ESP_LOGW(TAG, "Device 0x%04x left without rejoin - may not return", 
                             leave_params->short_addr);
                }
                
                /* Remove device from registry */
                zb_registry_remove_device(leave_params->short_addr);
                ESP_LOGI(TAG, "Device 0x%04x removed from registry", leave_params->short_addr);
                
                /* Notify application via callback */
                if (device_event_cb) {
                    device_event_cb(false, leave_params->short_addr);
                }
            }
        }
        break;

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        {
            /* Device is unreachable - likely out of range or powered off */
            esp_zb_zdo_device_unavailable_params_t *unavail_params = 
                (esp_zb_zdo_device_unavailable_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            
            if (unavail_params) {
                ESP_LOGW(TAG, "*** DEVICE UNAVAILABLE: 0x%04x (out of range or offline) ***", 
                         unavail_params->short_addr);
                
                /* Mark device as inactive in registry but don't remove it
                 * It may come back in range and rejoin automatically */
                zb_registry_mark_inactive(unavail_params->short_addr);
                
                /* Optionally notify application */
                if (device_event_cb) {
                    ESP_LOGI(TAG, "Device 0x%04x marked as unavailable - will auto-rejoin when in range", 
                             unavail_params->short_addr);
                }
            }
        }
        break;

    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        ESP_LOGW(TAG, "*** NO ACTIVE LINKS - ALL ROUTES EXPIRED ***");
        ESP_LOGI(TAG, "This is normal if all routers are out of range");
        ESP_LOGI(TAG, "Devices will automatically rejoin when they come back in range");
        break;

    default:
        ESP_LOGD(TAG, "Unhandled signal: 0x%x", sig_type);
        break;
    }
}
