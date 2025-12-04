/*
 * CanaryCap Zigbee Mesh Network Configuration
 * 
 * Provides enhanced mesh networking capabilities with proper coordinator/router
 * role management, network formation, and child device support.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_zigbee_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

/* Maximum number of child devices a router/coordinator can support */
#define ZB_MESH_MAX_CHILDREN            10

/* Network security settings */
#define ZB_MESH_INSTALL_CODE_POLICY     false

/* Network parameters */
#define ZB_MESH_PRIMARY_CHANNEL_MASK    ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define ZB_MESH_PERMIT_JOIN_DURATION    240  // seconds - longer window for joins

/* Periodic network reopen interval for coordinator (seconds) */
#define ZB_MESH_REOPEN_INTERVAL_SEC     60   // reopen every 60 seconds for frequent discovery

/* Commissioning retry interval when not joined (seconds) */
#define ZB_MESH_COMMISSION_RETRY_SEC    30

/* Device role */
typedef enum {
    ZB_MESH_ROLE_COORDINATOR,
    ZB_MESH_ROLE_ROUTER
} zb_mesh_role_t;

/* Network state */
typedef enum {
    ZB_MESH_STATE_UNINITIALIZED,
    ZB_MESH_STATE_FORMING,
    ZB_MESH_STATE_JOINING,
    ZB_MESH_STATE_JOINED,
    ZB_MESH_STATE_DISCONNECTED,
    ZB_MESH_STATE_REJOINING
} zb_mesh_state_t;

/* ============================================================================
 * Initialization and Configuration
 * ============================================================================ */

/**
 * @brief Initialize the Zigbee mesh network
 * 
 * This configures the device role and prepares the Zigbee stack.
 * Must be called before starting the Zigbee task.
 * 
 * @param role Device role (coordinator or router)
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_init(zb_mesh_role_t role);

/**
 * @brief Get the configured device role
 * 
 * @return Current device role
 */
zb_mesh_role_t zb_mesh_get_role(void);

/**
 * @brief Check if device is coordinator
 * 
 * @return true if coordinator, false if router
 */
bool zb_mesh_is_coordinator(void);

/**
 * @brief Get current network state
 * 
 * @return Current mesh network state
 */
zb_mesh_state_t zb_mesh_get_state(void);

/**
 * @brief Check if device is currently joined to network
 * 
 * @return true if joined, false otherwise
 */
bool zb_mesh_is_joined(void);

/* ============================================================================
 * Network Management
 * ============================================================================ */

/**
 * @brief Start network formation (coordinator only)
 * 
 * Initiates the process of creating a new Zigbee network.
 * Only valid for coordinator devices.
 * 
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_form_network(void);

/**
 * @brief Start network steering (join process for routers)
 * 
 * Initiates the process of joining an existing Zigbee network.
 * Valid for router devices.
 * 
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_join_network(void);

/**
 * @brief Open network to allow new devices to join
 * 
 * For coordinators: opens the network for a specified duration
 * For routers: this is typically automatic
 * 
 * @param duration_sec Duration in seconds to allow joins (0 = close network)
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_open_network(uint8_t duration_sec);

/**
 * @brief Leave the current network
 * 
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_leave_network(void);

/**
 * @brief Perform a factory reset
 * 
 * Clears all network settings and returns device to factory state.
 * 
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_factory_reset(void);

/* ============================================================================
 * Network Information
 * ============================================================================ */

/**
 * @brief Get network information structure
 */
typedef struct {
    uint16_t short_addr;
    uint16_t pan_id;
    uint8_t channel;
    uint8_t depth;
    bool is_joined;
    zb_mesh_role_t role;
    zb_mesh_state_t state;
    esp_zb_ieee_addr_t ieee_addr;
    esp_zb_ieee_addr_t extended_pan_id;
    uint8_t num_children;
} zb_mesh_network_info_t;

/**
 * @brief Get current network information
 * 
 * @param info Pointer to structure to fill with network info
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_get_network_info(zb_mesh_network_info_t *info);

/**
 * @brief Print network information to console
 */
void zb_mesh_print_network_info(void);

/* ============================================================================
 * Callbacks for network events
 * ============================================================================ */

/**
 * @brief Callback for network state changes
 * 
 * @param old_state Previous state
 * @param new_state Current state
 */
typedef void (*zb_mesh_state_change_cb_t)(zb_mesh_state_t old_state, zb_mesh_state_t new_state);

/**
 * @brief Register callback for network state changes
 * 
 * @param callback Function to call on state change
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_register_state_callback(zb_mesh_state_change_cb_t callback);

/**
 * @brief Callback for device join/leave events
 * 
 * @param joined true if device joined, false if left
 * @param device_addr Short address of the device
 */
typedef void (*zb_mesh_device_event_cb_t)(bool joined, uint16_t device_addr);

/**
 * @brief Register callback for child device join/leave events
 * 
 * @param callback Function to call when child joins/leaves
 * @return ESP_OK on success
 */
esp_err_t zb_mesh_register_device_callback(zb_mesh_device_event_cb_t callback);

/* ============================================================================
 * Internal functions (called by Zigbee signal handler)
 * ============================================================================ */

/**
 * @brief Update mesh state (called internally by signal handler)
 * 
 * @param new_state New network state
 */
void zb_mesh_update_state(zb_mesh_state_t new_state);

/**
 * @brief Handle Zigbee app signals (called from esp_zb_app_signal_handler)
 * 
 * @param signal_struct Zigbee signal structure
 */
void zb_mesh_handle_signal(esp_zb_app_signal_t *signal_struct);

/**
 * @brief Get the short address of the last joined device (router)
 * 
 * Returns 0xFFFF if none is known.
 */
uint16_t zb_mesh_get_last_joined_addr(void);

#ifdef __cplusplus
}
#endif
