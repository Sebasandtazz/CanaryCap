/*
 * CanaryCap Zigbee Cluster Manager
 * 
 * This module manages the custom Zigbee clusters for the CanaryCap safety system.
 * Provides high-level API for sending and receiving cluster messages across the mesh network.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "zb_canary_cluster.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Initialization and configuration
 * ============================================================================ */

/**
 * @brief Initialize the Zigbee cluster manager
 * 
 * This must be called after Zigbee stack initialization but before starting
 * to send cluster messages. It registers the custom clusters and sets up
 * message handlers.
 * 
 * @param is_coordinator true if this device is a coordinator, false for router
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t zb_cluster_manager_init(bool is_coordinator);

/**
 * @brief Check if the cluster manager is ready to send messages
 * 
 * @return true if connected to network and ready, false otherwise
 */
bool zb_cluster_manager_is_ready(void);

/**
 * @brief Set the network joined state
 * 
 * @param joined true when joined to network, false when disconnected
 */
void zb_cluster_manager_set_joined(bool joined);

/* ============================================================================
 * Network Status Messages
 * ============================================================================ */

/**
 * @brief Send a network status report
 * 
 * Broadcasts the device's network connection status to all nodes in the mesh.
 * Use this when joining, leaving, or experiencing network issues.
 * 
 * @param state Network state (connected/disconnected/rejoining)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t zb_cluster_send_network_status(canary_network_state_t state);

/**
 * @brief Callback type for network status messages received from other nodes
 * 
 * @param msg Pointer to the received network status message
 * @param src_addr Short address of the device that sent the message
 */
typedef void (*zb_network_status_callback_t)(const canary_network_status_msg_t *msg, uint16_t src_addr);

/**
 * @brief Register a callback for network status messages
 * 
 * @param callback Function to call when network status message is received
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_register_network_status_callback(zb_network_status_callback_t callback);

/* ============================================================================
 * Impact Alert Messages
 * ============================================================================ */

/**
 * @brief Send an impact alert report
 * 
 * Broadcasts an impact detection event to all nodes in the mesh network.
 * 
 * @param severity Impact severity level
 * @param type Type of impact detected
 * @param magnitude Total magnitude of impact (in G-force)
 * @param x_g X-axis acceleration (in G-force)
 * @param y_g Y-axis acceleration (in G-force)
 * @param z_g Z-axis acceleration (in G-force)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t zb_cluster_send_impact_alert(
    canary_impact_severity_t severity,
    canary_impact_type_t type,
    float magnitude,
    float x_g,
    float y_g,
    float z_g
);

/**
 * @brief Callback type for impact alert messages received from other nodes
 * 
 * @param msg Pointer to the received impact alert message
 * @param src_addr Short address of the device that sent the message
 */
typedef void (*zb_impact_alert_callback_t)(const canary_impact_alert_msg_t *msg, uint16_t src_addr);

/**
 * @brief Register a callback for impact alert messages
 * 
 * @param callback Function to call when impact alert is received
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_register_impact_alert_callback(zb_impact_alert_callback_t callback);

/* ============================================================================
 * Gas Alert Messages
 * ============================================================================ */

/**
 * @brief Send a gas alert report
 * 
 * Broadcasts a gas detection event to all nodes in the mesh network.
 * 
 * @param detected Whether gas was detected (true) or cleared (false)
 * @param voltage_mv ADC voltage reading in millivolts
 * @param raw_value Raw ADC value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t zb_cluster_send_gas_alert(
    bool detected,
    uint32_t voltage_mv,
    uint32_t raw_value
);

/**
 * @brief Callback type for gas alert messages received from other nodes
 * 
 * @param msg Pointer to the received gas alert message
 * @param src_addr Short address of the device that sent the message
 */
typedef void (*zb_gas_alert_callback_t)(const canary_gas_alert_msg_t *msg, uint16_t src_addr);

/**
 * @brief Register a callback for gas alert messages
 * 
 * @param callback Function to call when gas alert is received
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_register_gas_alert_callback(zb_gas_alert_callback_t callback);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Get string representation of impact type
 * 
 * @param type Impact type enum value
 * @return String description of the impact type
 */
const char* zb_cluster_impact_type_to_string(canary_impact_type_t type);

/**
 * @brief Get string representation of network state
 * 
 * @param state Network state enum value
 * @return String description of the network state
 */
const char* zb_cluster_network_state_to_string(canary_network_state_t state);

/**
 * @brief Get string representation of impact severity
 * 
 * @param severity Impact severity enum value
 * @return String description of the severity
 */
const char* zb_cluster_impact_severity_to_string(canary_impact_severity_t severity);

/* ============================================================================
 * Heartbeat Messages
 * ============================================================================ */

/**
 * @brief Send a heartbeat message
 * 
 * Broadcasts a periodic "I'm alive" message to all nodes in the mesh network.
 * This allows coordinators to track device presence via timeout detection.
 * 
 * @param seq_num Heartbeat sequence number (incrementing)
 * @param uptime_sec Device uptime in seconds
 * @param battery_level Battery level percentage (0-100, use 255 if N/A)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t zb_cluster_send_heartbeat(uint32_t seq_num, uint32_t uptime_sec, uint8_t battery_level);

/**
 * @brief Callback type for heartbeat messages received from other nodes
 * 
 * @param msg Pointer to the received heartbeat message
 * @param src_addr Short address of the device that sent the message
 */
typedef void (*zb_heartbeat_callback_t)(const canary_heartbeat_msg_t *msg, uint16_t src_addr);

/**
 * @brief Register a callback for heartbeat messages
 * 
 * @param callback Function to call when heartbeat is received
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_register_heartbeat_callback(zb_heartbeat_callback_t callback);

#ifdef __cplusplus
}
#endif
