/*
 * CanaryCap Zigbee Device Registry
 * 
 * Manages discovered Zigbee devices for peer-to-peer router communication.
 * Based on ESP-IDF Zigbee examples: uses ZDO discovery and binding.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "esp_zb_switch.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of devices to track */
#define ZB_REGISTRY_MAX_DEVICES 10

/* Device information structure */
typedef struct {
    uint16_t short_addr;                    // Network short address
    uint8_t endpoint;                       // Endpoint ID
    esp_zb_ieee_addr_t ieee_addr;          // IEEE 64-bit address
    bool is_bound;                          // Whether binding is established
    bool has_impact_cluster;                // Supports impact alerts
    bool has_gas_cluster;                   // Supports gas alerts
} zb_device_info_t;

/**
 * @brief Initialize the device registry
 * 
 * @return ESP_OK on success
 */
esp_err_t zb_registry_init(void);

/**
 * @brief Start device discovery process
 * 
 * Initiates ZDO match descriptor search for devices with custom clusters.
 * This should be called after successfully joining the network.
 * 
 * @return ESP_OK on success
 */
esp_err_t zb_registry_start_discovery(void);

/**
 * @brief Add or update a device in the registry
 * 
 * @param short_addr Network short address
 * @param endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_registry_add_device(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Update device IEEE address
 * 
 * @param short_addr Network short address
 * @param ieee_addr IEEE 64-bit address
 * @return ESP_OK on success
 */
esp_err_t zb_registry_update_ieee_addr(uint16_t short_addr, const esp_zb_ieee_addr_t ieee_addr);

/**
 * @brief Mark device as bound
 * 
 * @param short_addr Network short address
 * @return ESP_OK on success
 */
esp_err_t zb_registry_mark_bound(uint16_t short_addr);

/**
 * @brief Get list of all registered devices
 * 
 * @param devices Array to fill with device info
 * @param max_devices Maximum number of devices to return
 * @param count Output: actual number of devices returned
 * @return ESP_OK on success
 */
esp_err_t zb_registry_get_devices(zb_device_info_t *devices, uint8_t max_devices, uint8_t *count);

/**
 * @brief Get a specific device by short address
 * 
 * @param short_addr Network short address to find
 * @param device Output: device information
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t zb_registry_get_device(uint16_t short_addr, zb_device_info_t *device);

/**
 * @brief Remove a device from registry
 * 
 * @param short_addr Network short address
 * @return ESP_OK on success
 */
esp_err_t zb_registry_remove_device(uint16_t short_addr);

/**
 * @brief Clear all devices from registry
 */
void zb_registry_clear(void);

/**
 * @brief Get count of registered devices
 * 
 * @return Number of devices in registry
 */
uint8_t zb_registry_get_count(void);

#ifdef __cplusplus
}
#endif
