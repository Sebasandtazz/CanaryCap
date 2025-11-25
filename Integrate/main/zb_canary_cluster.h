/*
 * CanaryCap Custom Zigbee Cluster Definitions
 * 
 * This header defines custom Zigbee clusters for the CanaryCap safety monitoring system.
 * Three main event types are supported:
 * 1. Network Status (connection/disconnection events)
 * 2. Impact Detection (from ADXL345 accelerometer)
 * 3. Gas Detection (from ADC sensor)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_zigbee_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Custom Cluster IDs (Manufacturer-specific range: 0xFC00 - 0xFFFF)
 * ============================================================================ */
#define ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS    0xFC00
#define ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT      0xFC01
#define ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT         0xFC02

/* Custom manufacturer code for CanaryCap */
#define CANARY_MANUFACTURER_CODE                   0x1234

/* Endpoint for CanaryCap custom clusters */
#define CANARY_ENDPOINT                            10

/* ============================================================================
 * Attribute IDs for each custom cluster
 * ============================================================================ */

/* Network Status Cluster Attributes */
#define ZB_ZCL_ATTR_NETWORK_STATUS_STATE_ID        0x0000  // uint8: 0=disconnected, 1=connected
#define ZB_ZCL_ATTR_NETWORK_STATUS_SHORT_ADDR_ID   0x0001  // uint16: device short address
#define ZB_ZCL_ATTR_NETWORK_STATUS_TIMESTAMP_ID    0x0002  // uint32: timestamp of status change

/* Impact Alert Cluster Attributes */
#define ZB_ZCL_ATTR_IMPACT_ALERT_SEVERITY_ID       0x0000  // uint8: 0=none, 1=minor, 2=major, 3=severe
#define ZB_ZCL_ATTR_IMPACT_ALERT_TYPE_ID           0x0001  // uint8: impact type enum
#define ZB_ZCL_ATTR_IMPACT_ALERT_MAGNITUDE_ID      0x0002  // int16: magnitude * 1000 (for precision)
#define ZB_ZCL_ATTR_IMPACT_ALERT_X_AXIS_ID         0x0003  // int16: X-axis * 1000
#define ZB_ZCL_ATTR_IMPACT_ALERT_Y_AXIS_ID         0x0004  // int16: Y-axis * 1000
#define ZB_ZCL_ATTR_IMPACT_ALERT_Z_AXIS_ID         0x0005  // int16: Z-axis * 1000
#define ZB_ZCL_ATTR_IMPACT_ALERT_TIMESTAMP_ID      0x0006  // uint32: timestamp of impact

/* Gas Alert Cluster Attributes */
#define ZB_ZCL_ATTR_GAS_ALERT_DETECTED_ID          0x0000  // uint8: 0=no gas, 1=gas detected
#define ZB_ZCL_ATTR_GAS_ALERT_VOLTAGE_ID           0x0001  // uint16: voltage in mV
#define ZB_ZCL_ATTR_GAS_ALERT_RAW_VALUE_ID         0x0002  // uint16: raw ADC value
#define ZB_ZCL_ATTR_GAS_ALERT_TIMESTAMP_ID         0x0003  // uint32: timestamp of detection

/* ============================================================================
 * Custom Command IDs for each cluster
 * ============================================================================ */

/* Network Status Commands */
#define ZB_ZCL_CMD_NETWORK_STATUS_REPORT_ID        0x00
#define ZB_ZCL_CMD_NETWORK_STATUS_QUERY_ID         0x01

/* Impact Alert Commands */
#define ZB_ZCL_CMD_IMPACT_ALERT_REPORT_ID          0x00
#define ZB_ZCL_CMD_IMPACT_ALERT_ACK_ID             0x01

/* Gas Alert Commands */
#define ZB_ZCL_CMD_GAS_ALERT_REPORT_ID             0x00
#define ZB_ZCL_CMD_GAS_ALERT_ACK_ID                0x01

/* ============================================================================
 * Enumerations for attribute values
 * ============================================================================ */

/* Network Status States */
typedef enum {
    CANARY_NETWORK_DISCONNECTED = 0,
    CANARY_NETWORK_CONNECTED = 1,
    CANARY_NETWORK_REJOINING = 2
} canary_network_state_t;

/* Impact Severity Levels */
typedef enum {
    CANARY_IMPACT_NONE = 0,
    CANARY_IMPACT_MINOR = 1,
    CANARY_IMPACT_MAJOR = 2,
    CANARY_IMPACT_SEVERE = 3
} canary_impact_severity_t;

/* Impact Types */
typedef enum {
    CANARY_IMPACT_TYPE_UNKNOWN = 0,
    CANARY_IMPACT_TYPE_FALL = 1,
    CANARY_IMPACT_TYPE_CEILING_COLLAPSE = 2,
    CANARY_IMPACT_TYPE_HARD_LANDING = 3,
    CANARY_IMPACT_TYPE_FRONT = 4,
    CANARY_IMPACT_TYPE_REAR = 5,
    CANARY_IMPACT_TYPE_RIGHT_SIDE = 6,
    CANARY_IMPACT_TYPE_LEFT_SIDE = 7,
    CANARY_IMPACT_TYPE_BLUNT = 8
} canary_impact_type_t;

/* Gas Detection States */
typedef enum {
    CANARY_GAS_NOT_DETECTED = 0,
    CANARY_GAS_DETECTED = 1
} canary_gas_state_t;

/* ============================================================================
 * Data structures for cluster messages
 * ============================================================================ */

/* Network Status Report Message */
typedef struct {
    uint8_t state;              // canary_network_state_t
    uint16_t short_addr;        // device short address
    uint32_t timestamp;         // timestamp of event
    uint8_t pan_id_high;        // PAN ID high byte
    uint8_t pan_id_low;         // PAN ID low byte
    uint8_t channel;            // current Zigbee channel
} __attribute__((packed)) canary_network_status_msg_t;

/* Impact Alert Report Message */
typedef struct {
    uint8_t severity;           // canary_impact_severity_t
    uint8_t type;               // canary_impact_type_t
    int16_t magnitude;          // magnitude * 1000
    int16_t x_axis;             // X-axis * 1000
    int16_t y_axis;             // Y-axis * 1000
    int16_t z_axis;             // Z-axis * 1000
    uint32_t timestamp;         // timestamp of impact
    uint16_t source_addr;       // short address of reporting device
} __attribute__((packed)) canary_impact_alert_msg_t;

/* Gas Alert Report Message */
typedef struct {
    uint8_t detected;           // canary_gas_state_t
    uint16_t voltage_mv;        // voltage in millivolts
    uint16_t raw_value;         // raw ADC value
    uint32_t timestamp;         // timestamp of detection
    uint16_t source_addr;       // short address of reporting device
} __attribute__((packed)) canary_gas_alert_msg_t;

/* ============================================================================
 * Helper macros for attribute list creation
 * ============================================================================ */

/* Network Status Cluster Attribute List */
#define ZB_ZCL_DECLARE_CANARY_NETWORK_STATUS_ATTRIB_LIST(attr_list_name, \
    state_attr, short_addr_attr, timestamp_attr) \
    ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list_name, CANARY_NETWORK_STATUS) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_NETWORK_STATUS_STATE_ID, (state_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_NETWORK_STATUS_SHORT_ADDR_ID, (short_addr_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_NETWORK_STATUS_TIMESTAMP_ID, (timestamp_attr)) \
    ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

/* Impact Alert Cluster Attribute List */
#define ZB_ZCL_DECLARE_CANARY_IMPACT_ALERT_ATTRIB_LIST(attr_list_name, \
    severity_attr, type_attr, magnitude_attr, x_attr, y_attr, z_attr, timestamp_attr) \
    ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list_name, CANARY_IMPACT_ALERT) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_SEVERITY_ID, (severity_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_TYPE_ID, (type_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_MAGNITUDE_ID, (magnitude_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_X_AXIS_ID, (x_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_Y_AXIS_ID, (y_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_Z_AXIS_ID, (z_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IMPACT_ALERT_TIMESTAMP_ID, (timestamp_attr)) \
    ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

/* Gas Alert Cluster Attribute List */
#define ZB_ZCL_DECLARE_CANARY_GAS_ALERT_ATTRIB_LIST(attr_list_name, \
    detected_attr, voltage_attr, raw_attr, timestamp_attr) \
    ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list_name, CANARY_GAS_ALERT) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_GAS_ALERT_DETECTED_ID, (detected_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_GAS_ALERT_VOLTAGE_ID, (voltage_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_GAS_ALERT_RAW_VALUE_ID, (raw_attr)) \
    ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_GAS_ALERT_TIMESTAMP_ID, (timestamp_attr)) \
    ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

#ifdef __cplusplus
}
#endif
