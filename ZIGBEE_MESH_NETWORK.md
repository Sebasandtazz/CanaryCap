# CanaryCap Zigbee Mesh Network Implementation

## Overview

This implementation provides a complete Zigbee mesh networking solution for the CanaryCap safety monitoring system. The system uses custom Zigbee clusters to transmit three types of critical safety events across a mesh network:

1. **Network Status** - Connection/disconnection events
2. **Impact Detection** - ADXL345 accelerometer alerts
3. **Gas Detection** - ADC sensor threshold alerts

## Architecture

### Module Structure

```
Integrate/main/
├── zb_canary_cluster.h         # Custom cluster definitions
├── zb_cluster_manager.h/c      # Cluster message handling
├── zb_mesh_network.h/c         # Mesh network management
└── main.c                      # Application integration
```

### Key Components

#### 1. Custom Zigbee Clusters (`zb_canary_cluster.h`)

Defines three manufacturer-specific clusters:
- **Network Status Cluster (0xFC00)**: Device connection state
- **Impact Alert Cluster (0xFC01)**: Accelerometer impact events
- **Gas Alert Cluster (0xFC02)**: Gas sensor threshold events

#### 2. Cluster Manager (`zb_cluster_manager.h/c`)

Provides high-level API for:
- Sending cluster messages (broadcast to all mesh nodes)
- Receiving and parsing cluster messages
- Callback registration for incoming alerts
- Automatic debouncing to prevent message spam

#### 3. Mesh Network Manager (`zb_mesh_network.h/c`)

Handles:
- Coordinator/Router role configuration
- Network formation and joining
- Automatic reconnection and retry logic
- Child device management
- Network state tracking

## Device Roles

### Coordinator
- Forms the Zigbee network
- Periodically reopens network to allow new devices to join
- Acts as network hub (can route messages)
- **Build with**: `#define ZB_BUILD_AS_ROUTER` commented out (default)

### Router
- Joins existing network
- Routes messages between devices
- Can have child end devices
- **Build with**: `#define ZB_BUILD_AS_ROUTER` uncommented

## Message Types

### Network Status Messages

Sent when:
- Device joins network
- Device disconnects from network
- Device is rejoining

Contains:
- Connection state (connected/disconnected/rejoining)
- Device short address
- PAN ID and channel
- Timestamp

### Impact Alert Messages

Sent when:
- ADXL345 detects impact above threshold
- Severity: Minor, Major, or Severe
- Type: Fall, Ceiling Collapse, Hard Landing, Front/Rear/Side Impact, etc.

Contains:
- Impact severity level
- Impact type classification
- Magnitude (in G-force)
- X, Y, Z axis values
- Source device address
- Timestamp

### Gas Alert Messages

Sent when:
- ADC voltage exceeds 3.3V threshold
- Gas is detected or cleared

Contains:
- Detection state (detected/not detected)
- Voltage reading in mV
- Raw ADC value
- Source device address
- Timestamp

## Configuration

### Build Configuration

In `main.c`:
```c
// For COORDINATOR build (default):
//#define ZB_BUILD_AS_ROUTER

// For ROUTER build:
#define ZB_BUILD_AS_ROUTER
```

### Network Parameters

In `zb_mesh_network.h`:
```c
#define ZB_MESH_MAX_CHILDREN            10      // Max child devices
#define ZB_MESH_PERMIT_JOIN_DURATION    180     // Join window (seconds)
#define ZB_MESH_REOPEN_INTERVAL_SEC     300     // Reopen period (coordinator)
#define ZB_MESH_COMMISSION_RETRY_SEC    30      // Retry interval
```

### Cluster IDs

In `zb_canary_cluster.h`:
```c
#define ZB_ZCL_CLUSTER_ID_CANARY_NETWORK_STATUS    0xFC00
#define ZB_ZCL_CLUSTER_ID_CANARY_IMPACT_ALERT      0xFC01
#define ZB_ZCL_CLUSTER_ID_CANARY_GAS_ALERT         0xFC02
#define CANARY_MANUFACTURER_CODE                   0x1234
#define CANARY_ENDPOINT                            10
```

## Usage Example

### Sending Messages

```c
// Send impact alert
zb_cluster_send_impact_alert(
    CANARY_IMPACT_SEVERE,           // severity
    CANARY_IMPACT_TYPE_FALL,        // type
    2.5f,                           // magnitude (G-force)
    0.5f, -0.3f, -2.3f             // x, y, z axes
);

// Send gas alert
zb_cluster_send_gas_alert(
    true,                           // detected
    3500,                           // voltage_mv
    4095                            // raw ADC value
);

// Send network status
zb_cluster_send_network_status(
    CANARY_NETWORK_CONNECTED        // state
);
```

### Receiving Messages

Register callbacks in your application:

```c
// Called when impact alert received from another device
void impact_received(const canary_impact_alert_msg_t *msg, uint16_t src_addr) {
    float magnitude = msg->magnitude / 1000.0f;
    ESP_LOGW(TAG, "IMPACT from 0x%04x: %s (%.2fg)",
             src_addr,
             zb_cluster_impact_type_to_string(msg->type),
             magnitude);
    
    // Trigger local alert (LED, buzzer, etc.)
    trigger_local_alert();
}

// Register callback
zb_cluster_register_impact_alert_callback(impact_received);
```

## Network Behavior

### Coordinator Startup
1. Forms new network (if factory new)
2. Opens network for joins (180 seconds by default)
3. Periodically reopens network every 5 minutes
4. Broadcasts network status when joined

### Router Startup
1. Searches for coordinator
2. Joins network via steering
3. Retries join every 30 seconds if failed
4. Broadcasts network status when joined

### Message Flow
```
Device 1 (Router)           Coordinator          Device 2 (Router)
     |                           |                       |
     |--- Impact Detected ------>|                       |
     |                           |------- Broadcast ---->|
     |                           |                       |--- Local Alert
     |<------ ACK (optional) ----|                       |
```

## Mesh Network Topology

The system supports a true mesh topology where:
- **Coordinator** forms the network and acts as the central hub
- **Routers** can join the coordinator and relay messages
- **End Devices** (if implemented) can sleep and communicate through parents
- Messages are automatically routed through the mesh to reach all nodes

Example topology:
```
        [Coordinator]
           /    \
          /      \
    [Router 1]  [Router 2]
        |           |
    [Router 3]  [Router 4]
```

## Signal Handler Integration

The `esp_zb_app_signal_handler` in `main.c` delegates all Zigbee stack signals to the mesh network handler:

```c
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    zb_mesh_handle_signal(signal_struct);
}
```

This handles:
- Network formation/steering
- Device announcements
- Join/leave events
- Network state transitions

## Automatic Features

### Debouncing
- Impact alerts: 2 second minimum interval
- Gas alerts: 5 second minimum interval
- Prevents message flooding

### Auto-Reconnect
- Automatically retries join/formation if disconnected
- Configurable retry interval (30 seconds default)
- Sends disconnect notification when leaving

### Periodic Network Reopen (Coordinator Only)
- Reopens network every 5 minutes by default
- Allows late-joining devices to connect
- Configurable duration and interval

## Local Indicators

When receiving remote alerts:

**Impact Alert (Major/Severe)**
- LED_GPIO1 turns on
- Buzzer sounds at 3kHz for 1 second

**Gas Alert**
- LED_GPIO2 turns on
- Buzzer sounds at 2.5kHz for 1.5 seconds

**Network Joined**
- LED_GPIO2 brief flash
- Short beep at 2kHz

**Network Disconnected**
- LED_GPIO2 blinks continuously
- Beep at 2kHz for 500ms

## Debugging

### Enable Verbose Logging

In sdkconfig or menuconfig:
```
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
CONFIG_ESP_ZB_LOG_LEVEL_DEBUG=y
```

### Network Info

Call `zb_mesh_print_network_info()` to display:
- Device role
- Short address
- PAN ID
- Channel
- IEEE address
- Connection state

### Monitor Cluster Messages

Watch for log tags:
- `zb_cluster_mgr`: Cluster message send/receive
- `zb_mesh`: Network state changes
- `ESP_ZB_ON_OFF_SWITCH`: Zigbee stack events

## Troubleshooting

### Devices Not Joining
1. Verify coordinator has formed network successfully
2. Check network is open for joins (permit join status)
3. Ensure routers are on same channel
4. Try factory reset: `zb_mesh_factory_reset()`

### Messages Not Received
1. Verify both devices are joined (check short address != 0xFFFF)
2. Check cluster callbacks are registered
3. Ensure debounce intervals haven't prevented send
4. Monitor Zigbee signal handler for errors

### Network Instability
1. Check for interference on Zigbee channel
2. Verify adequate power supply for RF
3. Increase max children if network is large
4. Check physical distance between nodes

## Future Enhancements

Potential additions:
- Acknowledgment mechanism for critical alerts
- Message priority levels
- Network health monitoring
- OTA firmware updates over mesh
- Sleep mode for battery-powered end devices
- Binding to specific device groups
- Encrypted custom cluster data

## References

- ESP-IDF Zigbee SDK: https://github.com/espressif/esp-zigbee-sdk
- Zigbee Cluster Library Specification
- ESP32-C6 Zigbee Documentation
- CanaryCap Hardware Documentation

---

**Note**: This implementation uses manufacturer-specific cluster IDs (0xFC00-0xFFFF range) which are reserved for custom applications. For production, consider registering official cluster IDs with the Zigbee Alliance.
