# CanaryCap Zigbee Mesh Network - Implementation Summary

## What Was Built

I've completely rebuilt the Zigbee portion of the CanaryCap firmware to use **custom Zigbee clusters** for mesh networking with proper coordinator/router role management.

## New Files Created

### 1. `zb_canary_cluster.h`
- Custom cluster definitions (IDs: 0xFC00-0xFC02)
- Data structures for three message types:
  - Network Status (connection/disconnection)
  - Impact Alerts (ADXL345 accelerometer)
  - Gas Alerts (ADC threshold detection)
- Enumerations for severity levels and impact types

### 2. `zb_cluster_manager.h` / `zb_cluster_manager.c`
- High-level API for sending/receiving cluster messages
- Automatic message broadcasting to all mesh nodes
- Callback system for receiving alerts from other devices
- Built-in debouncing (2s for impacts, 5s for gas)
- Thread-safe implementation with mutex protection

### 3. `zb_mesh_network.h` / `zb_mesh_network.c`
- Mesh network management layer
- Coordinator/Router role configuration
- Network formation and joining logic
- Automatic reconnection and retry mechanisms
- Network state tracking and callbacks
- Child device management (up to 10 children per node)

### 4. Documentation
- `ZIGBEE_MESH_NETWORK.md` - Complete technical documentation
- `ZIGBEE_QUICKSTART.md` - Quick reference for building and testing

## Modified Files

### `main.c`
- Integrated cluster manager for impact detection
- Integrated cluster manager for gas detection
- Replaced old simple Zigbee task with mesh network task
- Added callbacks for receiving remote alerts
- Updated signal handler to use mesh network handler
- Removed old `send_zigbee_event` functions

## Key Features

### ✅ Mesh Network Support
- True Zigbee mesh topology
- Messages automatically routed through network
- Supports multiple routers extending range
- Self-healing if intermediate nodes fail

### ✅ Coordinator/Router Roles
- **Coordinator**: Forms network, allows joins, acts as hub
- **Router**: Joins network, routes messages, supports children
- Role selected at compile time via `#define ZB_BUILD_AS_ROUTER`

### ✅ Three Message Types

**1. Network Status**
- Sent when joining/leaving network
- Contains: state, short address, PAN ID, channel
- Automatic broadcast on connection changes

**2. Impact Alerts**
- Triggered by ADXL345 accelerometer
- Severity levels: Minor, Major, Severe
- Impact types: Fall, Ceiling Collapse, Front/Rear/Side Impact, etc.
- Contains: magnitude, X/Y/Z axes, source address

**3. Gas Alerts**
- Triggered when ADC > 3.3V
- Contains: detection state, voltage, raw value, source
- Sent when gas detected or cleared

### ✅ Automatic Behaviors
- **Debouncing**: Prevents message spam
- **Auto-reconnect**: Retries join if connection lost
- **Periodic reopen**: Coordinator reopens network every 5 minutes
- **State tracking**: Monitors connection status continuously

### ✅ Local + Remote Alerts
- Local device triggers visual/audio alerts
- Alerts broadcast to all mesh nodes
- Remote nodes also trigger their LEDs/buzzers
- Creates network-wide alarm system

## How It Works

### Message Flow
```
Device 1 (ADXL345)          Coordinator          Device 2
     |                           |                    |
     |-- Impact Detected         |                    |
     |-- Send Impact Alert ----->|                    |
     |   (via cluster)            |                    |
     |                           |-- Broadcast ------>|
     |                           |    (mesh routing)  |
     |                           |                    |-- Receive Alert
     |                           |                    |-- Trigger Buzzer
     |                           |                    |-- Blink LED
```

### Network Topology
```
        [Coordinator]
        (Forms Network)
           /    \
          /      \
    [Router 1]  [Router 2]
    (Has ADXL)  (Has Gas)
        |
    [Router 3]
    (Remote Alert)
```

## Configuration

### Build for Coordinator
```c
// In main.c - comment out or remove:
// #define ZB_BUILD_AS_ROUTER
```

### Build for Router
```c
// In main.c - uncomment:
#define ZB_BUILD_AS_ROUTER
```

### Network Parameters
```c
// In zb_mesh_network.h
#define ZB_MESH_MAX_CHILDREN            10      // Max child devices
#define ZB_MESH_PERMIT_JOIN_DURATION    180     // Join window
#define ZB_MESH_REOPEN_INTERVAL_SEC     300     // Reopen every 5 min
```

## Usage Examples

### Send Impact Alert (in your code)
```c
zb_cluster_send_impact_alert(
    CANARY_IMPACT_SEVERE,           // severity
    CANARY_IMPACT_TYPE_FALL,        // type
    2.5f,                           // magnitude (g-force)
    0.5f, -0.3f, -2.3f             // x, y, z axes
);
```

### Receive Impact Alert (callback)
```c
void impact_received(const canary_impact_alert_msg_t *msg, uint16_t src_addr) {
    ESP_LOGW(TAG, "IMPACT from device 0x%04x: %s", 
             src_addr,
             zb_cluster_impact_type_to_string(msg->type));
    
    // Trigger local alert
    gpio_set_level(LED_GPIO1, 0);
    buzzer_beep_ms(3000, 1000);
}

// Register callback during initialization
zb_cluster_register_impact_alert_callback(impact_received);
```

## Testing

### 1. Deploy Coordinator
```bash
cd Integrate
# Ensure ZB_BUILD_AS_ROUTER is commented out in main.c
idf.py build flash monitor
```

### 2. Deploy Router(s)
```bash
cd Integrate
# Ensure ZB_BUILD_AS_ROUTER is defined in main.c
idf.py build flash monitor
```

### 3. Trigger Events
- **Impact**: Shake or tap device with ADXL345
- **Gas**: Apply >3.3V to ADC GPIO11
- **Network**: Watch connection status in logs

### 4. Observe Behavior
- All devices receive and log alerts
- LEDs activate on all nodes
- Buzzers sound on all nodes
- Network auto-recovers from disconnections

## API Reference

### Cluster Manager Functions
```c
// Initialize
esp_err_t zb_cluster_manager_init(bool is_coordinator);

// Send messages
esp_err_t zb_cluster_send_impact_alert(...);
esp_err_t zb_cluster_send_gas_alert(...);
esp_err_t zb_cluster_send_network_status(...);

// Register callbacks
esp_err_t zb_cluster_register_impact_alert_callback(...);
esp_err_t zb_cluster_register_gas_alert_callback(...);
esp_err_t zb_cluster_register_network_status_callback(...);

// Utility
const char* zb_cluster_impact_type_to_string(...);
bool zb_cluster_manager_is_ready(void);
```

### Mesh Network Functions
```c
// Initialize
esp_err_t zb_mesh_init(zb_mesh_role_t role);

// Network operations
esp_err_t zb_mesh_form_network(void);
esp_err_t zb_mesh_join_network(void);
esp_err_t zb_mesh_leave_network(void);
esp_err_t zb_mesh_open_network(uint8_t duration_sec);

// Status
bool zb_mesh_is_joined(void);
zb_mesh_state_t zb_mesh_get_state(void);
void zb_mesh_print_network_info(void);

// Callbacks
esp_err_t zb_mesh_register_state_callback(...);
esp_err_t zb_mesh_register_device_callback(...);
```

## Benefits Over Previous Implementation

### Before (Old System)
- ❌ Used simple On/Off cluster (not designed for alerts)
- ❌ No structured message format
- ❌ Limited information transmitted
- ❌ No automatic role management
- ❌ Manual network reopening
- ❌ No message type distinction

### After (New System)
- ✅ Custom clusters designed for safety alerts
- ✅ Structured messages with detailed info
- ✅ Impact severity, type, magnitude, axes transmitted
- ✅ Gas voltage, raw values, detection state transmitted
- ✅ Automatic coordinator/router management
- ✅ Auto-reopen network for late joins
- ✅ Three distinct message types with callbacks
- ✅ Debouncing and thread-safe operation
- ✅ Network state tracking and recovery

## Next Steps

### Immediate
1. Build and test on hardware
2. Deploy one coordinator + multiple routers
3. Verify alerts propagate across mesh
4. Test network recovery (power cycle nodes)

### Future Enhancements
- Add acknowledgment for critical alerts
- Implement message priority levels
- Add network health monitoring
- Support OTA firmware updates
- Add sleep mode for battery operation
- Implement binding to device groups
- Add message encryption

## Build & Deploy Commands

```bash
# Coordinator
cd Integrate
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# Router (edit main.c first to define ZB_BUILD_AS_ROUTER)
cd Integrate
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

## Success Indicators

Watch serial monitor for:
- `"Mesh network initialized as COORDINATOR/ROUTER"`
- `"Network formed successfully"` (coordinator)
- `"Network steering completed"` (router)
- `"Short Address: 0xXXXX"` (not 0xFFFF)
- `"Impact alert sent: ..."` (when triggered)
- `"IMPACT ALERT from 0xXXXX: ..."` (when received)

## Support & Documentation

- **Full docs**: `ZIGBEE_MESH_NETWORK.md`
- **Quick start**: `ZIGBEE_QUICKSTART.md`
- **Cluster defs**: `Integrate/main/zb_canary_cluster.h`
- **API**: `Integrate/main/zb_cluster_manager.h`
- **Network**: `Integrate/main/zb_mesh_network.h`

---

**Implementation Status**: ✅ Complete and ready for testing

The Zigbee mesh networking system is fully implemented with custom clusters for impact detection, gas detection, and network status monitoring. The system supports proper coordinator/router roles and automatic mesh network management.
