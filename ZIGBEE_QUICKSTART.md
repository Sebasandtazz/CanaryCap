# Quick Start Guide - Zigbee Mesh Network

## Building the Firmware

### Build as Coordinator

1. Open `Integrate/main/main.c`
2. Comment out or remove the `ZB_BUILD_AS_ROUTER` definition:
```c
// #define ZB_BUILD_AS_ROUTER  // Commented = Coordinator
```

3. Build and flash:
```bash
cd Integrate
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Build as Router

1. Open `Integrate/main/main.c`
2. Ensure `ZB_BUILD_AS_ROUTER` is defined:
```c
#define ZB_BUILD_AS_ROUTER  // Uncommented = Router
```

3. Build and flash:
```bash
cd Integrate
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Typical Deployment

### Step 1: Deploy Coordinator
- Flash one device as Coordinator
- Power on and wait for network formation
- Look for log message: `"Network formed successfully"`
- Note the PAN ID and channel in logs

### Step 2: Deploy Routers
- Flash additional devices as Routers
- Power on near the Coordinator
- Wait for join success: `"Network steering completed"`
- Verify short address assigned (not 0xFFFF)

### Step 3: Test Messaging
- Trigger an impact on one device
- Watch logs on other devices for received alerts
- Verify LEDs and buzzers activate on all nodes

## Log Messages to Watch

### Coordinator Startup
```
I (xxx) zb_mesh: Mesh network initialized as COORDINATOR
I (xxx) zb_mesh: Starting network formation
I (xxx) zb_mesh: Network formed successfully
I (xxx) zb_mesh: Short Address: 0x0000
I (xxx) zb_mesh: PAN ID: 0xXXXX
I (xxx) zb_mesh: Channel: XX
```

### Router Startup
```
I (xxx) zb_mesh: Mesh network initialized as ROUTER
I (xxx) zb_mesh: Starting network steering (join)
I (xxx) zb_mesh: Network steering completed
I (xxx) zb_mesh: Short Address: 0xXXXX (not 0xFFFF)
```

### Impact Alert Sent
```
W (xxx) zb_cluster_mgr: Impact alert sent: Fall (severity=Severe, mag=2.50g)
```

### Impact Alert Received
```
W (xxx) zb_cluster_mgr: IMPACT ALERT from 0xXXXX: Fall severity=Severe mag=2.50g
W (xxx) ESP_ZB_ON_OFF_SWITCH: IMPACT ALERT from 0xXXXX: Fall (severity=Severe, mag=2.50g)
```

### Gas Alert Sent
```
W (xxx) zb_cluster_mgr: Gas alert sent: detected=1 voltage=3500mV
```

### Gas Alert Received
```
W (xxx) zb_cluster_mgr: GAS ALERT from 0xXXXX: detected=1 voltage=3500mV
```

## Network Commands

These functions are available in your code:

```c
// Check if joined to network
if (zb_mesh_is_joined()) {
    // Device is connected
}

// Get network information
zb_mesh_network_info_t info;
zb_mesh_get_network_info(&info);

// Print detailed network status
zb_mesh_print_network_info();

// Leave network
zb_mesh_leave_network();

// Factory reset (clears all settings)
zb_mesh_factory_reset();

// Manually open network for joins (coordinator)
zb_mesh_open_network(180); // seconds
```

## Testing Impact Detection

### Simulate Impact
Physically tap or shake the device with ADXL345 accelerometer to trigger:

- **Minor Impact**: Gentle tap (~0.5g)
- **Major Impact**: Moderate hit (~1.0g)
- **Severe Impact**: Hard shake or drop (>2.0g)

### Expected Behavior
1. Local device logs impact type and magnitude
2. Impact sent via Zigbee mesh to all nodes
3. All nodes receive message and log it
4. LEDs activate on receiving nodes
5. Buzzers sound on receiving nodes (for major/severe)

## Testing Gas Detection

### Simulate Gas
Apply voltage to ADC channel 6 (GPIO11):

- **Below threshold**: 0-3.2V = no alert
- **Above threshold**: >3.3V = gas alert triggered

### Expected Behavior
1. Local device logs voltage and raw value
2. Gas alert sent via Zigbee mesh
3. All nodes receive alert
4. LED_GPIO2 activates on receiving nodes
5. Buzzer sounds on receiving nodes

## Troubleshooting Quick Fixes

### Router Won't Join
```bash
# Factory reset coordinator to reopen network
# In coordinator serial monitor, press reset button
# Or reflash coordinator firmware
```

### Messages Not Received
- Check both devices show `joined=1` in logs
- Verify different short addresses (not both 0xFFFF)
- Ensure devices are within Zigbee range (~10-30m indoors)
- Check for `Cluster manager initialized` in logs

### Network Lost
- Devices will automatically retry join every 30 seconds
- Watch for `"Not joined, retrying commissioning"`
- Coordinator will retry formation if needed

### Reset Everything
1. Flash coordinator with `idf.py erase-flash flash`
2. Flash routers with `idf.py erase-flash flash`
3. Power cycle all devices
4. Wait 60 seconds for network formation and joins

## Network Topology Examples

### Simple (2 devices)
```
[Coordinator] <---> [Router]
```

### Small Network (4 devices)
```
        [Coordinator]
           /    \
    [Router 1]  [Router 2]
          |
    [Router 3]
```

### Large Network (7+ devices)
```
        [Coordinator]
           /    |    \
          /     |     \
    [R1]     [R2]     [R3]
      |        |        |
    [R4]     [R5]     [R6]
```

## GPIO Pin Reference

### LEDs
- **LED_GPIO1 (GPIO 18)**: Local impact alerts
- **LED_GPIO2 (GPIO 13)**: Network status & gas alerts

### Sensors
- **I2C_SCL (GPIO 9)**: ADXL345 clock
- **I2C_SDA (GPIO 8)**: ADXL345 data
- **ADC (GPIO 11)**: Gas sensor input

### Controls
- **BUTTON (GPIO 15)**: Cancel major impact timer
- **BUZZER (GPIO 19)**: Audio alerts

## Performance Notes

- **Message latency**: ~100-500ms between nodes
- **Network capacity**: Up to 10 child devices per coordinator/router
- **Range**: 10-30m indoors, 100m+ line-of-sight outdoors
- **Power**: ~20mA average with periodic messaging

## Additional Resources

- Full documentation: `ZIGBEE_MESH_NETWORK.md`
- Cluster definitions: `Integrate/main/zb_canary_cluster.h`
- API reference: `Integrate/main/zb_cluster_manager.h`
- Network management: `Integrate/main/zb_mesh_network.h`
