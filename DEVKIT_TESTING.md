# Device Connection/Disconnection Monitoring - DevKit Testing

## Changes Made

### 1. Onboard LED Support (GPIO 8)
- Added `LED_ONBOARD` definition for ESP32-C6-DevKit onboard LED (GPIO 8)
- Initialized in `app_main()` 
- LED behavior:
  - **Connected to network**: LED stays ON
  - **Disconnected**: LED turns OFF
  - **Device joins**: Flashes rapidly 5 times
  - **New device joins network**: Blinks 3 times
  - **Device leaves network**: Blinks slowly 2 times

### 2. Enhanced Connection/Disconnection Detection

#### Local Device State Changes (`mesh_state_change_callback`)
```
JOINED:
- Log: "==> DEVICE CONNECTED TO NETWORK <=="
- Onboard LED: Flash rapidly 5x, then stay ON
- Buzzer: Short beep (200ms)

DISCONNECTED:
- Log: "==> DEVICE DISCONNECTED FROM NETWORK <=="
- Onboard LED: Turn OFF
- LED_GPIO2: Blink continuously (300ms intervals)
- Buzzer: Warning beep (500ms)
```

#### Remote Device Events (`mesh_device_event_callback`)
```
NEW DEVICE JOINED:
- Log: "*** NEW DEVICE JOINED NETWORK: 0x[addr] ***"
- Onboard LED: Blink 3 times
- Buzzer: Quick beep (150ms)

DEVICE LEFT:
- Log: "*** DEVICE LEFT NETWORK: 0x[addr] ***"
- Onboard LED: Slow blink 2 times
- Buzzer: Warning beep (300ms)
```

#### Network Status Messages (`network_status_received_callback`)
```
REMOTE CONNECTED:
- Log: "*** REMOTE DEVICE CONNECTED: 0x[addr] (PAN:0x... CH:...) ***"

REMOTE DISCONNECTED:
- Log: "*** REMOTE DEVICE DISCONNECTED: 0x[addr] ***"
- LED_GPIO1: Brief flash
- Buzzer: Short beep (200ms)
```

### 3. Sensor Tasks Disabled for DevKit Testing
- I2C (ADXL345) task commented out
- ADC (gas sensor) task commented out
- Added log message: "ADC and I2C tasks disabled for DevKit testing"

## Testing Without Sensors

### What Works:
✅ Zigbee mesh networking
✅ Device join/leave detection
✅ Network connection/disconnection monitoring
✅ Onboard LED indicators
✅ Buzzer alerts
✅ Remote device status messages

### What's Disabled:
❌ ADXL345 accelerometer (I2C) - commented out
❌ Gas sensor (ADC) - commented out

## Testing Procedure

### 1. Single Device Test
```bash
# Flash as coordinator
cd Integrate
idf.py build flash monitor
```

**Expected behavior:**
- Device boots
- Forms network (if coordinator)
- Onboard LED flashes 5x then stays ON
- Log: "==> DEVICE CONNECTED TO NETWORK <=="

### 2. Two Device Test

**Device 1 (Coordinator):**
```c
// main.c - line 48
//#define ZB_BUILD_AS_ROUTER  // Commented = Coordinator
```

**Device 2 (Router):**
```c
// main.c - line 48
#define ZB_BUILD_AS_ROUTER  // Uncommented = Router
```

**Expected behavior:**

**Coordinator sees:**
- `*** NEW DEVICE JOINED NETWORK: 0x[addr] ***`
- Onboard LED blinks 3 times
- Quick beep

**Router sees:**
- `==> DEVICE CONNECTED TO NETWORK <==`
- Onboard LED flashes 5x then stays ON
- Beep

### 3. Disconnect Test

**Power off one device:**

**Remaining device sees:**
- `*** DEVICE LEFT NETWORK: 0x[addr] ***`
- Onboard LED blinks slowly 2x
- Warning beep

### 4. Network Status Messages

Both devices periodically broadcast their status. When received:
- `*** REMOTE DEVICE CONNECTED: 0x[addr] ***`
- `*** REMOTE DEVICE DISCONNECTED: 0x[addr] ***`

## LED Summary

| Event | Onboard LED (GPIO 8) | LED_GPIO2 (13) | LED_GPIO1 (18) |
|-------|---------------------|----------------|----------------|
| Connected | Flash 5x, stay ON | Brief flash | - |
| Disconnected | Turn OFF | Blink continuously | - |
| New device joins | Blink 3x | - | - |
| Device leaves | Slow blink 2x | - | - |
| Remote disconnect | - | - | Brief flash |

## Buzzer Summary

| Event | Frequency | Duration |
|-------|-----------|----------|
| Connected | 2000 Hz | 200 ms |
| Disconnected | 2000 Hz | 500 ms |
| New device joins | 2500 Hz | 150 ms |
| Device leaves | 1500 Hz | 300 ms |
| Remote disconnect | 1500 Hz | 200 ms |

## Serial Monitor Output

Look for these key messages:

```
I (xxx) zb_mesh: Onboard LED initialized on GPIO 8
I (xxx) zb_mesh: ADC and I2C tasks disabled for DevKit testing
W (xxx) ESP_ZB_ON_OFF_SWITCH: *** MESH STATE CHANGED: X -> Y ***
W (xxx) ESP_ZB_ON_OFF_SWITCH: ==> DEVICE CONNECTED TO NETWORK <==
W (xxx) ESP_ZB_ON_OFF_SWITCH: *** NEW DEVICE JOINED NETWORK: 0xXXXX ***
E (xxx) ESP_ZB_ON_OFF_SWITCH: *** DEVICE LEFT NETWORK: 0xXXXX ***
E (xxx) ESP_ZB_ON_OFF_SWITCH: ==> DEVICE DISCONNECTED FROM NETWORK <==
```

## Re-enabling Sensors Later

To re-enable when you have sensors:

1. **I2C/ADXL345:**
```c
// Uncomment in app_main():
uint8_t data[6];
uint8_t devid = 0;
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

i2c_master_init(&bus_handle, &dev_handle);
// ... rest of I2C init code

major_impact_timer = xTimerCreate("MajorImpactTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, major_impact_timer_callback);
xTaskCreate(i2c_task, "i2c_task", 4096, dev_handle, 5, NULL);
```

2. **ADC/Gas Sensor:**
```c
// Uncomment in app_main():
adc_continuous_handle_t handle = NULL;
continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
xTaskCreate(adc_task, "adc_task", 4096, handle, 5, &adc_task_handle);
// ... rest of ADC init code
```

## GPIO Pin Reference for DevKit

- **GPIO 8**: Onboard LED (active LOW)
- **GPIO 18**: LED_GPIO1 (external)
- **GPIO 13**: LED_GPIO2 (external)
- **GPIO 19**: Buzzer
- **GPIO 15**: Button (with pull-up)

## Troubleshooting

**Onboard LED not working:**
- Verify GPIO 8 is correct for your DevKit model
- Some DevKits use different pins (check schematic)

**Devices not detecting each other:**
- Check both are on same Zigbee channel
- Verify one is coordinator, others are routers
- Check serial logs for "Network formed successfully"

**No buzzer sound:**
- GPIO 19 may not have buzzer connected on DevKit
- This is normal - focus on LED indicators and logs
