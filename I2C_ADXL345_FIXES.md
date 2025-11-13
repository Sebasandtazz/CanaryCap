# ADXL345 I2C Communication Fixes

## Problem Summary
The ADXL345 accelerometer was unable to communicate with the ESP32-C6 microcontroller, resulting in `ESP_ERR_INVALID_RESPONSE` errors.

## Root Causes Identified and Fixed

### 1. **GPIO Pin Conflicts (PRIMARY ISSUE)**
**Problem:** GPIO 0 and GPIO 1 have special purposes on ESP32-C6:
- **GPIO 0**: Boot strapping pin (must be low during boot)
- **GPIO 1**: UART0 TX pin (used for console output)

These pins interfere with I2C bus operations.

**Solution:** Changed I2C pins to GPIO 8 and GPIO 9
- `GPIO 8` → SCL (Serial Clock)
- `GPIO 9` → SDA (Serial Data)

### 2. **I2C Bus Recovery**
Added automatic I2C bus recovery routine that:
- Generates 9 clock pulses to release stuck slave devices
- Creates proper I2C STOP condition to reset the bus
- Runs before I2C initialization

### 3. **I2C Timing and Configuration Improvements**
- Increased I2C frequency from 50 kHz → 100 kHz (standard I2C speed)
- Increased timeout from 2000 ms → 5000 ms for better reliability
- Added pull-up enable flags in configuration
- Added intr_priority and trans_queue_depth settings

### 4. **Improved Diagnostics**
Enhanced the ADXL345 probe function to display:
- Actual GPIO pins being used
- I2C address being accessed
- Expected vs actual DEVID values

## Code Changes

### File: `/Integrate/main/main.c`

#### Before:
```c
#define I2C_MASTER_SCL_IO           1 // SCL on ESP32-C6 WROOM (GPIO1)
#define I2C_MASTER_SDA_IO           0 // SDA on ESP32-C6 WROOM (GPIO0)
#define I2C_MASTER_FREQ_HZ          50000
#define I2C_MASTER_TIMEOUT_MS       2000
```

#### After:
```c
#define I2C_MASTER_SCL_IO           8 // SCL on ESP32-C6 (GPIO8 - better for I2C)
#define I2C_MASTER_SDA_IO           9 // SDA on ESP32-C6 (GPIO9 - better for I2C)
#define I2C_MASTER_FREQ_HZ          100000  // Start with standard 100kHz I2C speed
#define I2C_MASTER_TIMEOUT_MS       5000    // Increased timeout for reliability
```

## Hardware Changes Required

**IMPORTANT:** You must physically rewire your PCB or breadboard connections:

| Signal | Old GPIO | New GPIO |
|--------|----------|----------|
| SCL    | GPIO 1   | GPIO 8   |
| SDA    | GPIO 0   | GPIO 9   |

### Physical Connection Checklist:
- [ ] Disconnect ADXL345 SCL from GPIO 1
- [ ] Disconnect ADXL345 SDA from GPIO 0
- [ ] Connect ADXL345 SCL to GPIO 8
- [ ] Connect ADXL345 SDA to GPIO 9
- [ ] Verify pull-up resistors (4.7k-10k Ω recommended)
- [ ] Check power and GND connections

## ESP32-C6 GPIO Capabilities

GPIO pins suitable for I2C on ESP32-C6:
- **GPIO 8-9** ← **RECOMMENDED (now using)**
- GPIO 2-5
- GPIO 10-15
- GPIO 19-20
- GPIO 22-25
- GPIO 26-30

**Avoid for I2C:**
- GPIO 0: Boot strapping pin
- GPIO 1: UART0 TX
- GPIO 6-7: Internal use (flash)
- GPIO 16-18: Reserved

## Testing and Verification

After making hardware connections and rebuilding:

```bash
cd /Users/justinbetz/Documents/GitHub/CanaryCap/Integrate
idf.py clean
idf.py build
idf.py flash monitor
```

### Expected Console Output:
```
[ADXL345] I2C bus recovery completed
[ADXL345] I2C initialized successfully
[ADXL345] ADXL345 probe successful: DEVID=0x0E5
```

### If Still Failing:
Look for diagnostic messages showing:
```
[ADXL345] Check connections: SCL=GPIO8, SDA=GPIO9, ADDR=0x53
```

## Debugging Tips

1. **Use a logic analyzer** on GPIO 8 & 9 to verify I2C clock and data signals
2. **Check pull-ups** - If SCL/SDA don't return to high, pull-ups are missing or too weak
3. **Verify address** - ADXL345_ADDR (0x53) must match your chip (check ALT ADDRESS pin)
4. **Power check** - ADXL345 needs proper power supply (3.3V or 5V depending on module)
5. **Try slower speed** - If still failing, reduce I2C_MASTER_FREQ_HZ to 50000

## References

- [ESP32-C6 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf)
- [ADXL345 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf)
- [I2C Bus Recovery Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
