# ADXL345 I2C Communication - Quick Fix Summary

## Critical Change: GPIO Pins

The ADXL345 is now configured to use **GPIO 8 (SCL) and GPIO 9 (SDA)** instead of GPIO 0 and 1.

### Why?
- GPIO 0 is a boot strapping pin
- GPIO 1 is UART TX (console output)
- These conflict with I2C operations

### What You Need to Do:

1. **Update your hardware connections:**
   - ADXL345 SCL → ESP32-C6 GPIO 8
   - ADXL345 SDA → ESP32-C6 GPIO 9

2. **Rebuild and flash:**
   ```bash
   cd /Users/justinbetz/Documents/GitHub/CanaryCap/Integrate
   idf.py clean build flash monitor
   ```

3. **Verify in logs:**
   You should see:
   ```
   I2C bus recovery completed
   I2C initialized successfully
   ADXL345 probe successful: DEVID=0x0E5
   ```

## If Still Not Working:

Check the diagnostic output:
```
[ADXL345] Check connections: SCL=GPIO8, SDA=GPIO9, ADDR=0x53
```

Then verify:
1. ✓ Wires properly connected to GPIO 8 and 9
2. ✓ Pull-up resistors present (4.7k recommended)
3. ✓ ADXL345 has power (check 3.3V)
4. ✓ Check ADXL345 address if not detecting (use 0x1D if ALT ADDRESS is tied high)

---

**File Modified:** `/Integrate/main/main.c`
- GPIO 8 = SCL (was GPIO 1)
- GPIO 9 = SDA (was GPIO 0)
- I2C frequency: 100 kHz
- I2C timeout: 5000 ms
