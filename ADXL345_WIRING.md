# ADXL345 I2C Wiring Configuration

## Updated I2C Bus Connections

```
ESP32-C6 (Microcontroller)           ADXL345 (Accelerometer)
──────────────────────────           ────────────────────────

GPIO 8  (SCL) ─────────┬───────────→ SCL
                       │  (with 4.7k pull-up to 3.3V)
                       
GPIO 9  (SDA) ─────────┬───────────→ SDA
                       │  (with 4.7k pull-up to 3.3V)
                       
GND ──────────────────→ GND
                       
3.3V ─────────────────→ VCC

CSB ──────────────────→ VCC  (I2C mode - tie high)
ALT ADDR ─────────────→ GND  (I2C address = 0x53)
```

## I2C Configuration Summary

| Parameter | Value | Notes |
|-----------|-------|-------|
| **SCL Pin** | GPIO 8 | Serial Clock Line |
| **SDA Pin** | GPIO 9 | Serial Data Line |
| **I2C Frequency** | 100 kHz | Standard I2C speed |
| **I2C Timeout** | 5000 ms | Increased for reliability |
| **Pull-ups** | 4.7 kΩ each | Optional (internal enabled) |
| **ADXL345 Address** | 0x53 | When ALT ADDRESS = GND |
| **Voltage** | 3.3V | ESP32-C6 logic level |

## Connection Verification Checklist

Before flashing, verify:

- [ ] SCL wire connected to GPIO 8 (not GPIO 1)
- [ ] SDA wire connected to GPIO 9 (not GPIO 0)
- [ ] GND properly connected between ESP32-C6 and ADXL345
- [ ] 3.3V power connected to ADXL345 VCC
- [ ] CSB pin tied to VCC (I2C mode)
- [ ] ALT ADDRESS pin tied to GND (address 0x53)
- [ ] No loose connections or cold solder joints
- [ ] Pull-up resistors present or internal pull-ups enabled

## After Flashing

Run the monitor and look for boot messages:

```
I2C bus recovery completed
I2C initialized successfully
ADXL345 probe successful: DEVID=0x0E5
ADXL345 probe: DEVID=0xE5
Enable measurement mode: register write OK
Data format set: register write OK
```

If you see errors, check:
1. Physical wiring (especially GPIO 8 and 9)
2. I2C address (0x53 vs 0x1D)
3. Power supply to ADXL345
4. Pull-up resistor values

## Troubleshooting

### Symptom: "ESP_ERR_INVALID_RESPONSE"
- Check GPIO 8 and 9 are connected (not GPIO 0 or 1)
- Verify pull-ups (should be 4.7k each)
- Check ADXL345 power supply

### Symptom: "I2C probe failed"
- Verify ADXL345 address: `0x53` (if `0x1D`, ALT ADDRESS is high)
- Check CSB pin configuration (must be high for I2C mode)
- Confirm power and GND connections

### Symptom: Wrong DEVID (not 0xE5)
- Device at different I2C address
- Wrong chip selected
- I2C communication error

---

**Last Updated:** November 12, 2025
**Status:** GPIO pins corrected (GPIO 8 & 9 instead of 0 & 1)
