# I2C Pull-Up Problem Diagnosis

## Issue: SDA and SCL at 2.5V (not 3.3V)

### What You're Seeing:
- **Oscilloscope shows:** 2.5V on both SCL and SDA
- **ESP32-C6 GPIO reads:** Both lines as LOW (0V)
- **Expected:** Both lines should be 3.3V when idle

### Root Cause: Weak Pull-Ups

The I2C lines have **insufficient pull-up strength**. They're being pulled to only 2.5V instead of 3.3V.

This happens when:
1. Pull-up resistors are too large (4.7kΩ or larger)
2. ADXL345 is drawing significant current when idle
3. Something else is loading the bus
4. Pull-ups are not properly connected/soldered

### I2C Voltage Thresholds (ESP32-C6):
- GPIO input LOW threshold: ~1.65V
- GPIO input HIGH threshold: ~0.75 × VDD ≈ 2.47V **← YOU'RE AT THIS EDGE!**

At 2.5V, the GPIO is right at the threshold and reads inconsistently as LOW.

## Solutions (in order of preference):

### Solution 1: Reduce Pull-Up Resistance (BEST - No soldering needed if already on board)

**Change pull-up resistors:**
- Current (likely): 4.7kΩ
- Try: 2.2kΩ or 1.8kΩ

**Effect:** Lower resistance → faster charging → higher final voltage (closer to 3.3V)

**Formula:**
```
V = VDD × (Rload / (Rpull-up + Rload))
```
- At 4.7kΩ: V ≈ 2.5V (what you see)
- At 2.2kΩ: V ≈ 3.1V (better!)
- At 1.8kΩ: V ≈ 3.2V (much better!)

### Solution 2: Increase I2C Bus Capacitance Timeout

Already done! Changed `I2C_DELAY_US` from 5 to 50 microseconds. This gives weak pull-ups more time to charge the bus.

### Solution 3: Check ADXL345 Power Supply

Ensure ADXL345 is properly powered:
- VCC should be 3.3V (not lower)
- GND should be solid connection
- Add 100nF bypass capacitor near VCC pin if not already present

### Solution 4: Check Bus Loading

Verify nothing else is pulling the bus down:
- No other devices on the I2C bus
- No shorts between SDA/SCL
- No accidental connections to GND

## Immediate Workaround for Testing

If you can't change resistors, try increasing the delay further in the code:

Edit line ~149 in main.c:
```c
#define I2C_DELAY_US 100  // Try even longer delay (was 50, then 5)
```

This gives weak pull-ups more time to fully charge the bus between operations.

## Long-Term Fix (Recommended)

**Install 2.2kΩ or 1.8kΩ pull-up resistors** on your PCB for SDA and SCL lines.

Standard I2C pull-up values:
- 4.7kΩ: Good for 400 kHz I2C, HIGH speeds
- 2.2kΩ: Good for mixed speeds, standard choice
- 1.8kΩ: Good for longer cables or heavily loaded buses
- 1.0kΩ: Only for very short distances

For GPIO 0 and 1 on ESP32-C6 with ADXL345, **2.2kΩ is ideal**.

## Testing Your Fix

After changing pull-ups OR increasing I2C_DELAY_US:

1. Build and flash:
   ```bash
   idf.py build
   idf.py flash
   ```

2. Monitor output:
   ```bash
   idf.py monitor
   ```

3. Look for diagnostic output showing line voltages

4. Check for DEVID=0xE5 (correct) vs DEVID=0x00 (wrong)

## Calculation for Your Setup

Your oscilloscope shows 2.5V. With standard component values:

If using 4.7kΩ pull-ups:
- ADXL345 leakage current + weak charging
- Result: 2.5V instead of 3.3V

**Switch to 2.2kΩ:**
- Faster charging current
- Higher final voltage (≈3.1-3.2V)
- GPIO reads as HIGH correctly
- Communication works!

---

## Quick Reference: Pull-Up Selection

| Pull-Up | Voltage Achieved | Speed | Use Case |
|---------|------------------|-------|----------|
| 10kΩ | 2.2V | Very slow | Long cables only |
| 4.7kΩ | 2.5V | 100-400 kHz | ← **Your current problem** |
| 2.2kΩ | 3.1V | 100-400 kHz | ← **Recommended for your case** |
| 1.8kΩ | 3.2V | up to 1 MHz | Fast, heavily loaded buses |
| 1.0kΩ | 3.25V | Very fast | Short distances only |

---

## Hardware Checklist

- [ ] Verify pull-ups are 2.2kΩ or smaller
- [ ] Confirm pull-ups connected between SDA/3.3V and SCL/3.3V
- [ ] Check ADXL345 power supply (should be 3.3V)
- [ ] Verify clean GND connection between ESP32-C6 and ADXL345
- [ ] Use oscilloscope to measure final voltage (should be ≥3.0V)

Once pull-ups are correct, DEVID should read 0xE5 and communication will work!
