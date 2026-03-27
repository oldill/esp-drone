# Brushless Motor Support for ESP-Drone

This document describes how to configure and use brushless motors (1503 3100KV) with the ESP-Drone firmware.

## Overview

The ESP-Drone firmware now supports brushless motors with Electronic Speed Controllers (ESCs). This is ideal for applications requiring higher power and efficiency compared to brushed motors.

### Supported Motors
- **1503 3100KV brushless motor** - Primary target
- Any brushless motor compatible with standard servo PWM ESC control

### ESC Requirements
- Servo PWM signal input (50 Hz, 1000-2000 µs pulse width)
- Supply voltage compatible with battery (typically 3S or 4S LiPo)
- Programmable to accept standard servo PWM control

## Hardware Setup

### Connection Diagram
```
ESP32 GPIO Pin  ──→  ESC Signal Input
                     ESC Battery +   ──→  Battery +
                     ESC Battery -   ──→  Battery -
                     
ESC Phase Wires (A, B, C) ──→ Motor Phase Leads
Motor GND ──→ Battery GND (common)
```

### GPIO Configuration

Motor GPIO pins are configured in `main/Kconfig.projbuild`:

- **MOTOR01_PIN**: Motor 1 control signal (GPIO 5 for ESP32-S3 Drone v1.2)
- **MOTOR02_PIN**: Motor 2 control signal (GPIO 6 for ESP32-S3 Drone v1.2)
- **MOTOR03_PIN**: Motor 3 control signal (GPIO 3 for ESP32-S3 Drone v1.2)
- **MOTOR04_PIN**: Motor 4 control signal (GPIO 4 for ESP32-S3 Drone v1.2)

Each GPIO outputs a servo PWM signal to the ESC signal input pin.

## Software Configuration

### Selecting Brushless Motor Type

In `main/Kconfig.projbuild`, under the "motors config" menu:

```
┌─ ESPDrone Config
│  └─ motors config
│     └─ Motor type (choose one):
│        ☐ brushed 715 motor
│        ☐ brushed 720 motor
│        ☑ brushless 1503 motor (3100KV with ESCs)  ← SELECT THIS
```

To configure via terminal:
```bash
idf.py menuconfig
# Navigate to: ESPDrone Config → motors config → Motor type
# Select "brushless 1503 motor (3100KV with ESCs)"
# Save and exit
```

### Build and Flash

After configuration:
```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

(On Windows, `/dev/ttyUSB0` is typically `COM3` or similar)

## Motor Control

### Throttle Mapping

The firmware uses standard servo PWM mapping for brushless motors:

- **0% throttle** → 1000 µs pulse (ESC minimum/stopped)
- **50% throttle** → 1500 µs pulse (center)
- **100% throttle** → 2000 µs pulse (ESC maximum)

This follows the standard servo PWM protocol that all modern ESCs support.

### API Functions

The existing motor control API works seamlessly with brushless motors:

```c
// Set motor thrust (0 = stopped, 65536 = full throttle)
motorsSetRatio(MOTOR_M1, 32768);  // 50% throttle (1500µs)

// Get current motor ratio
int ratio = motorsGetRatio(MOTOR_M1);

// Initialize motors (reads config and sets up PWM)
motorsInit(motorMapDefaultBrushless);

// Stop all motors
motorsDeInit(motorMapDefaultBrushless);

// Test motors
motorsTest();
```

## PWM Signal Specifications

### Servo PWM Protocol
- **Frequency**: 50 Hz (20 ms period)
- **Pulse width range**: 1000 - 2000 microseconds
- **Implementation**: ESP32 RMT (Remote Control Transceiver) module
- **Channels**: 4 RMT channels (0-3)
- **Resolution**: 1 microsecond precision

### Signal Timing
```
Period = 20 ms (50 Hz)

Idle:  0µs ─────────────────────────────────────── 20ms
       
1% :   ┌──┐                                        
       │  │1000µs                                  
       └──┴────────────────────────────────────────

50%:   ┌──────────┐                                
       │  1500µs  │                                
       └──────────┴────────────────────────────────

100%:  ┌─────────────────┐                         
       │     2000µs      │                         
       └─────────────────┴────────────────────────
```

### ESC Protocol
The PWM signal follows the standard servo/ESC protocol:
- **Below 1000 µs**: ESC disarmed/failsafe
- **1000-1500 µs**: 0-50% throttle (linear)
- **1500 µs**: Neutral/center
- **1500-2000 µs**: 50-100% throttle (linear)
- **Above 2000 µs**: Over-range, usually ignored

## Calibration & Arming

### ESC Arming Procedure

Most ESCs require calibration before first use:

1. **Power off everything** (disconnect battery)

2. **Set motors to 0% throttle** in firmware/software

3. **Power on the ESC** (signal should be at 1000µs)

4. **Apply power to the drone** while keeping signal at 1000µs

5. **Some ESCs may beep** - follow the beep sequence for calibration (varies by ESC firmware)

6. **Set motors to 100% throttle** briefly (for full range calibration - 2000µs)

7. **Return to 0% throttle** and wait for confirmation beeps

8. **ESC is now calibrated** - apply throttle to test

For detailed calibration, refer to your ESC manual.

## Thrust Parameters

When using brushless motors, you may need to adjust thrust parameters in the flight controller. Key parameters in `components/core/crazyflie/modules/src/position_controller_pid.c`:

```c
// Add support for brushless motor thrust calibration:
#ifdef CONFIG_MOTOR_BRUSHLESS_1503
    #define thrustBase  32000  // Adjust based on your motor/ESC/propeller combo
    #define thrustMin   5000   // Minimum throttle to overcome drag
#endif
```

These values depend on:
- Motor KV rating (3100KV for the 1503)
- Propeller size and pitch
- Battery voltage and capacity
- Total drone weight

## Troubleshooting

### Motor won't spin
- **Check connections**: Verify GPIO pins are correctly wired to ESC signal inputs
- **ESC power**: Ensure ESC has battery power connected
- **Signal measurement**: Verify servo PWM signal with oscilloscope (should show 50Hz, 1000-2000µs pulses)
- **ESC mode**: Check if ESC is in programming mode - consult ESC documentation

### Motor spins inconsistently
- **ESC calibration**: Recalibrate ESC using the procedure above
- **Signal quality**: Ensure servo signal wire has good shielding (keep away from power lines)
- **Battery voltage**: Confirm battery voltage is in ESC's operating range (usually 2-4S LiPo)
- **Check servo signal**: Connect oscilloscope to confirm 50Hz frequency and correct pulse width range

### Control is jerky or unstable
- **Tune PID**: The default PID may need adjustment for brushless motors - adjust in `position_controller_pid.c`
- **Thrust calibration**: Recalibrate thrustBase and thrustMin values
- **Signal timing**: Verify RMT is generating correct 50Hz servo PWM (not high-frequency LEDC)

### ESC beeps repeatedly
- **Disarmed state**: ESC may be reporting that it's not armed - apply brief throttle
- **Low voltage**: Battery voltage may be too low for ESC to operate
- **Calibration needed**: Recalibrate ESC using calibration procedure above
- **Wrong throttle range**: ESC may need different min/max calibration

## Advanced: Custom Motor Definitions

To add support for other brushless motors, edit `components/drivers/general/motors/motors_def_cf2.c`:

```c
// Example: Add support for a different motor
static const MotorPerifDef CONN_M1_CUSTOM = {
    .drvType = BRUSHLESS,
    // RMT-based servo PWM automatically handles brushless control
};

const MotorPerifDef *motorMapCustom[NBR_OF_MOTORS] = {
    &CONN_M1_CUSTOM,
    &CONN_M2_CUSTOM,
    &CONN_M3_CUSTOM,
    &CONN_M4_CUSTOM
};
```

Then reference it in `platform_cf2.c` for selection.

## Performance Characteristics

### 1503 3100KV Motor Specifications
- **KV Rating**: 3100 RPM/Volt
- **Recommended Propeller**: 3-4 inch
- **Max Current**: ~15-20A per motor (varies by ESC)
- **Max Power**: ~200-300W per motor (at full throttle with 4S battery)
- **Thrust**: Depends on propeller and batteries
  - 3S LiPo (11.1V): ~400-600g thrust per motor (2.4" props)
  - 4S LiPo (14.8V): ~500-800g thrust per motor (2.4" props)

## Signal Analysis

To verify correct servo PWM output, measure with an oscilloscope:

```bash
# At 0% throttle (motor stopped)
Pulse width: ~1000 µs per 20ms period (50Hz)

# At 50% throttle
Pulse width: ~1500 µs per 20ms period (50Hz)

# At 100% throttle
Pulse width: ~2000 µs per 20ms period (50Hz)
```

Use an oscilloscope or logic analyzer on the motor GPIO pins to verify the signal matches your throttle commands.

## References

- [ESP-Drone GitHub](https://github.com/espressif/esp-drone)
- [ESP-IDF RMT Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/rmt.html)
- [Servo PWM Standard](https://wiki.multiwii.org/view/Main_Page) - MultiWii servo PWM documentation
- ESC-specific documentation (e.g., BLHeli, Simonk)
- Motor manufacturer specifications

## Support

For issues or questions:
1. Check this documentation
2. Review the [main README.md](README.md)
3. Consult ESC and motor manufacturer documentation
4. Create an issue on the ESP-Drone GitHub repository

