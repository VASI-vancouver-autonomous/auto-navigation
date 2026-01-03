# RoboClaw Configuration Guide for USB Connection

## Overview
This guide helps you configure the ROS 2 RoboClaw driver using values from Basic Micro Motion Studio after autotuning.

## Step 1: Find Your USB Device

Your RoboClaw is connected via USB. The device should be:
```
/dev/ttyACM0
```

Verify it exists:
```bash
ls -l /dev/ttyACM0
```

If you need to add your user to the dialout group (to access the device without sudo):
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

## Step 2: Get Values from Basic Micro Motion Studio

After autotuning in Motion Studio, you need to extract these values for **each motor** (M1 and M2):

### From Motion Studio - PID Settings Tab:
**IMPORTANT: Use VELOCITY PID values, NOT position PID values!**

The ROS 2 driver uses velocity control (speed control) for differential drive robots. You need:

1. **Velocity P (Proportional)** - Copy the Velocity P value for Motor 1 and Motor 2
2. **Velocity I (Integral)** - Copy the Velocity I value for Motor 1 and Motor 2  
3. **Velocity D (Derivative)** - Copy the Velocity D value for Motor 1 and Motor 2
4. **QPPS (Quadrature Pulses Per Second)** - This is the maximum speed setting, usually found in the speed/velocity settings

**Note:** Position PID values are NOT used by this driver. Only use Velocity PID values.

### From Motion Studio - Encoder Settings:
5. **Encoder CPR (Counts Per Revolution)** - The encoder resolution
6. **Gear Ratio** - If applicable (e.g., 30:1 for Pololu motors)

### From Motion Studio - Motor/System Settings:
7. **Max Current** - Maximum current rating for your motors (in Amps)

## Step 3: Calculate Robot-Specific Values

You'll also need to measure/calculate these physical parameters:

### Wheel Parameters:
- **Wheel Radius** (in meters) - Measure the radius of your wheels
- **Wheel Separation** (in meters) - Distance between the centers of the left and right wheels

### Encoder Calibration:
- **Quad Pulses Per Meter** - Calculate this based on:
  ```
  Quad Pulses Per Meter = (Encoder CPR × Gear Ratio × 4) / (2 × π × Wheel Radius)
  ```
  The "× 4" accounts for quadrature encoding (4 edges per pulse)
  
- **Quad Pulses Per Revolution** - Calculate this:
  ```
  Quad Pulses Per Revolution = Encoder CPR × Gear Ratio × 4
  ```

### Example Calculation:
If you have:
- Encoder CPR: 64
- Gear Ratio: 30:1
- Wheel Radius: 0.10169 meters (4 inches)

Then:
- Quad Pulses Per Revolution = 64 × 30 × 4 = 7,680 pulses/revolution
- Quad Pulses Per Meter = 7,680 / (2 × π × 0.10169) = 12,020 pulses/meter

## Step 4: Configure motor_driver.yaml

Update the configuration file with your values:

```yaml
motor_driver_node:
  ros__parameters:
    # Acceleration rate (adjust based on your robot's capabilities)
    accel_quad_pulses_per_second: 1000  # Start with 1000, adjust as needed

    # USB Device - Change to your actual device
    device_name: "/dev/ttyACM0"  # Your USB device
    baud_rate: 38400  # Ignored for USB, but keep this value

    # RoboClaw address (default is 128, check your Motion Studio settings)
    device_port: 128  # Change if you set a different address

    # Motor 1 (Left Motor) - From Motion Studio
    m1_p: <YOUR_M1_P_VALUE>      # From Motion Studio PID settings
    m1_i: <YOUR_M1_I_VALUE>      # From Motion Studio PID settings
    m1_d: <YOUR_M1_D_VALUE>      # From Motion Studio PID settings
    m1_qpps: <YOUR_M1_QPPS>      # Max QPPS from Motion Studio

    # Motor 2 (Right Motor) - From Motion Studio
    m2_p: <YOUR_M2_P_VALUE>      # From Motion Studio PID settings
    m2_i: <YOUR_M2_I_VALUE>      # From Motion Studio PID settings
    m2_d: <YOUR_M2_D_VALUE>      # From Motion Studio PID settings
    m2_qpps: <YOUR_M2_QPPS>      # Max QPPS from Motion Studio

    # Maximum current (from your motor specifications)
    m1_max_current: <YOUR_MOTOR_MAX_CURRENT>  # e.g., 6.0 for 6A motors
    m2_max_current: <YOUR_MOTOR_MAX_CURRENT>  # e.g., 6.0 for 6A motors

    # Safety limits - Adjust based on your robot's capabilities
    max_angular_velocity: 0.5    # rad/s - adjust to your robot's max turn rate
    max_linear_velocity: 0.5     # m/s - adjust to your robot's max speed

    # Safety timeout (stop motors if no command received)
    max_seconds_uncommanded_travel: 0.25

    # Publishing options
    publish_joint_states: false   # Set to true if you need joint states
    publish_odom: true            # Keep true for odometry

    # Encoder calibration (calculated values from Step 3)
    quad_pulses_per_meter: <YOUR_CALCULATED_VALUE>        # From calculation
    quad_pulses_per_revolution: <YOUR_CALCULATED_VALUE>  # From calculation

    # Robot physical parameters (measured)
    wheel_radius: <YOUR_WHEEL_RADIUS>        # In meters
    wheel_separation: <YOUR_WHEEL_SEPARATION> # In meters

    # Status topic
    roboclaw_status_topic: "roboclaw_status"

    # Update rate
    sensor_rate_hz: 20.0

    # Debugging (set to true for troubleshooting)
    do_debug: false
    do_low_level_debug: false
```

## Step 5: Example Configuration

Here's a complete example with typical values:

```yaml
motor_driver_node:
  ros__parameters:
    accel_quad_pulses_per_second: 1000
    
    device_name: "/dev/ttyACM0"
    baud_rate: 38400
    device_port: 128
    
    # Example PID values (replace with your Motion Studio values)
    m1_p: 7.26
    m1_i: 1.37
    m1_d: 0.0
    m1_qpps: 2437
    
    m2_p: 7.26
    m2_i: 1.29
    m2_d: 0.0
    m2_qpps: 2437
    
    m1_max_current: 6.0
    m2_max_current: 6.0
    
    max_angular_velocity: 0.5
    max_linear_velocity: 0.5
    max_seconds_uncommanded_travel: 0.25
    
    publish_joint_states: false
    publish_odom: true
    
    quad_pulses_per_meter: 1566
    quad_pulses_per_revolution: 979.62
    
    wheel_radius: 0.10169
    wheel_separation: 0.345
    
    roboclaw_status_topic: "roboclaw_status"
    sensor_rate_hz: 20.0
    
    do_debug: false
    do_low_level_debug: false
```

## Step 6: Verify Configuration

1. **Check device access:**
   ```bash
   ls -l /dev/ttyACM0
   # Should show: crw-rw---- 1 root dialout
   ```

2. **Test connection (optional):**
   ```bash
   # Install minicom if needed
   sudo apt install minicom
   
   # Test serial communication (be careful - don't send random commands)
   minicom -D /dev/ttyACM0 -b 38400
   ```

3. **Launch the driver:**
   ```bash
   ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py
   ```

4. **Check topics:**
   ```bash
   ros2 topic list
   # Should see: /cmd_vel, /odom, /roboclaw_status
   ```

5. **Monitor status:**
   ```bash
   ros2 topic echo /roboclaw_status
   ```

## Troubleshooting

### Device Not Found
- Check USB connection: `lsusb`
- Verify device: `ls -l /dev/ttyACM*`
- Check permissions: `groups` (should include `dialout`)

### Motors Not Responding
- Verify `device_port` matches RoboClaw address (default 128)
- Check PID values match Motion Studio
- Enable debug: `do_debug: true` to see commands/responses

### Incorrect Odometry
- Recalculate `quad_pulses_per_meter` and `quad_pulses_per_revolution`
- Verify `wheel_radius` and `wheel_separation` are correct
- Check encoder connections

### Communication Errors
- Enable low-level debug: `do_low_level_debug: true`
- Check USB cable quality
- Verify baud rate (though USB ignores this setting)

## Quick Reference: Where to Find Values

| Value | Location in Motion Studio |
|-------|---------------------------|
| P, I, D | PID Settings tab → **Velocity PID** → Motor 1/Motor 2 (NOT Position PID) |
| QPPS | Speed/Velocity settings → Max QPPS |
| Encoder CPR | Encoder Settings → Counts Per Revolution |
| Gear Ratio | Motor Settings → Gear Ratio |
| Max Current | Motor Specifications (from datasheet) |
| Wheel Radius | Physical measurement |
| Wheel Separation | Physical measurement |

## Notes

- **USB vs UART**: When using USB, the `baud_rate` parameter is ignored, but you should still set it to a valid value (38400 is fine)
- **Device Port**: This is the RoboClaw's address (default 128). If you changed it in Motion Studio, update this value
- **Safety First**: Start with conservative `max_linear_velocity` and `max_angular_velocity` values and increase gradually
- **Autotuning**: The PID values from Motion Studio's autotune should work well, but you may need fine-tuning based on actual robot behavior

