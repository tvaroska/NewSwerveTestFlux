# Troubleshooting Guide

## Common Issues

### Robot Not Moving

**Symptoms**: Robot is enabled but doesn't respond to joystick input

**Possible Causes**:
1. Driver Station not connected or enabled
2. Code not deployed properly
3. Battery voltage too low
4. Controller not connected

**Solutions**:
1. Check Driver Station connection status (green light)
2. Verify robot is enabled (not just connected)
3. Check battery voltage (should be >12V under load)
4. Reconnect Xbox controller to Driver Station computer
5. Redeploy robot code if necessary

### Swerve Drive Issues

**Symptoms**: Robot moves erratically or modules not aligned

**Possible Causes**:
1. Module encoders need calibration
2. Field-centric heading incorrect
3. CAN bus communication issues

**Solutions**:
1. Reset field-centric heading with Left Bumper
2. Check Smart Dashboard for module positions
3. Power cycle robot to reset CAN devices
4. Run SysId characterization if performance is poor

### Pneumatic System Problems

**Symptoms**: Solenoids not actuating or no air pressure

**Possible Causes**:
1. Compressor not enabled
2. Low air pressure
3. Solenoid wiring issues
4. PneumaticHub not responding

**Solutions**:
1. Enable compressor with Start button
2. Check pressure gauge (should read 20-60 PSI)
3. Verify solenoid connections to PneumaticHub ports 0,1 and 6,7
4. Check PneumaticHub CAN ID (should be 3)
5. Power cycle robot to reset PneumaticHub

### Communication Issues

**Symptoms**: Intermittent robot response or lag

**Possible Causes**:
1. WiFi interference
2. Low battery affecting radio
3. Driver Station computer issues

**Solutions**:
1. Switch to USB tether connection
2. Check battery voltage and connections
3. Restart Driver Station software
4. Move closer to robot to improve signal strength

## Diagnostic Tools

### Smart Dashboard Values

Monitor these values during operation:
- Battery voltage
- CAN device status
- Swerve module positions and velocities
- Pneumatic pressure readings
- Robot pose and heading

### Driver Station Diagnostics

Check these indicators:
- Robot connection status (should be green)
- Battery voltage display
- CAN device list (should show all expected devices)
- Joystick/controller status

## Emergency Procedures

### Robot Runaway
1. Press SPACEBAR or click Disable in Driver Station immediately
2. If software disable fails, pull main breaker on robot
3. Never attempt to physically stop robot - let it hit wall if necessary

### Pneumatic Emergency
1. Disable robot to stop compressor
2. Manually release air pressure if needed
3. Check all fittings and hoses before re-enabling

### Electrical Issues
1. Disable robot immediately
2. Check for smoke, burning smell, or sparks
3. Disconnect battery if safe to do so
4. Do not re-enable until issue is identified and resolved

## Getting Help

1. Check Driver Station console for error messages
2. Review Smart Dashboard for system status
3. Consult with programming team for software issues
4. Contact electrical team for hardware problems
5. Refer to CTRE Phoenix documentation for swerve drive issues