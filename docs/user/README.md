# User Documentation

This directory contains documentation for robot operators, drive team members, and competition use.

## Quick Reference

### Robot Controls (Xbox Controller)

**Driving**:
- **Left Stick**: Move robot (X = strafe, Y = forward/backward)
- **Right Stick X**: Rotate robot
- **Left Bumper**: Reset field-centric heading

**Pneumatics**:
- **A Button**: Extend pneumatics
- **B Button**: Retract pneumatics  
- **X Button**: Turn off pneumatics
- **Start Button**: Enable compressor
- **Y Button**: Disable compressor

### Robot Capabilities

- **Swerve Drive**: Omnidirectional movement with field-centric control
- **Top Speed**: 4.99 m/s (16.4 ft/s)
- **Pneumatic System**: Dual solenoid control with automatic compressor

### Safety Notes

- Always ensure clear area around robot before enabling
- Compressor operates between 20-60 PSI automatically
- Field-centric mode adjusts based on alliance color
- Robot will apply alliance perspective automatically

## Competition Setup

1. Connect to robot WiFi or tether
2. Deploy code using Driver Station
3. Verify all systems in Test mode
4. Reset field-centric heading at start of match

## Troubleshooting

**Robot not responding**:
- Check Driver Station connection
- Verify robot is enabled
- Check battery voltage

**Swerve modules not aligned**:
- Use Left Bumper to reset field-centric
- Check module encoders in Smart Dashboard

**Pneumatics not working**:
- Verify compressor is enabled (Start button)
- Check pressure gauge (should be 20-60 PSI)
- Ensure solenoids are connected properly