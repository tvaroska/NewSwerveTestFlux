# Robot Controls Reference

## Xbox Controller Layout

### Primary Controls

| Input | Function | Notes |
|-------|----------|-------|
| **Left Stick Y** | Forward/Backward | Negative Y = Forward |
| **Left Stick X** | Strafe Left/Right | Negative X = Left |
| **Right Stick X** | Rotate | Negative X = Counterclockwise |
| **Left Bumper** | Reset Field-Centric | Press to reset robot orientation |

### Pneumatic Controls

| Button | Function | Description |
|--------|----------|-------------|
| **A** | Extend Pneumatics | Both solenoids forward |
| **B** | Retract Pneumatics | Both solenoids reverse |
| **X** | Pneumatics Off | Turn off both solenoids |
| **Start** | Enable Compressor | Start automatic compressor |
| **Y** | Disable Compressor | Stop compressor |

### Advanced Controls

| Input | Function | Notes |
|-------|----------|-------|
| **B + Left Stick** | Point Wheels | Wheels point in stick direction |
| **Back + Y** | SysId Forward Dynamic | Characterization routine |
| **Back + X** | SysId Reverse Dynamic | Characterization routine |
| **Start + Y** | SysId Forward Quasi | Characterization routine |
| **Start + X** | SysId Reverse Quasi | Characterization routine |

## Control Modes

### Field-Centric Drive
- Forward on controller always moves robot toward opposing alliance wall
- Automatically adjusts based on alliance color (red/blue)
- Reset with Left Bumper if robot orientation is incorrect

### Robot-Centric Drive
- Not currently implemented in this configuration
- Would make forward on controller always move robot's front forward

## Deadbands

- **Translation**: 10% of maximum speed (0.499 m/s)
- **Rotation**: 10% of maximum angular rate (0.075 rotations/s)
- Small joystick movements below these thresholds are ignored

## Safety Features

- Robot automatically applies alliance perspective on enable
- Idle mode applies configured neutral mode when disabled
- Emergency stop via Driver Station always available