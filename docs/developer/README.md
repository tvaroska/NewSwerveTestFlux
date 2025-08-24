# Developer Documentation

This directory contains technical documentation for developers working on this FRC robot codebase.

## Development Setup

### Prerequisites
- WPILib 2025 installed
- Java 17 JDK
- Git
- CTRE Phoenix Tuner X (for swerve configuration)

### Getting Started
1. Clone repository
2. Open in VS Code with WPILib extension
3. Run `./gradlew build` to verify setup
4. Deploy to robot with `./gradlew deploy`

## Architecture Overview

### Command-Based Framework
This robot uses WPILib's command-based programming framework:
- **Robot.java**: Main robot class, handles initialization and periodic functions
- **RobotContainer.java**: Manages subsystems and command bindings
- **Subsystems**: Encapsulate hardware control (drivetrain, pneumatics)
- **Commands**: Define robot behaviors (mostly built-in for swerve)

### Key Components

**CommandSwerveDrivetrain** (`src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`):
- Extends CTRE Phoenix 6 TunerSwerveDrivetrain
- Implements WPILib Subsystem interface
- Handles alliance perspective and simulation
- Provides SysId characterization routines

**Pneumatics** (`src/main/java/frc/robot/subsystems/Pneumatics.java`):
- Controls PneumaticHub and DoubleSolenoids
- Simple command interface for extend/retract/off
- Compressor management

**TunerConstants** (`src/main/java/frc/robot/generated/TunerConstants.java`):
- Auto-generated from CTRE Tuner X project
- Contains all swerve module hardware configuration
- **Do not manually edit** - regenerate from Tuner X

## Development Workflow

### Making Changes
1. Create feature branch from main
2. Make changes following existing code style
3. Test thoroughly in simulation and on robot
4. Create pull request for review

### Testing
- **Simulation**: `./gradlew simulateJava`
- **Unit Tests**: `./gradlew test`
- **Robot Testing**: Deploy and test incrementally

### Code Style
- Follow WPILib Java conventions
- Use meaningful variable names
- Add comments for complex logic
- Keep methods focused and small

## Hardware Configuration

### CAN Bus Layout
- **Drivetrain Bus**: All swerve modules, encoders, and Pigeon 2
- **PneumaticHub**: CAN ID 3

### Swerve Module IDs
- **Front Left**: Drive=1, Steer=2, Encoder=24
- **Front Right**: Drive=19, Steer=18, Encoder=23  
- **Back Left**: Drive=4, Steer=3, Encoder=25
- **Back Right**: Drive=16, Steer=17, Encoder=26

### Pneumatic Ports
- **DoubleSolenoid A**: Ports 0,1 on PneumaticHub
- **DoubleSolenoid B**: Ports 6,7 on PneumaticHub

## Key Concepts

### Field-Centric Control
- Robot maintains orientation relative to field
- Alliance color determines "forward" direction
- Automatically applied based on Driver Station alliance

### SysId Characterization
- Built-in routines for drive, steer, and rotation
- Use for tuning PID gains and feedforward
- Triggered by controller combinations (Back/Start + X/Y)

### Simulation
- Desktop simulation enabled by default
- Useful for testing control logic without robot
- Physics simulation runs at 200Hz for better PID behavior

## Common Development Tasks

### Adding New Subsystem
1. Create class extending SubsystemBase
2. Add hardware initialization in constructor
3. Implement periodic() if needed
4. Add to RobotContainer and configure bindings

### Modifying Controls
1. Edit configureBindings() in RobotContainer.java
2. Use CommandXboxController for button bindings
3. Test in simulation before deploying

### Tuning Swerve Drive
1. Use CTRE Tuner X for hardware configuration
2. Regenerate TunerConstants.java from Tuner X
3. Use SysId for characterization and PID tuning
4. Test incrementally with different speeds/accelerations

### Adding Autonomous
1. Create Command classes for autonomous routines
2. Implement getAutonomousCommand() in RobotContainer
3. Use PathPlanner or WPILib trajectory following
4. Test paths in simulation first

## Debugging

### Common Issues
- **CAN timeouts**: Check wiring and device IDs
- **Swerve drift**: Verify encoder offsets and inversion
- **Control lag**: Check for blocking code in periodic methods
- **Build errors**: Verify all dependencies in build.gradle

### Diagnostic Tools
- **Smart Dashboard**: Monitor real-time values
- **CTRE Phoenix Tuner X**: Configure and debug CTRE devices
- **Driver Station**: View console output and system status
- **Robot Characterization**: SysId data analysis

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [FRC Programming Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html)