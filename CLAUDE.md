# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) robot project built with WPILib 2025 using Java and Gradle. The robot features a CTRE Phoenix 6 swerve drivetrain with pneumatic subsystems.

## Development Commands

### Build and Deploy
- `./gradlew build` - Build the robot code
- `./gradlew deploy` - Deploy code to the RoboRIO
- `./gradlew simulateJava` - Run robot simulation
- `./gradlew test` - Run unit tests (JUnit 5)

### Platform-specific Commands
- Use `./gradlew` on Linux/macOS or `gradlew.bat` on Windows

## Architecture

### Core Components

**Robot Structure** (Command-based framework):
- `Robot.java` - Main robot class extending TimedRobot
- `RobotContainer.java` - Container for subsystems and command bindings
- `Main.java` - Entry point

**Subsystems**:
- `CommandSwerveDrivetrain` - CTRE Phoenix 6 swerve drive implementation with SysId characterization support
- `Pneumatics` - Dual DoubleSolenoid control with PneumaticHub (CAN ID 3)

**Generated Configuration**:
- `TunerConstants.java` - Auto-generated from CTRE Tuner X project configuration
- `tuner-project.json` - Contains swerve module hardware configuration (4 modules with Kraken X60 motors, CANcoders)

### Hardware Configuration

**Swerve Drivetrain**:
- 4 swerve modules with WCP Kraken X60 motors
- CTRE CANcoder encoders for steering
- Pigeon 2 IMU (CAN ID 20)
- Speed: 4.99 m/s at 12V, max angular rate: 0.75 rotations/s

**Pneumatics**:
- PneumaticHub on CAN ID 3
- Two DoubleSolenoids: ports (0,1) and (6,7)
- Analog compressor control (20-60 PSI)

### Key Features

**Controls** (Xbox Controller):
- Left stick: Translation control
- Right stick X: Rotation control  
- A: Pneumatics forward
- B: Pneumatics reverse (also point wheels)
- X: Pneumatics off
- Y: Enable/disable compressor
- Left bumper: Reset field-centric heading
- Back/Start + X/Y: SysId characterization routines

**Development Features**:
- Desktop simulation support enabled
- SysId routines for drive, steer, and rotation characterization
- Telemetry logging with SignalLogger
- Alliance-aware operator perspective

## Dependencies

Key vendor libraries in `vendordeps/`:
- Phoenix 6 (CTRE)
- REVLib
- WPILib New Commands

## Notes

- Robot code includes commented-out elevator control code for reference
- Uses open-loop voltage control for drive motors
- Supports both CAN FD and CAN 2.0 buses
- Configured for Java 17