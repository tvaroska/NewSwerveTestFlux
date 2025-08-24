# Code Architecture Overview

## High-Level Architecture

This FRC robot uses a **Command-Based Architecture** built on the WPILib framework with CTRE Phoenix 6 integration.

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Robot.java    │───▶│ RobotContainer   │───▶│   Subsystems    │
│  (TimedRobot)   │    │   (Bindings)     │    │   (Hardware)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        │                       │
         ▼                        ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Periodic Loops  │    │    Commands      │    │ Hardware Layer  │
│ (Auto/Teleop)   │    │  (Behaviors)     │    │ (Motors/Sensors)│
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Core Components

### 1. Robot.java (Main Entry Point)
- **Extends**: `TimedRobot`
- **Role**: Application lifecycle management
- **Key Features**:
  - Initializes RobotContainer
  - Manages autonomous/teleop transitions
  - Handles command scheduler
  - Contains commented elevator code for reference

```java
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); // Core command execution
    }
}
```

### 2. RobotContainer.java (System Orchestrator)
- **Role**: Central configuration hub
- **Responsibilities**:
  - Instantiate all subsystems
  - Configure controller bindings
  - Define autonomous commands
  - Set default commands

```java
public class RobotContainer {
    // Subsystem instances
    private final Pneumatics pneumatics = new Pneumatics();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // Controller and request objects
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
}
```

### 3. Subsystem Layer

#### CommandSwerveDrivetrain
- **Extends**: `TunerSwerveDrivetrain` (CTRE generated) + `Subsystem` (WPILib)
- **Architecture Pattern**: Adapter/Wrapper
- **Key Features**:
  - Wraps CTRE Phoenix 6 swerve implementation
  - Provides WPILib Command interface
  - Handles alliance perspective automatically
  - Includes SysId characterization routines

```java
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    // SysId routines for characterization
    private final SysIdRoutine m_sysIdRoutineTranslation;
    private final SysIdRoutine m_sysIdRoutineSteer;
    private final SysIdRoutine m_sysIdRoutineRotation;
    
    // Command factory method
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
}
```

#### Pneumatics Subsystem
- **Extends**: `SubsystemBase`
- **Architecture Pattern**: Simple Hardware Abstraction
- **Design**: Stateless service interface

```java
public class Pneumatics extends SubsystemBase {
    private final PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
    private final DoubleSolenoid m_solenoidA = m_pH.makeDoubleSolenoid(0, 1);
    private final DoubleSolenoid m_solenoidB = m_pH.makeDoubleSolenoid(6, 7);
    
    // Simple command interface
    public void setForward() { /* both solenoids forward */ }
    public void setReverse() { /* both solenoids reverse */ }
}
```

## Design Patterns

### 1. Command Pattern (WPILib Framework)
- **Purpose**: Encapsulate robot behaviors as objects
- **Implementation**: Most behaviors use built-in commands
- **Example**: `InstantCommand(() -> pneumatics.setForward())`

### 2. Supplier Pattern (Functional Interface)
- **Purpose**: Lazy evaluation of swerve requests
- **Benefit**: Allows real-time joystick input
- **Usage**: `drivetrain.applyRequest(() -> drive.withVelocityX(...))`

### 3. Factory Pattern (TunerConstants)
- **Purpose**: Centralized hardware configuration
- **Implementation**: CTRE Tuner X generates factory methods
- **Example**: `TunerConstants.createDrivetrain()`

### 4. Observer Pattern (Telemetry)
- **Purpose**: Decouple data logging from subsystem logic
- **Implementation**: `drivetrain.registerTelemetry(logger::telemeterize)`

## Data Flow

### Control Flow (Teleop)
```
Xbox Controller ──▶ RobotContainer ──▶ SwerveRequest ──▶ CommandSwerveDrivetrain ──▶ Hardware
      │                    │                │                      │                    │
   Input Events        Button Bindings   Request Objects      Command Execution    Motor Control
```

### Periodic Execution
```
Robot.robotPeriodic() ──▶ CommandScheduler.run() ──▶ Active Commands ──▶ Subsystem.periodic()
                                    │
                              Command Lifecycle:
                              • initialize()
                              • execute()  
                              • isFinished()
                              • end()
```

## Hardware Abstraction Layers

### Layer 1: Hardware Drivers (CTRE/WPILib)
- Direct hardware communication
- CAN bus protocols
- Low-level motor control

### Layer 2: Subsystem Wrappers
- Hardware abstraction
- Safety interlocks
- State management

### Layer 3: Command Interface
- Behavior definitions
- Coordination between subsystems
- User interface bindings

### Layer 4: Application Logic (RobotContainer)
- System orchestration
- Mode transitions
- Global state management

## Configuration Management

### Static Configuration (Compile-time)
- **TunerConstants.java**: Hardware IDs, mechanical constants
- **Robot.java**: System initialization
- **RobotContainer.java**: Control bindings

### Dynamic Configuration (Runtime)
- **Smart Dashboard**: PID tuning, debugging values
- **Alliance Station**: Field orientation, match state
- **Driver Station**: Enable/disable, mode selection

## Error Handling Strategy

### Hardware Faults
- CTRE devices report faults via CAN status
- Subsystems monitor device health in `periodic()`
- Graceful degradation when possible

### Software Exceptions
- WPILib command scheduler catches exceptions
- Individual command failures don't crash robot
- Error reporting via Driver Station console

### Safety Systems
- Hardware: Circuit breakers, motor controllers
- Software: Timeout detection, bounds checking
- Operational: Driver Station emergency stop

## Testing Architecture

### Simulation Support
- Desktop simulation enabled by default
- Physics simulation runs at 200Hz
- Allows testing without hardware

### Unit Testing
- JUnit 5 framework configured
- Focus on algorithm testing
- Mock hardware for subsystem tests

### Integration Testing
- Deploy and test on robot incrementally
- Use Smart Dashboard for real-time monitoring
- SysId characterization for system validation

## Extension Points

### Adding New Subsystems
1. Create class extending `SubsystemBase`
2. Add hardware initialization in constructor
3. Implement state management in `periodic()`
4. Add to RobotContainer and configure bindings

### Adding Autonomous
1. Create `Command` classes for behaviors
2. Use `SequentialCommandGroup` for sequences
3. Integrate with path planning libraries
4. Return from `getAutonomousCommand()`

### Adding Vision
1. Integrate with PhotonVision or Limelight
2. Use `addVisionMeasurement()` for pose estimation
3. Create targeting commands for auto-aim
4. Handle vision pipeline switching