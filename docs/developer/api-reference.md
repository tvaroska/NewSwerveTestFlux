# API Reference

## Subsystems

### CommandSwerveDrivetrain

**Location**: `frc.robot.subsystems.CommandSwerveDrivetrain`

**Purpose**: Controls the 4-module swerve drivetrain using CTRE Phoenix 6

#### Key Methods

```java
// Apply a swerve request (most common usage)
public Command applyRequest(Supplier<SwerveRequest> requestSupplier)

// SysId characterization routines  
public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
public Command sysIdDynamic(SysIdRoutine.Direction direction)

// Vision integration
public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds)
public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs)
```

#### Usage Examples

```java
// Basic field-centric drive (in RobotContainer)
drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
    )
);

// Point wheels at angle
joystick.b().whileTrue(drivetrain.applyRequest(() ->
    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
));

// Brake mode
joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
```

#### Important Constants

```java
// From TunerConstants
public static final double kSpeedAt12Volts = 4.991383093407191; // m/s
public static final SwerveDrivetrainConstants DrivetrainConstants = ...;
public static final SwerveModuleConstants<?, ?, ?> FrontLeft = ...;
// etc.
```

### Pneumatics

**Location**: `frc.robot.subsystems.Pneumatics`

**Purpose**: Controls dual DoubleSolenoids and compressor via PneumaticHub

#### Methods

```java
// Solenoid control
public void setForward()    // Extend both solenoids
public void setReverse()    // Retract both solenoids  
public void setOff()        // Turn off both solenoids

// Compressor control
public void enableCompressor()   // Enable analog compressor (20-60 PSI)
public void disableCompressor()  // Disable compressor
```

#### Usage Examples

```java
// In RobotContainer configureBindings()
joystick.a().onTrue(new InstantCommand(() -> pneumatics.setForward()));
joystick.b().onTrue(new InstantCommand(() -> pneumatics.setReverse()));
joystick.x().onTrue(new InstantCommand(() -> pneumatics.setOff()));
joystick.start().onTrue(new InstantCommand(() -> pneumatics.enableCompressor()));
joystick.y().onTrue(new InstantCommand(() -> pneumatics.disableCompressor()));
```

## Swerve Requests

### Common SwerveRequest Types

```java
// Field-centric driving
SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

// Robot-centric driving  
SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1);

// Brake mode (X-pattern)
SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

// Point wheels in direction
SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

// Idle (apply neutral mode)
SwerveRequest.Idle idle = new SwerveRequest.Idle();
```

### SysId Requests (for characterization)

```java
// Translation characterization (drive motors)
SwerveRequest.SysIdSwerveTranslation translationChar = new SwerveRequest.SysIdSwerveTranslation();

// Steer characterization (steer motors)  
SwerveRequest.SysIdSwerveSteerGains steerChar = new SwerveRequest.SysIdSwerveSteerGains();

// Rotation characterization (robot rotation)
SwerveRequest.SysIdSwerveRotation rotationChar = new SwerveRequest.SysIdSwerveRotation();
```

## Hardware Configuration

### CAN Device IDs

```java
// Swerve modules (from tuner-project.json)
// Front Left: Drive=1, Steer=2, Encoder=24
// Front Right: Drive=19, Steer=18, Encoder=23
// Back Left: Drive=4, Steer=3, Encoder=25  
// Back Right: Drive=16, Steer=17, Encoder=26

// Other devices
public static final int PIGEON_2_ID = 20;
public static final int PNEUMATIC_HUB_ID = 3;
```

### Pneumatic Ports

```java
// In Pneumatics.java
private static final int PH_CAN_ID = 3;
private final DoubleSolenoid m_solenoidA = m_pH.makeDoubleSolenoid(0, 1);
private final DoubleSolenoid m_solenoidB = m_pH.makeDoubleSolenoid(6, 7);
```

## Controller Bindings

### Xbox Controller Button IDs

```java
// Buttons (joystick.button())
.a()        // Button 1
.b()        // Button 2  
.x()        // Button 3
.y()        // Button 4
.leftBumper()   // Button 5
.rightBumper()  // Button 6
.back()     // Button 7
.start()    // Button 8

// Axes (joystick.getAxis())
.getLeftX()     // Left stick X
.getLeftY()     // Left stick Y  
.getRightX()    // Right stick X
.getRightY()    // Right stick Y
.getLeftTriggerAxis()   // Left trigger
.getRightTriggerAxis()  // Right trigger
```

### Trigger Types

```java
// Button events
.onTrue()       // When button pressed
.onFalse()      // When button released
.whileTrue()    // While button held
.whileFalse()   // While button not held

// Combining triggers
.and()          // Both conditions true
.or()           // Either condition true
.negate()       // Opposite of condition
```

## Constants and Configuration

### Drive Constants

```java
// From RobotContainer
private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

// Deadbands (10% of max)
.withDeadband(MaxSpeed * 0.1)
.withRotationalDeadband(MaxAngularRate * 0.1)
```

### Simulation Constants

```java
// From CommandSwerveDrivetrain
private static final double kSimLoopPeriod = 0.005; // 5 ms simulation loop
```

### Alliance Perspective

```java
// From CommandSwerveDrivetrain  
private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
```