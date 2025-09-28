// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.navigation.AirTagNavigation;
import frc.robot.vision.FieldConfiguration;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Pneumatics;

public class RobotContainer {

    private final Pneumatics pneumatics = new Pneumatics();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    // Navigation system
    private final AirTagNavigation navigation = new AirTagNavigation(
        vision.getLocalization(),
        new FieldConfiguration(),
        this::createDriveCommand
    );

    public RobotContainer() {
        configureBindings();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // ===== PNEUMATICS CONTROLS =====
        joystick.a().onTrue(new InstantCommand(() -> pneumatics.setForward()));
        joystick.x().onTrue(new InstantCommand(() -> pneumatics.setOff()));

        // ===== VISION & NAVIGATION CONTROLS =====
        // Left trigger: Navigate to closest AirTag
        joystick.leftTrigger().onTrue(navigation.navigateToClosestAirTag());

        // Right trigger: Cancel navigation and emergency stop
        joystick.rightTrigger().onTrue(Commands.runOnce(() -> {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().cancelAll();
        }));

        // D-Pad navigation to specific AirTags
        new Trigger(() -> joystick.getHID().getPOV() == 0)   // UP
            .onTrue(navigation.navigateToAirTag(1));
        new Trigger(() -> joystick.getHID().getPOV() == 90)  // RIGHT
            .onTrue(navigation.navigateToAirTag(2));
        new Trigger(() -> joystick.getHID().getPOV() == 180) // DOWN
            .onTrue(navigation.navigateToAirTag(3));
        new Trigger(() -> joystick.getHID().getPOV() == 270) // LEFT
            .onTrue(navigation.navigateToAirTag(5));

        // ===== DRIVE CONTROLS =====
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Reset field-centric heading and gyroscope
        joystick.leftBumper().onTrue(Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
            vision.resetGyroscope()
        ));

        // ===== PNEUMATICS WITH MODIFIER =====
        // Right bumper + Y/X for pneumatics controls (moved to avoid conflicts)
        joystick.rightBumper().and(joystick.y()).onTrue(new InstantCommand(() -> pneumatics.enableCompressor()));
        joystick.rightBumper().and(joystick.x()).onTrue(new InstantCommand(() -> pneumatics.disableCompressor()));

        // ===== SYSID CONTROLS =====
        // Run SysId routines when holding back/start and X/Y.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ===== VISION CALIBRATION CONTROLS =====
        // Start + left bumper: Start calibration validation
        joystick.start().and(joystick.leftBumper()).onTrue(vision.startCalibrationValidation());

        // Back + left bumper: Print calibration report
        joystick.back().and(joystick.leftBumper()).onTrue(vision.printCalibrationReport());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command createDriveCommand() {
        // Return a command that can be used by navigation system
        return drivetrain.applyRequest(() ->
            new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.Velocity)
        );
    }

    public Command getAutonomousCommand() {
        // Enhanced autonomous with vision navigation
        return Commands.sequence(
            Commands.print("Starting autonomous with AirTag navigation"),

            // Navigate to speaker AirTag (ID 5)
            navigation.navigateToAirTag(5),
            Commands.waitSeconds(2.0),

            // Navigate to alliance station AirTag (ID 1)
            navigation.navigateToAirTag(1),
            Commands.waitSeconds(2.0),

            // Return to closest AirTag for endgame
            navigation.navigateToClosestAirTag(),

            Commands.print("Autonomous complete")
        );
    }

    /**
     * Emergency autonomous fallback (if vision fails)
     */
    public Command getEmergencyAutonomous() {
        return Commands.sequence(
            Commands.print("Emergency autonomous - limited vision available"),

            // Basic mobility autonomous using CTRE swerve
            drivetrain.applyRequest(() ->
                new SwerveRequest.FieldCentric()
                    .withVelocityX(1.0)  // Drive forward 1 m/s
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(2.0),  // Drive forward for 2 seconds

            drivetrain.applyRequest(() ->
                new SwerveRequest.SwerveDriveBrake()  // Stop
            ),

            Commands.print("Emergency autonomous complete")
        );
    }

    // Getters for subsystems
    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public VisionSubsystem getVision() { return vision; }
    public AirTagNavigation getNavigation() { return navigation; }
    public Pneumatics getPneumatics() { return pneumatics; }
}
