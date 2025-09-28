package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.swerve.SwerveModule;
import frc.robot.vision.MultiCameraLocalization;
import frc.robot.vision.FieldConfiguration;
import frc.robot.vision.FieldCalibrationValidator;
import frc.robot.vision.GyroscopeManager;
import java.util.ArrayList;

/**
 * VisionSubsystem integrates the vision localization system with the CTRE Phoenix 6 swerve drivetrain.
 * It provides robot pose estimation using PhotonVision and AprilTags, with automatic gyroscope detection
 * and integration with the CTRE Pigeon2.
 */
public class VisionSubsystem extends SubsystemBase {
    private final MultiCameraLocalization localization;
    private final FieldCalibrationValidator calibrationValidator;
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Initialize with automatic gyroscope detection (prefers CTRE Pigeon2)
        this.localization = new MultiCameraLocalization(drivetrain.getKinematics());

        // Alternative: Specify gyroscope type explicitly for CTRE systems
        // this.localization = new MultiCameraLocalization(
        //     drivetrain.getKinematics(),
        //     GyroscopeManager.GyroscopeType.PIGEON2_CAN  // Force Pigeon2 usage
        // );

        this.calibrationValidator = new FieldCalibrationValidator(
            new FieldConfiguration(),
            localization
        );

        System.out.println("VisionSubsystem: Initialized with CTRE swerve integration");
    }

    /**
     * Reset gyroscope heading (useful for field-centric control)
     */
    public Command resetGyroscope() {
        return Commands.runOnce(() -> {
            localization.resetGyroscope();
            System.out.println("VisionSubsystem: Gyroscope reset");
        });
    }

    /**
     * Get gyroscope status for dashboard monitoring
     */
    public boolean isGyroscopeHealthy() {
        return localization.isUsingGyroscope();
    }

    /**
     * Get current robot pose from the vision system
     */
    public Pose2d getCurrentPose() {
        return localization.getCurrentPose();
    }

    /**
     * Get visible AirTag count for navigation decisions
     */
    public int getVisibleAirTagCount() {
        return localization.getVisibleAirTagCount();
    }

    @Override
    public void periodic() {
        // Get module positions from CTRE swerve drivetrain
        SwerveModulePosition[] modulePositions = getCurrentModulePositions();

        // Update pose estimation with vision and odometry
        localization.updatePose(modulePositions, new ArrayList<>());

        // Add vision measurements to CTRE drivetrain if we have good vision pose
        Pose2d visionPose = localization.getCurrentPose();
        if (localization.getVisibleAirTagCount() >= 2) {
            try {
                // Add vision measurement to CTRE's pose estimator
                drivetrain.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
            } catch (Exception e) {
                System.err.println("VisionSubsystem: Failed to add vision measurement to CTRE drivetrain: " + e.getMessage());
            }
        }

        // Update dashboard monitoring
        localization.updateDashboard();
    }

    /**
     * Extract module positions from CTRE swerve modules
     */
    private SwerveModulePosition[] getCurrentModulePositions() {
        try {
            // Get modules from CTRE drivetrain
            SwerveModule[] modules = drivetrain.getModules();
            SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

            for (int i = 0; i < modules.length; i++) {
                SwerveModule module = modules[i];

                // Get drive position (distance traveled) and steer angle from CTRE module
                double drivePositionMeters = module.getDriveMotor().getPosition().getValueAsDouble();
                double steerAngleRadians = module.getSteerMotor().getPosition().getValueAsDouble();

                positions[i] = new SwerveModulePosition(
                    drivePositionMeters,
                    new edu.wpi.first.math.geometry.Rotation2d(steerAngleRadians)
                );
            }

            return positions;
        } catch (Exception e) {
            System.err.println("VisionSubsystem: Error getting module positions: " + e.getMessage());
            // Return default positions to prevent crashes
            return new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
        }
    }

    /**
     * Create a command to start field calibration validation
     */
    public Command startCalibrationValidation() {
        return Commands.runOnce(() -> {
            calibrationValidator.startCalibrationValidation();
            System.out.println("VisionSubsystem: Field calibration validation started");
        });
    }

    /**
     * Create a command to record a calibration point at a known position
     */
    public Command recordCalibrationPoint(Pose2d knownPose, String description) {
        return Commands.runOnce(() -> {
            calibrationValidator.recordCalibrationPoint(knownPose, description);
            System.out.printf("VisionSubsystem: Recorded calibration point: %s%n", description);
        });
    }

    /**
     * Create a command to print detailed calibration report
     */
    public Command printCalibrationReport() {
        return Commands.runOnce(() -> {
            calibrationValidator.printDetailedReport();
        });
    }

    /**
     * Get the MultiCameraLocalization system for advanced usage
     */
    public MultiCameraLocalization getLocalization() {
        return localization;
    }

    /**
     * Get the field calibration validator
     */
    public FieldCalibrationValidator getCalibrationValidator() {
        return calibrationValidator;
    }

    /**
     * Get the gyroscope type being used
     */
    public String getGyroscopeType() {
        return localization.getGyroscope().getGyroscopeType().name();
    }

    /**
     * Check if the vision system is providing reliable pose estimates
     */
    public boolean isVisionHealthy() {
        return localization.getVisibleAirTagCount() > 0 &&
               localization.getActiveCameraCount() > 0 &&
               !localization.getPerformanceMonitor().hasPerformanceIssues();
    }

    /**
     * Get system health status for diagnostics
     */
    public String getSystemHealthStatus() {
        if (isVisionHealthy() && isGyroscopeHealthy()) {
            return "HEALTHY";
        } else if (isVisionHealthy() || isGyroscopeHealthy()) {
            return "DEGRADED";
        } else {
            return "CRITICAL";
        }
    }

    /**
     * Manual enable/disable gyroscope usage
     */
    public Command setGyroscopeEnabled(boolean enabled) {
        return Commands.runOnce(() -> {
            localization.setUseGyroscope(enabled);
            System.out.printf("VisionSubsystem: Gyroscope usage set to %s%n", enabled);
        });
    }

    /**
     * Get detailed status for logging/debugging
     */
    public String getDetailedStatus() {
        return String.format(
            "VisionSubsystem{pose=(%.2f,%.2f,%.1f°), airtags=%d, cameras=%d, gyro=%s, health=%s}",
            getCurrentPose().getX(),
            getCurrentPose().getY(),
            getCurrentPose().getRotation().getDegrees(),
            getVisibleAirTagCount(),
            localization.getActiveCameraCount(),
            getGyroscopeType(),
            getSystemHealthStatus()
        );
    }
}