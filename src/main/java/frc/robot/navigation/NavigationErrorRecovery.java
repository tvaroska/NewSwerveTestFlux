package frc.robot.navigation;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.vision.FieldConfiguration;
import frc.robot.vision.MultiCameraLocalization;
import java.util.*;

public class NavigationErrorRecovery {

    private final MultiCameraLocalization localization;
    private final FieldConfiguration fieldConfig;
    private final Map<String, Integer> errorCounts;
    private final List<RecoveryAction> recoveryHistory;

    // Recovery thresholds
    private static final double LOCALIZATION_TIMEOUT = 5.0; // seconds
    private static final double NAVIGATION_TIMEOUT = 30.0; // seconds
    private static final double POSE_VALIDITY_THRESHOLD = 0.5; // meters
    private static final int MAX_RECOVERY_ATTEMPTS = 3;
    private static final double VISION_LOSS_TIMEOUT = 2.0; // seconds

    public NavigationErrorRecovery(MultiCameraLocalization localization, FieldConfiguration fieldConfig) {
        this.localization = localization;
        this.fieldConfig = fieldConfig;
        this.errorCounts = new HashMap<>();
        this.recoveryHistory = new ArrayList<>();
    }

    public boolean isLocalizationHealthy() {
        try {
            Pose2d currentPose = localization.getCurrentPose();

            // Check if pose is valid
            if (!isValidPose(currentPose)) {
                recordError("invalid_pose");
                return false;
            }

            // Check gyroscope health (critical for navigation stability)
            if (!isGyroscopeHealthy()) {
                recordError("gyroscope_unhealthy");
                return false;
            }

            // Check if we have recent vision updates
            int visibleTags = localization.getVisibleAirTagCount();
            if (visibleTags == 0) {
                recordError("no_vision");
                return false;
            }

            // Check camera health
            int activeCameras = localization.getActiveCameraCount();
            if (activeCameras == 0) {
                recordError("no_cameras");
                return false;
            }

            // Check for performance issues
            if (localization.getPerformanceMonitor().hasPerformanceIssues()) {
                recordError("performance_issues");
                return false;
            }

            return true;
        } catch (Exception e) {
            recordError("localization_exception");
            System.err.println("Localization health check failed: " + e.getMessage());
            return false;
        }
    }

    /**
     * Check gyroscope health specifically for navigation
     */
    private boolean isGyroscopeHealthy() {
        try {
            // If gyroscope is disabled, we can still navigate (just less accurately)
            if (!localization.isUsingGyroscope()) {
                return true; // Not using gyro is OK, just note it
            }

            // Check if gyroscope is providing reasonable data
            edu.wpi.first.math.geometry.Rotation2d gyroReading = localization.getGyroscopeRotation();
            if (gyroReading == null) {
                return false;
            }

            // Check angular velocity is reasonable (not stuck or spinning wildly)
            double angularVel = localization.getAngularVelocity();
            if (!Double.isFinite(angularVel) || Math.abs(angularVel) > 720.0) { // 720 deg/sec max
                return false;
            }

            return true;
        } catch (Exception e) {
            System.err.println("Gyroscope health check failed: " + e.getMessage());
            return false;
        }
    }

    public Command createRecoveryCommand(RecoveryContext context) {
        RecoveryStrategy strategy = determineRecoveryStrategy(context);

        SmartDashboard.putString("Recovery_Strategy", strategy.name());
        System.out.println("Executing recovery strategy: " + strategy.name());

        switch (strategy) {
            case WAIT_FOR_VISION:
                return createWaitForVisionCommand();

            case ROTATE_TO_SCAN:
                return createRotateToScanCommand();

            case MOVE_TO_KNOWN_POSITION:
                return createMoveToKnownPositionCommand(context);

            case EMERGENCY_STOP:
                return createEmergencyStopCommand();

            case RESET_LOCALIZATION:
                return createResetLocalizationCommand();

            case RESET_GYROSCOPE:
                return createResetGyroscopeCommand();

            default:
                return Commands.print("Unknown recovery strategy");
        }
    }

    private RecoveryStrategy determineRecoveryStrategy(RecoveryContext context) {
        // Priority-based recovery strategy selection

        // Critical issues first
        if (context.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
            return RecoveryStrategy.EMERGENCY_STOP;
        }

        if (context.navigationTime > NAVIGATION_TIMEOUT) {
            return RecoveryStrategy.EMERGENCY_STOP;
        }

        // Localization issues
        if (!isValidPose(context.currentPose)) {
            return RecoveryStrategy.RESET_LOCALIZATION;
        }

        // Gyroscope issues
        if (!isGyroscopeHealthy()) {
            return RecoveryStrategy.RESET_GYROSCOPE;
        }

        if (localization.getVisibleAirTagCount() == 0) {
            if (context.timeSinceLastVision > VISION_LOSS_TIMEOUT) {
                return RecoveryStrategy.ROTATE_TO_SCAN;
            } else {
                return RecoveryStrategy.WAIT_FOR_VISION;
            }
        }

        // Navigation stuck
        if (context.distanceToTarget > 0.5 && context.navigationTime > 15.0) {
            return RecoveryStrategy.MOVE_TO_KNOWN_POSITION;
        }

        // Default: wait briefly and retry
        return RecoveryStrategy.WAIT_FOR_VISION;
    }

    private Command createWaitForVisionCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Recovery_Action", "Waiting for vision")),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> recordRecoveryAction("wait_for_vision", true))
        );
    }

    private Command createRotateToScanCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Recovery_Action", "Rotating to scan for AirTags")),

            // Rotate 360 degrees slowly to scan for AirTags
            Commands.run(() -> {
                // This would be implemented with your drive subsystem
                // Example: drive.rotate(Math.toRadians(10)); // 10 degrees per cycle
                SmartDashboard.putString("Drive_Mode", "Scanning rotation");
            }).withTimeout(6.0), // 6 seconds for full rotation

            Commands.runOnce(() -> {
                boolean success = localization.getVisibleAirTagCount() > 0;
                recordRecoveryAction("rotate_to_scan", success);
                if (success) {
                    SmartDashboard.putString("Recovery_Result", "AirTags found");
                } else {
                    SmartDashboard.putString("Recovery_Result", "No AirTags found");
                }
            })
        );
    }

    private Command createMoveToKnownPositionCommand(RecoveryContext context) {
        // Move to center of field or other known good position
        Pose2d fallbackPosition = new Pose2d(8.27, 4.1, new Rotation2d()); // Field center

        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Recovery_Action", "Moving to known position")),

            // This would use your path planning to move to a known good position
            Commands.runOnce(() -> {
                SmartDashboard.putString("Target_Recovery_Position",
                    String.format("(%.1f, %.1f)", fallbackPosition.getX(), fallbackPosition.getY()));
            }),

            Commands.waitSeconds(2.0), // Placeholder for actual movement

            Commands.runOnce(() -> recordRecoveryAction("move_to_known_position", true))
        );
    }

    private Command createEmergencyStopCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                SmartDashboard.putString("Recovery_Action", "EMERGENCY STOP");
                SmartDashboard.putBoolean("Emergency_Stop", true);
                System.err.println("EMERGENCY STOP: Navigation recovery failed");
            }),

            // Stop all motion
            Commands.runOnce(() -> {
                // This would stop your drive subsystem
                // Example: driveSubsystem.stop();
            }),

            Commands.runOnce(() -> recordRecoveryAction("emergency_stop", true))
        );
    }

    private Command createResetLocalizationCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Recovery_Action", "Resetting localization")),

            Commands.runOnce(() -> {
                try {
                    // Reset to a known position (would need to be implemented)
                    System.out.println("Resetting pose estimator to field center");
                    SmartDashboard.putBoolean("Localization_Reset", true);
                } catch (Exception e) {
                    System.err.println("Failed to reset localization: " + e.getMessage());
                }
            }),

            Commands.waitSeconds(1.0),

            Commands.runOnce(() -> recordRecoveryAction("reset_localization", true))
        );
    }

    private Command createResetGyroscopeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Recovery_Action", "Resetting gyroscope")),

            Commands.runOnce(() -> {
                try {
                    // Reset gyroscope to current heading
                    localization.resetGyroscope();
                    SmartDashboard.putBoolean("Gyroscope_Reset", true);
                    System.out.println("Gyroscope reset successful");
                } catch (Exception e) {
                    System.err.println("Failed to reset gyroscope: " + e.getMessage());
                }
            }),

            Commands.waitSeconds(2.0), // Give gyroscope time to stabilize

            Commands.runOnce(() -> recordRecoveryAction("reset_gyroscope", true))
        );
    }

    private void recordError(String errorType) {
        errorCounts.merge(errorType, 1, Integer::sum);
        SmartDashboard.putNumber("Error_" + errorType, errorCounts.get(errorType));
    }

    private void recordRecoveryAction(String action, boolean success) {
        recoveryHistory.add(new RecoveryAction(action, success, Timer.getFPGATimestamp()));

        // Keep only recent history
        double now = Timer.getFPGATimestamp();
        recoveryHistory.removeIf(ra -> now - ra.timestamp > 300.0); // 5 minutes

        SmartDashboard.putNumber("Recovery_Actions_Total", recoveryHistory.size());

        long successCount = recoveryHistory.stream()
            .filter(ra -> ra.success)
            .count();

        double successRate = recoveryHistory.isEmpty() ? 0.0 : (double) successCount / recoveryHistory.size();
        SmartDashboard.putNumber("Recovery_Success_Rate", successRate * 100);
    }

    private boolean isValidPose(Pose2d pose) {
        if (pose == null) return false;

        Translation2d translation = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        // Check for NaN or infinite values
        if (Double.isNaN(translation.getX()) || Double.isInfinite(translation.getX()) ||
            Double.isNaN(translation.getY()) || Double.isInfinite(translation.getY()) ||
            Double.isNaN(rotation.getRadians()) || Double.isInfinite(rotation.getRadians())) {
            return false;
        }

        // Check field bounds
        double x = translation.getX();
        double y = translation.getY();
        return x >= -1.0 && x <= 17.0 && y >= -1.0 && y <= 9.0;
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("Localization_Healthy", isLocalizationHealthy());

        // Show error counts
        int totalErrors = errorCounts.values().stream().mapToInt(Integer::intValue).sum();
        SmartDashboard.putNumber("Total_Navigation_Errors", totalErrors);

        // Show most common error
        String mostCommonError = errorCounts.entrySet().stream()
            .max(Map.Entry.comparingByValue())
            .map(Map.Entry::getKey)
            .orElse("None");
        SmartDashboard.putString("Most_Common_Error", mostCommonError);
    }

    public enum RecoveryStrategy {
        WAIT_FOR_VISION,
        ROTATE_TO_SCAN,
        MOVE_TO_KNOWN_POSITION,
        RESET_LOCALIZATION,
        RESET_GYROSCOPE,
        EMERGENCY_STOP
    }

    public static class RecoveryContext {
        public final Pose2d currentPose;
        public final Pose2d targetPose;
        public final double navigationTime;
        public final double timeSinceLastVision;
        public final double distanceToTarget;
        public final int recoveryAttempts;

        public RecoveryContext(Pose2d currentPose, Pose2d targetPose, double navigationTime,
                             double timeSinceLastVision, double distanceToTarget, int recoveryAttempts) {
            this.currentPose = currentPose;
            this.targetPose = targetPose;
            this.navigationTime = navigationTime;
            this.timeSinceLastVision = timeSinceLastVision;
            this.distanceToTarget = distanceToTarget;
            this.recoveryAttempts = recoveryAttempts;
        }
    }

    private static class RecoveryAction {
        public final String action;
        public final boolean success;
        public final double timestamp;

        public RecoveryAction(String action, boolean success, double timestamp) {
            this.action = action;
            this.success = success;
            this.timestamp = timestamp;
        }
    }
}