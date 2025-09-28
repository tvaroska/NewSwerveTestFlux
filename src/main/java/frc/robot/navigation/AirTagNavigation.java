package frc.robot.navigation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.FieldConfiguration;
import frc.robot.vision.MultiCameraLocalization;
import java.util.*;
import java.util.function.Supplier;

public class AirTagNavigation {

    private final MultiCameraLocalization localization;
    private final FieldConfiguration fieldConfig;
    private final Supplier<Command> driveCommandFactory;
    private final NavigationErrorRecovery errorRecovery;

    // Navigation parameters
    private static final double TARGET_DISTANCE = 1.0;  // 1 meter in front of AirTag
    private static final double POSITION_TOLERANCE = 0.1;  // 10cm tolerance
    private static final double ANGLE_TOLERANCE = Math.toRadians(5);  // 5 degree tolerance
    private static final double APPROACH_SPEED = 1.0;  // m/s approach speed
    private static final double FINAL_APPROACH_SPEED = 0.3;  // m/s final precision speed

    // Controllers for final approach
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    // Navigation state tracking
    private int currentTargetAirTag = -1;
    private double navigationStartTime = 0.0;
    private Pose2d lastValidPose = null;
    private int recoveryAttempts = 0;

    public AirTagNavigation(MultiCameraLocalization localization,
                           FieldConfiguration fieldConfig,
                           Supplier<Command> driveCommandFactory) {
        this.localization = localization;
        this.fieldConfig = fieldConfig;
        this.driveCommandFactory = driveCommandFactory;
        this.errorRecovery = new NavigationErrorRecovery(localization, fieldConfig);

        // PID controllers for precise final positioning
        this.xController = new PIDController(2.0, 0.0, 0.1);
        this.yController = new PIDController(2.0, 0.0, 0.1);
        this.rotationController = new PIDController(3.0, 0.0, 0.2);

        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotationController.setTolerance(ANGLE_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a command to navigate to a position in front of the specified AirTag
     * @param airTagId The ID of the target AirTag
     * @return Command that navigates to the AirTag
     */
    public Command navigateToAirTag(int airTagId) {
        return Commands.sequence(
            // Initialize navigation
            Commands.runOnce(() -> {
                currentTargetAirTag = airTagId;
                navigationStartTime = Timer.getFPGATimestamp();
                lastValidPose = localization.getCurrentPose();
                recoveryAttempts = 0;
                SmartDashboard.putString("Navigation", "Planning path to AirTag " + airTagId);
            }),

            // Pre-navigation health check
            createNavigationHealthCheck(),

            // Phase 1: Path planning and rough approach with error recovery
            createRobustPathToAirTag(airTagId),

            // Phase 2: Vision-based final approach with error recovery
            createRobustFinalApproach(airTagId),

            Commands.runOnce(() -> {
                SmartDashboard.putString("Navigation", "Arrived at AirTag " + airTagId);
                currentTargetAirTag = -1;
            })
        );
    }

    private Command createNavigationHealthCheck() {
        return Commands.either(
            Commands.print("Navigation health check passed"),

            // If health check fails, attempt recovery
            Commands.sequence(
                Commands.print("Navigation health check failed - attempting recovery"),
                createRecoveryCommand(),
                // Retry health check once
                Commands.either(
                    Commands.print("Recovery successful"),
                    Commands.sequence(
                        Commands.print("Recovery failed - navigation may be unreliable"),
                        Commands.runOnce(() -> SmartDashboard.putString("Navigation_Warning", "Degraded mode"))
                    ),
                    () -> errorRecovery.isLocalizationHealthy()
                )
            ),

            () -> errorRecovery.isLocalizationHealthy()
        );
    }

    private Command createRobustPathToAirTag(int airTagId) {
        return Commands.race(
            // Main navigation command
            createPathToAirTag(airTagId),

            // Timeout and error monitoring
            Commands.sequence(
                Commands.waitSeconds(15.0), // Navigation timeout
                Commands.runOnce(() -> {
                    SmartDashboard.putString("Navigation_Error", "Path navigation timeout");
                    recoveryAttempts++;
                }),
                createRecoveryCommand()
            )
        );
    }

    private Command createRobustFinalApproach(int airTagId) {
        return Commands.race(
            // Main final approach
            createFinalApproachWithMonitoring(airTagId),

            // Error monitoring and recovery
            Commands.sequence(
                Commands.waitSeconds(10.0), // Final approach timeout
                Commands.runOnce(() -> {
                    SmartDashboard.putString("Navigation_Error", "Final approach timeout");
                    recoveryAttempts++;
                }),
                createRecoveryCommand()
            )
        );
    }

    private Command createRecoveryCommand() {
        return Commands.defer(() -> {
            Pose2d currentPose = localization.getCurrentPose();
            Pose2d targetPose = calculateTargetPose(currentTargetAirTag);
            double navigationTime = Timer.getFPGATimestamp() - navigationStartTime;
            double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

            NavigationErrorRecovery.RecoveryContext context =
                new NavigationErrorRecovery.RecoveryContext(
                    currentPose, targetPose, navigationTime,
                    0.0, // timeSinceLastVision - would need tracking
                    distanceToTarget, recoveryAttempts
                );

            return errorRecovery.createRecoveryCommand(context);
        }, Set.of());
    }

    /**
     * Calculate the target pose in front of an AirTag
     * @param airTagId Target AirTag ID
     * @return Pose2d representing where robot should be positioned
     */
    public Pose2d calculateTargetPose(int airTagId) {
        Pose3d airtagPose = fieldConfig.getAirTagPose(airTagId);
        if (airtagPose == null) {
            throw new IllegalArgumentException("AirTag " + airTagId + " not found in field configuration");
        }

        // AirTag position and orientation
        Translation2d airtagPosition = airtagPose.toPose2d().getTranslation();
        Rotation2d airtagFacing = airtagPose.toPose2d().getRotation();

        // Calculate position TARGET_DISTANCE meters in front of AirTag
        // "In front" means opposite to the direction the AirTag is facing
        Rotation2d approachDirection = airtagFacing.plus(Rotation2d.fromRadians(Math.PI));

        Translation2d targetPosition = airtagPosition.plus(
            new Translation2d(TARGET_DISTANCE, approachDirection)
        );

        // Robot should face toward the AirTag
        Rotation2d targetRotation = airtagFacing;

        return new Pose2d(targetPosition, targetRotation);
    }

    /**
     * Creates a command for path planning and rough approach
     */
    private Command createPathToAirTag(int airTagId) {
        return Commands.runOnce(() -> {
            Pose2d currentPose = localization.getCurrentPose();
            Pose2d targetPose = calculateTargetPose(airTagId);

            SmartDashboard.putNumber("Target X", targetPose.getX());
            SmartDashboard.putNumber("Target Y", targetPose.getY());
            SmartDashboard.putNumber("Target Angle", targetPose.getRotation().getDegrees());

            // Create trajectory to target
            TrajectoryConfig config = new TrajectoryConfig(APPROACH_SPEED, 1.0);

            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose,
                List.of(), // No interior waypoints for simple approach
                targetPose,
                config
            );

            // Execute trajectory (would integrate with your swerve drive command)
            // This is a placeholder - replace with your actual trajectory following
            executeTrajectory(trajectory);
        });
    }

    /**
     * Creates a command for vision-based final approach with monitoring
     */
    private Command createFinalApproachWithMonitoring(int airTagId) {
        return Commands.run(() -> {
            try {
                Pose2d currentPose = localization.getCurrentPose();
                Pose2d targetPose = calculateTargetPose(airTagId);

                // Validate poses before proceeding
                if (!isValidNavigationPose(currentPose) || !isValidNavigationPose(targetPose)) {
                    SmartDashboard.putString("Navigation_Error", "Invalid pose during final approach");
                    return;
                }

                // Calculate control outputs
                double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
                double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
                double rotOutput = rotationController.calculate(
                    currentPose.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

                // Limit speeds for final approach
                xOutput = Math.max(-FINAL_APPROACH_SPEED, Math.min(FINAL_APPROACH_SPEED, xOutput));
                yOutput = Math.max(-FINAL_APPROACH_SPEED, Math.min(FINAL_APPROACH_SPEED, yOutput));

                // Convert field-relative speeds to robot-relative
                ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xOutput, yOutput, rotOutput);
                ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds,
                    currentPose.getRotation()
                );

                // Send speeds to drive subsystem
                executeChassisSpeeds(robotRelativeSpeeds);

                // Update progress tracking
                updateNavigationProgress(currentPose, targetPose);

            } catch (Exception e) {
                SmartDashboard.putString("Navigation_Error", "Exception in final approach: " + e.getMessage());
                System.err.println("Final approach error: " + e.getMessage());
            }
        }).until(() -> isAtTargetWithValidation(airTagId));
    }

    /**
     * Creates a command for vision-based final approach (legacy method)
     */
    private Command createFinalApproach(int airTagId) {
        return createFinalApproachWithMonitoring(airTagId);
    }

    private boolean isValidNavigationPose(Pose2d pose) {
        if (pose == null) return false;

        Translation2d translation = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        // Check for NaN or infinite values
        if (Double.isNaN(translation.getX()) || Double.isInfinite(translation.getX()) ||
            Double.isNaN(translation.getY()) || Double.isInfinite(translation.getY()) ||
            Double.isNaN(rotation.getRadians()) || Double.isInfinite(rotation.getRadians())) {
            return false;
        }

        // Check field bounds with margin
        double x = translation.getX();
        double y = translation.getY();
        return x >= -1.0 && x <= 17.0 && y >= -1.0 && y <= 9.0;
    }

    private void updateNavigationProgress(Pose2d currentPose, Pose2d targetPose) {
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        SmartDashboard.putNumber("Distance to Target", distance);
        SmartDashboard.putNumber("Angle Error", Math.toDegrees(angleError));

        // Track if robot is making progress
        if (lastValidPose != null) {
            double lastDistance = lastValidPose.getTranslation().getDistance(targetPose.getTranslation());
            boolean makingProgress = distance < lastDistance - 0.01; // 1cm improvement threshold

            SmartDashboard.putBoolean("Making_Progress", makingProgress);

            if (!makingProgress && distance > 0.2) {
                // Robot might be stuck
                SmartDashboard.putString("Navigation_Warning", "Robot may be stuck");
            }
        }

        lastValidPose = currentPose;
    }

    private boolean isAtTargetWithValidation(int airTagId) {
        try {
            return isAtTarget(airTagId);
        } catch (Exception e) {
            SmartDashboard.putString("Navigation_Error", "Error checking target arrival: " + e.getMessage());
            return false;
        }
    }

    /**
     * Check if robot is at the target position
     */
    public boolean isAtTarget(int airTagId) {
        Pose2d currentPose = localization.getCurrentPose();
        Pose2d targetPose = calculateTargetPose(airTagId);

        boolean positionReached = xController.atSetpoint() && yController.atSetpoint();
        boolean orientationReached = rotationController.atSetpoint();

        SmartDashboard.putBoolean("Position Reached", positionReached);
        SmartDashboard.putBoolean("Orientation Reached", orientationReached);

        return positionReached && orientationReached;
    }

    /**
     * Get all reachable AirTags from current position
     */
    public List<Integer> getReachableAirTags() {
        Pose2d currentPose = localization.getCurrentPose();
        List<Integer> reachable = new ArrayList<>();

        for (Map.Entry<Integer, Pose3d> entry : fieldConfig.getAirTagPositions().entrySet()) {
            int airTagId = entry.getKey();
            Pose2d targetPose = calculateTargetPose(airTagId);

            // Check if target is within field bounds and reachable
            if (isValidTargetPose(targetPose)) {
                reachable.add(airTagId);
            }
        }

        return reachable;
    }

    /**
     * Find the closest AirTag to current robot position
     */
    public int findClosestAirTag() {
        Pose2d currentPose = localization.getCurrentPose();
        double minDistance = Double.MAX_VALUE;
        int closestId = -1;

        for (int airTagId : getReachableAirTags()) {
            Pose2d targetPose = calculateTargetPose(airTagId);
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                closestId = airTagId;
            }
        }

        return closestId;
    }

    /**
     * Create a command to navigate to the closest AirTag
     */
    public Command navigateToClosestAirTag() {
        return Commands.either(
            navigateToAirTag(findClosestAirTag()),
            Commands.print("No reachable AirTags found"),
            () -> findClosestAirTag() != -1
        );
    }

    private boolean isValidTargetPose(Pose2d pose) {
        // Check field boundaries (adjust for your field size)
        double x = pose.getX();
        double y = pose.getY();

        return x >= 0.5 && x <= 16.0 && y >= 0.5 && y <= 7.7;
    }

    // Placeholder methods - replace with your actual drive implementation
    private void executeTrajectory(Trajectory trajectory) {
        SmartDashboard.putString("Navigation", "Executing trajectory to target");
        // Integrate with your swerve drive trajectory following
        // Example: swerveSubsystem.followTrajectory(trajectory);
    }

    private void executeChassisSpeeds(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("Drive Vx", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive Vy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive Omega", speeds.omegaRadiansPerSecond);
        // Send to your swerve drive subsystem
        // Example: swerveSubsystem.drive(speeds);
    }

    /**
     * Enhanced navigation status for dashboard
     */
    public void updateDashboard() {
        try {
            Pose2d currentPose = localization.getCurrentPose();
            List<Integer> reachable = getReachableAirTags();

            // Basic navigation status
            SmartDashboard.putNumber("Reachable AirTags", reachable.size());
            SmartDashboard.putString("Reachable IDs", reachable.toString());

            // Current target information
            if (currentTargetAirTag != -1) {
                SmartDashboard.putNumber("Current_Target_AirTag", currentTargetAirTag);
                SmartDashboard.putNumber("Navigation_Time",
                    Timer.getFPGATimestamp() - navigationStartTime);
                SmartDashboard.putNumber("Recovery_Attempts", recoveryAttempts);
            } else {
                SmartDashboard.putNumber("Current_Target_AirTag", -1);
                SmartDashboard.putNumber("Navigation_Time", 0);
                SmartDashboard.putNumber("Recovery_Attempts", 0);
            }

            // Closest AirTag analysis
            if (!reachable.isEmpty()) {
                int closest = findClosestAirTag();
                SmartDashboard.putNumber("Closest AirTag", closest);

                if (closest != -1) {
                    Pose2d targetPose = calculateTargetPose(closest);
                    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                    SmartDashboard.putNumber("Distance to Closest", distance);

                    // Navigation readiness assessment
                    boolean readyToNavigate = assessNavigationReadiness();
                    SmartDashboard.putBoolean("Ready_To_Navigate", readyToNavigate);

                    if (!readyToNavigate) {
                        SmartDashboard.putString("Navigation_Blocker", getNavigationBlocker());
                    } else {
                        SmartDashboard.putString("Navigation_Blocker", "None");
                    }
                }
            }

            // System health for navigation
            SmartDashboard.putBoolean("Navigation_System_Healthy",
                errorRecovery.isLocalizationHealthy());

            // Update error recovery dashboard
            errorRecovery.updateDashboard();

        } catch (Exception e) {
            SmartDashboard.putString("Navigation_Dashboard_Error", e.getMessage());
            System.err.println("Navigation dashboard update failed: " + e.getMessage());
        }
    }

    private boolean assessNavigationReadiness() {
        // Check localization health
        if (!errorRecovery.isLocalizationHealthy()) {
            return false;
        }

        // Check if we have any reachable AirTags
        if (getReachableAirTags().isEmpty()) {
            return false;
        }

        // Check if current pose is valid
        Pose2d currentPose = localization.getCurrentPose();
        if (!isValidNavigationPose(currentPose)) {
            return false;
        }

        // Check if we're not already in recovery mode
        if (recoveryAttempts > 0) {
            return false;
        }

        return true;
    }

    private String getNavigationBlocker() {
        if (!errorRecovery.isLocalizationHealthy()) {
            return "Localization unhealthy";
        }

        if (getReachableAirTags().isEmpty()) {
            return "No reachable AirTags";
        }

        Pose2d currentPose = localization.getCurrentPose();
        if (!isValidNavigationPose(currentPose)) {
            return "Invalid current pose";
        }

        if (recoveryAttempts > 0) {
            return "In recovery mode";
        }

        return "Unknown blocker";
    }

    /**
     * Get comprehensive navigation status
     */
    public NavigationStatus getNavigationStatus() {
        return new NavigationStatus(
            currentTargetAirTag,
            currentTargetAirTag != -1 ? Timer.getFPGATimestamp() - navigationStartTime : 0,
            recoveryAttempts,
            errorRecovery.isLocalizationHealthy(),
            assessNavigationReadiness(),
            getReachableAirTags().size(),
            findClosestAirTag()
        );
    }

    public static class NavigationStatus {
        public final int currentTarget;
        public final double navigationTime;
        public final int recoveryAttempts;
        public final boolean localizationHealthy;
        public final boolean readyToNavigate;
        public final int reachableAirTags;
        public final int closestAirTag;

        public NavigationStatus(int currentTarget, double navigationTime, int recoveryAttempts,
                              boolean localizationHealthy, boolean readyToNavigate,
                              int reachableAirTags, int closestAirTag) {
            this.currentTarget = currentTarget;
            this.navigationTime = navigationTime;
            this.recoveryAttempts = recoveryAttempts;
            this.localizationHealthy = localizationHealthy;
            this.readyToNavigate = readyToNavigate;
            this.reachableAirTags = reachableAirTags;
            this.closestAirTag = closestAirTag;
        }

        @Override
        public String toString() {
            return String.format("NavigationStatus{target=%d, time=%.1fs, recovery=%d, healthy=%s, ready=%s, reachable=%d, closest=%d}",
                currentTarget, navigationTime, recoveryAttempts, localizationHealthy, readyToNavigate, reachableAirTags, closestAirTag);
        }
    }
}