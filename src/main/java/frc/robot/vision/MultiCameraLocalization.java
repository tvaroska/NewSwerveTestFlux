package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.*;
import java.util.stream.Collectors;

public class MultiCameraLocalization {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;
    private final FieldConfiguration fieldConfig;
    private final PoseHistory poseHistory;
    private final ObservationFilter observationFilter;
    private final Map<Integer, PhotonPoseEstimator> poseEstimators;
    private final Map<Integer, PhotonCamera> cameras;
    private final PerformanceMonitor performanceMonitor;
    private final GyroscopeManager gyroscope;

    private static final double MAX_VELOCITY = 4.0; // m/s
    private static final double MAX_ANGULAR_VELOCITY = Math.PI; // rad/s
    private static final int HISTORY_SIZE = 50;

    // Gyroscope-based improvements
    private boolean useGyroscope = true;
    private Rotation2d lastGyroReading = new Rotation2d();
    private double gyroFailureTime = 0.0;

    public MultiCameraLocalization(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
        this.gyroscope = new GyroscopeManager();

        // Initialize pose estimator with gyroscope reading
        Rotation2d initialRotation = gyroscope.isHealthy() ? gyroscope.getRotation() : new Rotation2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            initialRotation,
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d()
        );

        this.fieldConfig = new FieldConfiguration();
        this.poseHistory = new PoseHistory(HISTORY_SIZE);
        this.observationFilter = new ObservationFilter();
        this.poseEstimators = new HashMap<>();
        this.cameras = new HashMap<>();
        this.performanceMonitor = new PerformanceMonitor();

        try {
            initializeCameras();
        } catch (Exception e) {
            System.err.println("Failed to initialize cameras: " + e.getMessage());
            e.printStackTrace();
            // Continue with limited functionality - vision will be disabled
        }

        // Log gyroscope status
        if (gyroscope.isHealthy()) {
            System.out.println("MultiCameraLocalization: Gyroscope initialized successfully");
        } else {
            System.out.println("MultiCameraLocalization: Gyroscope not available, using odometry-only mode");
            useGyroscope = false;
        }
    }

    /**
     * Constructor with specific gyroscope type (for testing/configuration)
     */
    public MultiCameraLocalization(SwerveDriveKinematics kinematics, GyroscopeManager.GyroscopeType gyroType) {
        this.kinematics = kinematics;
        this.gyroscope = new GyroscopeManager(gyroType);

        // Initialize pose estimator with gyroscope reading
        Rotation2d initialRotation = gyroscope.isHealthy() ? gyroscope.getRotation() : new Rotation2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            initialRotation,
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d()
        );

        this.fieldConfig = new FieldConfiguration();
        this.poseHistory = new PoseHistory(HISTORY_SIZE);
        this.observationFilter = new ObservationFilter();
        this.poseEstimators = new HashMap<>();
        this.cameras = new HashMap<>();
        this.performanceMonitor = new PerformanceMonitor();

        try {
            initializeCameras();
        } catch (Exception e) {
            System.err.println("Failed to initialize cameras: " + e.getMessage());
            e.printStackTrace();
        }

        useGyroscope = gyroscope.isHealthy();
        System.out.printf("MultiCameraLocalization: Gyroscope type %s, healthy: %s%n",
            gyroType, useGyroscope);
    }

    public void updatePose(SwerveModulePosition[] modulePositions, List<AirTagObservation> observations) {
        try (var timing = performanceMonitor.startTiming("updatePose")) {
            double timestamp = Timer.getFPGATimestamp();

            // Validate inputs
            if (modulePositions == null || modulePositions.length != 4) {
                performanceMonitor.recordError("localization", "Invalid module positions array");
                return;
            }

            // Validate module positions
            for (int i = 0; i < modulePositions.length; i++) {
                if (modulePositions[i] == null) {
                    performanceMonitor.recordError("localization", "Null module position at index " + i);
                    return;
                }
                if (!isValidModulePosition(modulePositions[i])) {
                    performanceMonitor.recordWarning("localization", "Suspicious module position at index " + i);
                }
            }

            try {
                // Get rotation - prefer gyroscope if available and healthy
                Rotation2d currentRotation = getCurrentRotation(modulePositions, timestamp);

                // Update odometry
                poseEstimator.update(currentRotation, modulePositions);
                Pose2d odometryPose = poseEstimator.getEstimatedPosition();

                // Validate odometry pose
                if (!isValidPose(odometryPose)) {
                    performanceMonitor.recordError("localization", "Invalid odometry pose: " + odometryPose);
                    return;
                }

                poseHistory.addPose(odometryPose, timestamp);

                // Process camera results using PhotonLib
                int validVisionUpdates = 0;
                int cameraFailures = 0;

                for (Map.Entry<Integer, PhotonCamera> entry : cameras.entrySet()) {
                    int cameraId = entry.getKey();
                    PhotonCamera camera = entry.getValue();
                    PhotonPoseEstimator estimator = poseEstimators.get(cameraId);

                    if (estimator == null) {
                        performanceMonitor.recordWarning("vision", "No estimator for camera " + cameraId);
                        continue;
                    }

                    try (var visionTiming = performanceMonitor.startTiming("vision_camera_" + cameraId)) {
                        PhotonPipelineResult result = camera.getAllUnreadResults().stream().findFirst().orElse(null);

                        if (result == null) {
                            cameraFailures++;
                            performanceMonitor.recordError("vision", "Null result from camera " + cameraId);
                            continue;
                        }

                        if (result.hasTargets()) {
                            Optional<EstimatedRobotPose> poseResult = estimator.update(result);

                            if (poseResult.isPresent()) {
                                EstimatedRobotPose estimatedPose = poseResult.get();

                                if (isPhotonPoseValid(estimatedPose, odometryPose)) {
                                    try {
                                        // Calculate uncertainty based on PhotonLib metrics
                                        Matrix<N3, N3> visionCovariance = calculatePhotonCovariance(estimatedPose);
                                        // Convert 3x3 covariance matrix to diagonal vector for addVisionMeasurement
                                        var stdDevs = edu.wpi.first.math.VecBuilder.fill(
                                            Math.sqrt(visionCovariance.get(0, 0)),
                                            Math.sqrt(visionCovariance.get(1, 1)),
                                            Math.sqrt(visionCovariance.get(2, 2))
                                        );
                                        poseEstimator.addVisionMeasurement(
                                            estimatedPose.estimatedPose.toPose2d(),
                                            estimatedPose.timestampSeconds,
                                            stdDevs
                                        );
                                        validVisionUpdates++;
                                    } catch (Exception e) {
                                        performanceMonitor.recordError("vision", "Failed to add vision measurement: " + e.getMessage());
                                    }
                                } else {
                                    performanceMonitor.recordWarning("vision", "Invalid vision pose from camera " + cameraId);
                                }
                            }
                        }
                    } catch (Exception e) {
                        cameraFailures++;
                        performanceMonitor.recordError("vision", "Camera " + cameraId + " processing failed: " + e.getMessage());
                    }
                }

                // Update performance metrics
                performanceMonitor.recordMetric("vision_updates_per_cycle", validVisionUpdates);
                performanceMonitor.recordMetric("camera_failure_rate",
                    cameras.size() > 0 ? (double) cameraFailures / cameras.size() : 0.0);

                // Check if gyroscope can be recovered
                checkGyroscopeRecovery(timestamp);

            } catch (Exception e) {
                performanceMonitor.recordError("localization", "Critical error in pose update: " + e.getMessage());
                e.printStackTrace();
            }
        } catch (Exception e) {
            System.err.println("Fatal error in updatePose: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private boolean isValidModulePosition(SwerveModulePosition position) {
        // Check for reasonable distance values
        double distance = position.distanceMeters;
        if (Double.isNaN(distance) || Double.isInfinite(distance)) {
            return false;
        }

        // Check for extreme values that might indicate encoder issues
        if (Math.abs(distance) > 10000.0) { // 10km is definitely wrong
            return false;
        }

        // Check angle validity
        double angleRad = position.angle.getRadians();
        if (Double.isNaN(angleRad) || Double.isInfinite(angleRad)) {
            return false;
        }

        return true;
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

        // Check for reasonable field bounds (with safety margin)
        double x = translation.getX();
        double y = translation.getY();

        // FRC field bounds with margin
        if (x < -2.0 || x > 18.0 || y < -2.0 || y > 10.0) {
            return false;
        }

        return true;
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }


    private void initializeCameras() {
        // Initialize cameras and pose estimators based on field configuration
        Set<Integer> activeCameraIds = fieldConfig.getActiveCameraIds();

        for (Integer cameraId : activeCameraIds) {
            // Create camera instance (name should match PhotonVision camera name)
            String cameraName = "Camera_" + cameraId;
            PhotonCamera camera = new PhotonCamera(cameraName);
            cameras.put(cameraId, camera);

            // Get camera pose relative to robot
            Transform3d robotToCamera = fieldConfig.getCameraPose(cameraId);
            if (robotToCamera != null) {
                // Create PhotonPoseEstimator for this camera
                PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                    fieldConfig.getAprilTagFieldLayout(),
                    getOptimalPoseStrategy(activeCameraIds.size()),
                    robotToCamera
                );
                estimator.setReferencePose(new Pose2d());

                // Configure estimator for multi-tag pose estimation
                estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                poseEstimators.put(cameraId, estimator);
            }
        }
    }

    private PoseStrategy getOptimalPoseStrategy(int cameraCount) {
        // Choose strategy based on camera configuration
        if (cameraCount >= 3) {
            return PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        } else if (cameraCount == 2) {
            return PoseStrategy.MULTI_TAG_PNP_ON_RIO;
        } else {
            return PoseStrategy.LOWEST_AMBIGUITY;
        }
    }

    private boolean isPhotonPoseValid(EstimatedRobotPose estimatedPose, Pose2d odometryPose) {
        if (estimatedPose == null) return false;

        Pose2d visionPose2d = estimatedPose.estimatedPose.toPose2d();

        // Check for reasonable pose change
        double distance = visionPose2d.getTranslation().getDistance(odometryPose.getTranslation());
        double angleDiff = Math.abs(visionPose2d.getRotation().minus(odometryPose.getRotation()).getRadians());

        // Evaluate ambiguity and target count
        int targetCount = estimatedPose.targetsUsed.size();
        double avgAmbiguity = estimatedPose.targetsUsed.stream()
            .mapToDouble(target -> target.getPoseAmbiguity())
            .average()
            .orElse(1.0);

        // More lenient validation for multi-target solutions
        double maxDistance = targetCount >= 3 ? 2.0 : (targetCount >= 2 ? 1.5 : 1.0);
        double maxAngleDiff = targetCount >= 3 ? Math.PI/2 : (targetCount >= 2 ? Math.PI/3 : Math.PI/4);
        double maxAmbiguity = targetCount >= 3 ? 0.3 : (targetCount >= 2 ? 0.2 : 0.15);

        return distance < maxDistance &&
               angleDiff < maxAngleDiff &&
               avgAmbiguity < maxAmbiguity;
    }

    private Matrix<N3, N3> calculatePhotonCovariance(EstimatedRobotPose estimatedPose) {
        int targetCount = estimatedPose.targetsUsed.size();

        // Calculate average ambiguity
        double avgAmbiguity = estimatedPose.targetsUsed.stream()
            .mapToDouble(target -> target.getPoseAmbiguity())
            .average()
            .orElse(0.5);

        // Calculate average distance to targets
        double avgDistance = estimatedPose.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(3.0);

        // Base uncertainty that scales with target count and quality
        double baseStdDev = 0.05; // 5cm base uncertainty

        // Scale based on number of targets (more targets = better)
        double targetFactor = Math.max(0.5, 3.0 / targetCount);

        // Scale based on ambiguity (lower ambiguity = better)
        double ambiguityFactor = Math.max(1.0, avgAmbiguity * 5.0);

        // Scale based on distance (closer targets = better)
        double distanceFactor = Math.max(1.0, avgDistance / 3.0);

        double xyStdDev = baseStdDev * targetFactor * ambiguityFactor * distanceFactor;
        double rotStdDev = Math.toRadians(2.0) * targetFactor * ambiguityFactor;

        Matrix<N3, N3> covariance = new Matrix<>(N3.instance, N3.instance);
        covariance.set(0, 0, xyStdDev * xyStdDev);
        covariance.set(1, 1, xyStdDev * xyStdDev);
        covariance.set(2, 2, rotStdDev * rotStdDev);

        return covariance;
    }

    private Rotation2d calculateRotationFromModules(SwerveModulePosition[] modulePositions) {
        if (poseHistory.history.isEmpty()) {
            return new Rotation2d(); // First update, no previous data
        }

        // Get previous pose for delta calculation
        Pose2d previousPose = poseEstimator.getEstimatedPosition();

        // Use kinematics to calculate twist (change in pose) from module deltas
        SwerveModulePosition[] previousPositions = getPreviousModulePositions();
        if (previousPositions == null) {
            storePreviousModulePositions(modulePositions);
            return previousPose.getRotation();
        }

        // Calculate module position deltas
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            if (modulePositions[i] != null && previousPositions[i] != null) {
                double deltaDistance = modulePositions[i].distanceMeters - previousPositions[i].distanceMeters;
                moduleDeltas[i] = new SwerveModulePosition(deltaDistance, modulePositions[i].angle);
            } else {
                moduleDeltas[i] = new SwerveModulePosition(0.0, new Rotation2d());
            }
        }

        // Convert module deltas to chassis twist using kinematics
        var kinematics = getKinematics();
        var twist = kinematics.toTwist2d(moduleDeltas);

        // Apply twist to get new rotation
        Rotation2d newRotation = previousPose.getRotation().plus(new Rotation2d(twist.dtheta));

        // Store current positions for next iteration
        storePreviousModulePositions(modulePositions);

        return newRotation;
    }

    private SwerveModulePosition[] previousModulePositions = null;

    private SwerveModulePosition[] getPreviousModulePositions() {
        return previousModulePositions;
    }

    private void storePreviousModulePositions(SwerveModulePosition[] positions) {
        previousModulePositions = new SwerveModulePosition[positions.length];
        for (int i = 0; i < positions.length; i++) {
            if (positions[i] != null) {
                previousModulePositions[i] = new SwerveModulePosition(
                    positions[i].distanceMeters,
                    positions[i].angle
                );
            }
        }
    }

    private SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Get current rotation using best available source (gyroscope preferred)
     */
    private Rotation2d getCurrentRotation(SwerveModulePosition[] modulePositions, double timestamp) {
        if (useGyroscope && gyroscope.isHealthy()) {
            try (var timing = performanceMonitor.startTiming("gyroscope_reading")) {
                Rotation2d gyroRotation = gyroscope.getRotation();

                // Validate gyroscope reading
                if (isValidGyroscopeReading(gyroRotation, timestamp)) {
                    lastGyroReading = gyroRotation;
                    performanceMonitor.recordMetric("gyroscope_health", 1.0);
                    return gyroRotation;
                } else {
                    performanceMonitor.recordWarning("gyroscope", "Invalid gyroscope reading, falling back to odometry");
                    recordGyroscopeFailure(timestamp);
                }
            } catch (Exception e) {
                performanceMonitor.recordError("gyroscope", "Gyroscope exception: " + e.getMessage());
                recordGyroscopeFailure(timestamp);
            }
        }

        // Fall back to odometry-based rotation calculation
        performanceMonitor.recordMetric("gyroscope_health", 0.0);
        return calculateRotationFromModules(modulePositions);
    }

    /**
     * Validate gyroscope reading for reasonableness
     */
    private boolean isValidGyroscopeReading(Rotation2d gyroRotation, double timestamp) {
        if (gyroRotation == null) {
            return false;
        }

        double angle = gyroRotation.getRadians();

        // Check for NaN or infinite values
        if (!Double.isFinite(angle)) {
            return false;
        }

        // Check for excessive rate of change (more than 2π rad/sec is suspicious for most robots)
        if (!lastGyroReading.equals(new Rotation2d())) {
            double angleDiff = Math.abs(gyroRotation.minus(lastGyroReading).getRadians());
            double maxReasonableChange = MAX_ANGULAR_VELOCITY * 0.02; // 20ms period assumption

            if (angleDiff > maxReasonableChange) {
                return false;
            }
        }

        return true;
    }

    /**
     * Record gyroscope failure and potentially disable it temporarily
     */
    private void recordGyroscopeFailure(double timestamp) {
        gyroFailureTime = timestamp;

        // If gyroscope has been failing for too long, disable it temporarily
        if (timestamp - gyroFailureTime > 5.0) { // 5 seconds of failures
            useGyroscope = false;
            performanceMonitor.recordError("gyroscope", "Gyroscope disabled due to persistent failures");
            System.err.println("MultiCameraLocalization: Gyroscope disabled due to persistent failures");
        }
    }

    /**
     * Attempt to re-enable gyroscope if it becomes healthy again
     */
    private void checkGyroscopeRecovery(double timestamp) {
        if (!useGyroscope && gyroscope.isHealthy()) {
            // Try to re-enable gyroscope after 10 seconds of being disabled
            if (timestamp - gyroFailureTime > 10.0) {
                useGyroscope = true;
                performanceMonitor.recordMetric("gyroscope_recovery", 1.0);
                System.out.println("MultiCameraLocalization: Gyroscope re-enabled");
            }
        }
    }



    public PoseHistory getPoseHistory() {
        return poseHistory;
    }

    public int getVisibleAirTagCount() {
        return observationFilter.getActiveAirTagCount();
    }

    public int getActiveCameraCount() {
        return observationFilter.getActiveCameraCount();
    }

    public PerformanceMonitor getPerformanceMonitor() {
        return performanceMonitor;
    }

    public GyroscopeManager getGyroscope() {
        return gyroscope;
    }

    public boolean isUsingGyroscope() {
        return useGyroscope && gyroscope.isHealthy();
    }

    public Rotation2d getGyroscopeRotation() {
        return gyroscope.getRotation();
    }

    public double getAngularVelocity() {
        return gyroscope.getAngularVelocity();
    }

    /**
     * Reset gyroscope to current heading (useful for field-centric driving)
     */
    public void resetGyroscope() {
        gyroscope.reset();
        System.out.println("MultiCameraLocalization: Gyroscope reset");
    }

    /**
     * Manually enable/disable gyroscope usage
     */
    public void setUseGyroscope(boolean use) {
        this.useGyroscope = use && gyroscope.isHealthy();
        System.out.printf("MultiCameraLocalization: Gyroscope usage set to %s%n", this.useGyroscope);
    }

    public void updateDashboard() {
        // Update basic pose information
        Pose2d currentPose = getCurrentPose();
        SmartDashboard.putNumber("Robot_X", currentPose.getX());
        SmartDashboard.putNumber("Robot_Y", currentPose.getY());
        SmartDashboard.putNumber("Robot_Angle", currentPose.getRotation().getDegrees());

        // Update vision status
        SmartDashboard.putNumber("Active_Cameras", cameras.size());
        SmartDashboard.putNumber("Active_AirTags", getVisibleAirTagCount());
        SmartDashboard.putBoolean("Vision_Healthy", cameras.size() > 0 && getVisibleAirTagCount() > 0);

        // Update gyroscope status
        SmartDashboard.putBoolean("Gyro_Enabled", useGyroscope);
        SmartDashboard.putBoolean("Gyro_Available", gyroscope.isHealthy());
        SmartDashboard.putString("Rotation_Source", useGyroscope && gyroscope.isHealthy() ? "Gyroscope" : "Odometry");
        gyroscope.updateDashboard();

        // Update performance monitoring
        performanceMonitor.updateDashboard();

        // Show performance warnings
        List<String> warnings = performanceMonitor.getPerformanceWarnings();
        if (!warnings.isEmpty()) {
            SmartDashboard.putString("Performance_Warnings", String.join("; ", warnings));
        } else {
            SmartDashboard.putString("Performance_Warnings", "None");
        }

        // Update system health status
        String healthStatus = getSystemHealthStatus();
        SmartDashboard.putString("System_Health", healthStatus);
    }

    private String getSystemHealthStatus() {
        List<String> issues = new ArrayList<>();

        // Check camera health
        if (cameras.isEmpty()) {
            issues.add("No cameras");
        } else {
            int workingCameras = 0;
            for (Map.Entry<Integer, PhotonCamera> entry : cameras.entrySet()) {
                try {
                    PhotonPipelineResult result = entry.getValue().getAllUnreadResults().stream().findFirst().orElse(null);
                    if (result != null) {
                        workingCameras++;
                    }
                } catch (Exception e) {
                    // Camera not responding
                }
            }
            if (workingCameras == 0) {
                issues.add("All cameras offline");
            } else if (workingCameras < cameras.size()) {
                issues.add(String.format("%d/%d cameras offline", cameras.size() - workingCameras, cameras.size()));
            }
        }

        // Check gyroscope health
        if (!gyroscope.isHealthy() && useGyroscope) {
            issues.add("Gyroscope unhealthy");
        } else if (!useGyroscope) {
            issues.add("Gyroscope disabled");
        }

        // Check localization health
        if (getVisibleAirTagCount() == 0) {
            issues.add("No AirTags visible");
        }

        // Check performance health
        if (performanceMonitor.hasPerformanceIssues()) {
            issues.add("Performance issues");
        }

        // Check pose validity
        if (!isValidPose(getCurrentPose())) {
            issues.add("Invalid pose");
        }

        if (issues.isEmpty()) {
            return "HEALTHY";
        } else if (issues.size() <= 2) {
            return "WARNING: " + String.join(", ", issues);
        } else {
            return "CRITICAL: Multiple issues";
        }
    }

    // Configuration now handled by FieldConfiguration class
    // Supports both hardcoded positions and dashboard calibration

    public static class AirTagObservation {
        public final int id;
        public final Translation3d position3D;
        public final int cameraId;
        public final double confidence;
        public final double timestamp;

        public AirTagObservation(int id, Translation3d position3D, int cameraId, double confidence) {
            this.id = id;
            this.position3D = position3D;
            this.cameraId = cameraId;
            this.confidence = confidence;
            this.timestamp = Timer.getFPGATimestamp();
        }

        public AirTagObservation(int id, Translation3d position3D, int cameraId, double confidence, double timestamp) {
            this.id = id;
            this.position3D = position3D;
            this.cameraId = cameraId;
            this.confidence = confidence;
            this.timestamp = timestamp;
        }
    }


    private static class PoseHistory {
        private final List<TimestampedPose> history;
        private final int maxSize;

        public PoseHistory(int maxSize) {
            this.maxSize = maxSize;
            this.history = new ArrayList<>();
        }

        public void addPose(Pose2d pose, double timestamp) {
            history.add(new TimestampedPose(pose, timestamp));
            if (history.size() > maxSize) {
                history.remove(0);
            }
        }

        public Pose2d getPoseAtTime(double timestamp) {
            if (history.isEmpty()) return null;

            // Find closest pose in time
            TimestampedPose closest = history.get(0);
            double minDiff = Math.abs(timestamp - closest.timestamp);

            for (TimestampedPose tp : history) {
                double diff = Math.abs(timestamp - tp.timestamp);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest = tp;
                }
            }

            return closest.pose;
        }

        public boolean hasRecentMovement(double timeWindow, double minDistance) {
            if (history.size() < 2) return false;

            double currentTime = Timer.getFPGATimestamp();
            List<TimestampedPose> recentPoses = history.stream()
                .filter(p -> currentTime - p.timestamp < timeWindow)
                .collect(Collectors.toList());

            if (recentPoses.size() < 2) return false;

            Pose2d first = recentPoses.get(0).pose;
            Pose2d last = recentPoses.get(recentPoses.size() - 1).pose;

            return first.getTranslation().getDistance(last.getTranslation()) > minDistance;
        }

        private static class TimestampedPose {
            public final Pose2d pose;
            public final double timestamp;

            public TimestampedPose(Pose2d pose, double timestamp) {
                this.pose = pose;
                this.timestamp = timestamp;
            }
        }
    }

    private static class ObservationFilter {
        private final Map<Integer, List<AirTagObservation>> recentObservations;
        private static final double OBSERVATION_TIMEOUT = 0.5; // seconds
        private static final double MAX_DETECTION_DISTANCE = 8.0; // meters

        public ObservationFilter() {
            this.recentObservations = new HashMap<>();
        }

        public List<AirTagObservation> filterObservations(List<AirTagObservation> observations,
                                                         Pose2d currentPose, double timestamp) {
            List<AirTagObservation> filtered = new ArrayList<>();

            for (AirTagObservation obs : observations) {
                if (isObservationValid(obs, currentPose, timestamp)) {
                    filtered.add(obs);
                    addToHistory(obs);
                }
            }

            cleanOldObservations(timestamp);
            return filtered;
        }

        private boolean isObservationValid(AirTagObservation obs, Pose2d currentPose, double timestamp) {
            // Distance check
            if (obs.position3D.getNorm() > MAX_DETECTION_DISTANCE) {
                return false;
            }

            // Confidence check
            if (obs.confidence < 0.3) {
                return false;
            }

            // Temporal consistency check
            List<AirTagObservation> recent = recentObservations.get(obs.id);
            if (recent != null && !recent.isEmpty()) {
                AirTagObservation lastObs = recent.get(recent.size() - 1);
                double timeDiff = timestamp - lastObs.timestamp;
                double positionDiff = obs.position3D.getDistance(lastObs.position3D);

                // Check for physically impossible movement
                double maxMovement = MAX_VELOCITY * timeDiff + 0.5; // 0.5m tolerance
                if (positionDiff > maxMovement) {
                    return false;
                }
            }

            return true;
        }

        private void addToHistory(AirTagObservation obs) {
            recentObservations.computeIfAbsent(obs.id, k -> new ArrayList<>()).add(obs);
        }

        private void cleanOldObservations(double currentTime) {
            for (List<AirTagObservation> observations : recentObservations.values()) {
                observations.removeIf(obs -> currentTime - obs.timestamp > OBSERVATION_TIMEOUT);
            }
        }

        public int getActiveAirTagCount() {
            return (int) recentObservations.values().stream()
                .mapToLong(List::size)
                .sum();
        }

        public int getActiveCameraCount() {
            return (int) recentObservations.values().stream()
                .flatMap(List::stream)
                .mapToInt(obs -> obs.cameraId)
                .distinct()
                .count();
        }
    }
}