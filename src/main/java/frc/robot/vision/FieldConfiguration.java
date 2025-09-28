package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.*;

public class FieldConfiguration {

    private static final boolean ENABLE_CALIBRATION = true;

    // Base field positions (can be overridden by calibration)
    private static final Map<Integer, Pose3d> BASE_AIRTAG_POSITIONS = createBaseAirTagPositions();
    private static final Map<Integer, Transform3d> BASE_CAMERA_POSITIONS = createBaseCameraPositions();

    // Runtime positions (after calibration adjustments)
    private final Map<Integer, Pose3d> airTagPositions;
    private final Map<Integer, Transform3d> cameraPositions;
    private final AprilTagFieldLayout aprilTagLayout;

    public FieldConfiguration() {
        this.airTagPositions = new HashMap<>(BASE_AIRTAG_POSITIONS);
        this.cameraPositions = new HashMap<>(BASE_CAMERA_POSITIONS);

        // Try to load official field layout, fall back to custom if needed
        this.aprilTagLayout = loadAprilTagLayout();

        if (ENABLE_CALIBRATION) {
            initializeCalibrationDashboard();
        }
    }

    public Map<Integer, Pose3d> getAirTagPositions() {
        if (ENABLE_CALIBRATION) {
            updateFromCalibration();
        }
        return new HashMap<>(airTagPositions);
    }

    public Map<Integer, Transform3d> getCameraPositions() {
        if (ENABLE_CALIBRATION) {
            updateCameraFromCalibration();
        }
        return new HashMap<>(cameraPositions);
    }

    public Pose3d getAirTagPose(int id) {
        return getAirTagPositions().get(id);
    }

    public Transform3d getCameraPose(int cameraId) {
        return getCameraPositions().get(cameraId);
    }

    private void initializeCalibrationDashboard() {
        // AirTag calibration entries
        for (Map.Entry<Integer, Pose3d> entry : BASE_AIRTAG_POSITIONS.entrySet()) {
            int id = entry.getKey();
            Pose3d pose = entry.getValue();

            SmartDashboard.putNumber("AirTag_" + id + "_X", pose.getX());
            SmartDashboard.putNumber("AirTag_" + id + "_Y", pose.getY());
            SmartDashboard.putNumber("AirTag_" + id + "_Z", pose.getZ());
            SmartDashboard.putNumber("AirTag_" + id + "_Yaw", pose.getRotation().getZ());
        }

        // Camera calibration entries
        for (Map.Entry<Integer, Transform3d> entry : BASE_CAMERA_POSITIONS.entrySet()) {
            int id = entry.getKey();
            Transform3d transform = entry.getValue();

            SmartDashboard.putNumber("Camera_" + id + "_X", transform.getX());
            SmartDashboard.putNumber("Camera_" + id + "_Y", transform.getY());
            SmartDashboard.putNumber("Camera_" + id + "_Z", transform.getZ());
            SmartDashboard.putNumber("Camera_" + id + "_Yaw", transform.getRotation().getZ());
        }

        SmartDashboard.putBoolean("Apply_Calibration", false);
    }

    private void updateFromCalibration() {
        if (!SmartDashboard.getBoolean("Apply_Calibration", false)) {
            return;
        }

        for (Integer id : BASE_AIRTAG_POSITIONS.keySet()) {
            double x = SmartDashboard.getNumber("AirTag_" + id + "_X", 0);
            double y = SmartDashboard.getNumber("AirTag_" + id + "_Y", 0);
            double z = SmartDashboard.getNumber("AirTag_" + id + "_Z", 0);
            double yaw = SmartDashboard.getNumber("AirTag_" + id + "_Yaw", 0);

            airTagPositions.put(id, new Pose3d(
                new Translation3d(x, y, z),
                new Rotation3d(0, 0, yaw)
            ));
        }
    }

    private void updateCameraFromCalibration() {
        if (!SmartDashboard.getBoolean("Apply_Calibration", false)) {
            return;
        }

        for (Integer id : BASE_CAMERA_POSITIONS.keySet()) {
            double x = SmartDashboard.getNumber("Camera_" + id + "_X", 0);
            double y = SmartDashboard.getNumber("Camera_" + id + "_Y", 0);
            double z = SmartDashboard.getNumber("Camera_" + id + "_Z", 0);
            double yaw = SmartDashboard.getNumber("Camera_" + id + "_Yaw", 0);

            cameraPositions.put(id, new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(0, 0, yaw)
            ));
        }
    }

    public int getActiveCameraCount() {
        return getCameraPositions().size();
    }

    public boolean isSingleCamera() {
        return getActiveCameraCount() == 1;
    }

    public boolean isDualCamera() {
        return getActiveCameraCount() == 2;
    }

    public boolean isMultiCamera() {
        return getActiveCameraCount() >= 3;
    }

    public Set<Integer> getActiveCameraIds() {
        return getCameraPositions().keySet();
    }

    public String getCameraConfigurationName() {
        int count = getActiveCameraCount();
        switch (count) {
            case 1: return "Single Camera";
            case 2: return "Dual Camera";
            case 3: return "Triple Camera";
            default: return count + " Camera Setup";
        }
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagLayout;
    }

    private AprilTagFieldLayout loadAprilTagLayout() {
        try {
            // Try to load the official 2024 field layout
            // Adjust this for the current FRC season
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            System.err.println("Failed to load official AprilTag layout, using custom layout: " + e.getMessage());
            return createCustomAprilTagLayout();
        }
    }

    private AprilTagFieldLayout createCustomAprilTagLayout() {
        // Create custom layout from our AirTag positions
        List<AprilTag> tags = new ArrayList<>();

        for (Map.Entry<Integer, Pose3d> entry : BASE_AIRTAG_POSITIONS.entrySet()) {
            tags.add(new AprilTag(entry.getKey(), entry.getValue()));
        }

        try {
            // Field dimensions for 2024 (adjust as needed)
            return new AprilTagFieldLayout(tags, 16.54, 8.21);
        } catch (Exception e) {
            System.err.println("Failed to create custom AprilTag layout: " + e.getMessage());
            return null;
        }
    }

    // Future: Auto-calibration from vision measurements
    public void runAutoCalibration(List<CalibrationMeasurement> measurements) {
        // Placeholder for future auto-calibration algorithm
        // Would analyze multiple pose estimates to refine AirTag positions
    }

    private static Map<Integer, Pose3d> createBaseAirTagPositions() {
        Map<Integer, Pose3d> positions = new HashMap<>();

        // Single AprilTag in the middle of the wall
        // Adjust these coordinates based on your actual setup:
        // - X: distance from robot starting position to wall
        // - Y: position along the wall (0 = center of field width)
        // - Z: height of tag above ground
        positions.put(1, new Pose3d(
            new Translation3d(5.0, 0.0, 1.5),           // 5 meters from origin, center of wall, 1.5m high
            new Rotation3d(0, 0, 0)                      // Facing toward robot starting area
        ));

        return positions;
    }

    private static Map<Integer, Transform3d> createBaseCameraPositions() {
        Map<Integer, Transform3d> positions = new HashMap<>();

        // === DUAL CAMERA SETUP (Front + Rear) ===
        // Adjust these values based on your actual camera mounting positions:

        // Camera 1: Front camera
        positions.put(1, new Transform3d(
            new Translation3d(0.30, 0.0, 0.50),     // 30cm forward, centered, 50cm high
            new Rotation3d(0, 0, 0)                  // Facing forward
        ));

        // Camera 2: Rear camera
        positions.put(2, new Transform3d(
            new Translation3d(-0.30, 0.0, 0.50),    // 30cm backward, centered, 50cm high
            new Rotation3d(0, 0, Math.PI)           // Facing backward (180° rotation)
        ));

        return positions;
    }

    public static class CalibrationMeasurement {
        public final int airTagId;
        public final Pose2d estimatedRobotPose;
        public final double confidence;

        public CalibrationMeasurement(int airTagId, Pose2d estimatedRobotPose, double confidence) {
            this.airTagId = airTagId;
            this.estimatedRobotPose = estimatedRobotPose;
            this.confidence = confidence;
        }
    }
}