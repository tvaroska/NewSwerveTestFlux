package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;

public class FieldCalibrationValidator {

    private final FieldConfiguration fieldConfig;
    private final MultiCameraLocalization localization;
    private final List<CalibrationMeasurement> measurements;
    private final Map<Integer, List<Double>> accuracyHistory;

    private static final double MEASUREMENT_TIMEOUT = 30.0; // seconds
    private static final double ACCURACY_THRESHOLD = 0.15; // 15cm
    private static final int MIN_MEASUREMENTS_PER_TAG = 5;

    public FieldCalibrationValidator(FieldConfiguration fieldConfig, MultiCameraLocalization localization) {
        this.fieldConfig = fieldConfig;
        this.localization = localization;
        this.measurements = new ArrayList<>();
        this.accuracyHistory = new HashMap<>();
    }

    public void startCalibrationValidation() {
        measurements.clear();
        accuracyHistory.clear();
        SmartDashboard.putBoolean("Calibration_Validation_Active", true);
        SmartDashboard.putString("Calibration_Status", "Starting validation...");
        System.out.println("=== Field Calibration Validation Started ===");
    }

    public void recordCalibrationPoint(Pose2d knownRobotPose, String description) {
        Pose2d estimatedPose = localization.getCurrentPose();
        double timestamp = Timer.getFPGATimestamp();

        CalibrationPoint point = new CalibrationPoint(
            knownRobotPose, estimatedPose, timestamp, description
        );

        measurements.add(new CalibrationMeasurement(point));

        double error = estimatedPose.getTranslation().getDistance(knownRobotPose.getTranslation());
        double angleError = Math.abs(estimatedPose.getRotation().minus(knownRobotPose.getRotation()).getRadians());

        System.out.printf("Calibration Point: %s\n", description);
        System.out.printf("  Known Pose: (%.2f, %.2f, %.1f°)\n",
            knownRobotPose.getX(), knownRobotPose.getY(), knownRobotPose.getRotation().getDegrees());
        System.out.printf("  Estimated Pose: (%.2f, %.2f, %.1f°)\n",
            estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getDegrees());
        System.out.printf("  Position Error: %.3fm, Angle Error: %.1f°\n",
            error, Math.toDegrees(angleError));

        updateDashboard();
    }

    public void recordAirTagAccuracy(int airTagId) {
        // Record accuracy for a specific AirTag based on current measurements
        List<CalibrationMeasurement> tagMeasurements = measurements.stream()
            .filter(m -> m.point.description.contains("AirTag " + airTagId))
            .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);

        if (!tagMeasurements.isEmpty()) {
            double avgError = tagMeasurements.stream()
                .mapToDouble(m -> m.point.estimatedPose.getTranslation()
                    .getDistance(m.point.knownPose.getTranslation()))
                .average()
                .orElse(0.0);

            accuracyHistory.computeIfAbsent(airTagId, k -> new ArrayList<>()).add(avgError);
        }
    }

    public CalibrationReport generateReport() {
        if (measurements.size() < MIN_MEASUREMENTS_PER_TAG) {
            return new CalibrationReport(false, "Insufficient measurements for validation");
        }

        CalibrationReport report = new CalibrationReport(true, "Calibration validation complete");

        // Calculate overall accuracy statistics
        List<Double> errors = measurements.stream()
            .mapToDouble(m -> m.point.estimatedPose.getTranslation()
                .getDistance(m.point.knownPose.getTranslation()))
            .boxed()
            .collect(ArrayList::new, ArrayList::add, ArrayList::addAll);

        if (!errors.isEmpty()) {
            report.averageError = errors.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
            report.maxError = errors.stream().mapToDouble(Double::doubleValue).max().orElse(0.0);
            report.minError = errors.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);

            // Calculate standard deviation
            double mean = report.averageError;
            double variance = errors.stream()
                .mapToDouble(e -> Math.pow(e - mean, 2))
                .average()
                .orElse(0.0);
            report.standardDeviation = Math.sqrt(variance);
        }

        // Analyze per-AirTag accuracy
        for (Map.Entry<Integer, List<Double>> entry : accuracyHistory.entrySet()) {
            int airTagId = entry.getKey();
            List<Double> tagErrors = entry.getValue();

            if (!tagErrors.isEmpty()) {
                double avgError = tagErrors.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                report.airTagAccuracy.put(airTagId, avgError);

                if (avgError > ACCURACY_THRESHOLD) {
                    report.issues.add("AirTag " + airTagId + " accuracy poor: " + String.format("%.3fm", avgError));
                }
            }
        }

        // Check for systematic biases
        analyzeSystematicBiases(report);

        // Generate recommendations
        generateRecommendations(report);

        return report;
    }

    private void analyzeSystematicBiases(CalibrationReport report) {
        if (measurements.size() < 3) return;

        // Check for consistent X/Y bias
        double sumXBias = 0, sumYBias = 0;
        int count = 0;

        for (CalibrationMeasurement measurement : measurements) {
            Translation2d known = measurement.point.knownPose.getTranslation();
            Translation2d estimated = measurement.point.estimatedPose.getTranslation();

            sumXBias += estimated.getX() - known.getX();
            sumYBias += estimated.getY() - known.getY();
            count++;
        }

        double avgXBias = sumXBias / count;
        double avgYBias = sumYBias / count;

        if (Math.abs(avgXBias) > 0.05) {
            report.issues.add(String.format("Systematic X bias detected: %.3fm", avgXBias));
        }

        if (Math.abs(avgYBias) > 0.05) {
            report.issues.add(String.format("Systematic Y bias detected: %.3fm", avgYBias));
        }

        report.systematicBias = new Translation2d(avgXBias, avgYBias);
    }

    private void generateRecommendations(CalibrationReport report) {
        if (report.averageError > ACCURACY_THRESHOLD) {
            report.recommendations.add("Overall accuracy poor - check AirTag positions and camera calibration");
        }

        if (report.standardDeviation > 0.1) {
            report.recommendations.add("High variance in measurements - check for environmental factors");
        }

        // Check for camera-specific issues
        Map<Integer, Integer> cameraUsage = new HashMap<>();
        for (CalibrationMeasurement measurement : measurements) {
            // This would require tracking which cameras contributed to each measurement
            // For now, provide general recommendations
        }

        if (report.airTagAccuracy.size() < fieldConfig.getAirTagPositions().size()) {
            report.recommendations.add("Not all AirTags validated - move robot to see all tags");
        }

        if (Math.abs(report.systematicBias.getNorm()) > 0.05) {
            report.recommendations.add("Systematic bias detected - adjust AirTag positions in field configuration");
        }

        if (report.maxError > 0.5) {
            report.recommendations.add("Extreme outlier detected - check for measurement errors");
        }
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Calibration_Measurements", measurements.size());

        if (!measurements.isEmpty()) {
            CalibrationReport report = generateReport();

            SmartDashboard.putNumber("Calibration_Avg_Error", report.averageError * 1000); // mm
            SmartDashboard.putNumber("Calibration_Max_Error", report.maxError * 1000); // mm
            SmartDashboard.putNumber("Calibration_Std_Dev", report.standardDeviation * 1000); // mm

            SmartDashboard.putBoolean("Calibration_Passed",
                report.averageError < ACCURACY_THRESHOLD && report.maxError < 0.3);

            if (!report.issues.isEmpty()) {
                SmartDashboard.putString("Calibration_Issues", String.join("; ", report.issues));
            } else {
                SmartDashboard.putString("Calibration_Issues", "None");
            }

            if (!report.recommendations.isEmpty()) {
                SmartDashboard.putString("Calibration_Recommendations",
                    String.join("; ", report.recommendations));
            }
        }

        // Show validation progress
        long recentMeasurements = measurements.stream()
            .mapToLong(m -> (long) m.point.timestamp)
            .filter(t -> Timer.getFPGATimestamp() - t < MEASUREMENT_TIMEOUT)
            .count();

        SmartDashboard.putNumber("Recent_Measurements", recentMeasurements);
        SmartDashboard.putBoolean("Validation_Complete", measurements.size() >= MIN_MEASUREMENTS_PER_TAG);
    }

    public void printDetailedReport() {
        CalibrationReport report = generateReport();

        System.out.println("\n=== FIELD CALIBRATION VALIDATION REPORT ===");
        System.out.printf("Measurements: %d\n", measurements.size());
        System.out.printf("Average Error: %.3fm (%.1fmm)\n", report.averageError, report.averageError * 1000);
        System.out.printf("Maximum Error: %.3fm (%.1fmm)\n", report.maxError, report.maxError * 1000);
        System.out.printf("Standard Deviation: %.3fm (%.1fmm)\n", report.standardDeviation, report.standardDeviation * 1000);
        System.out.printf("Systematic Bias: (%.3f, %.3f)m\n",
            report.systematicBias.getX(), report.systematicBias.getY());

        System.out.println("\nPer-AirTag Accuracy:");
        for (Map.Entry<Integer, Double> entry : report.airTagAccuracy.entrySet()) {
            System.out.printf("  AirTag %d: %.3fm (%.1fmm)\n",
                entry.getKey(), entry.getValue(), entry.getValue() * 1000);
        }

        if (!report.issues.isEmpty()) {
            System.out.println("\nIssues:");
            for (String issue : report.issues) {
                System.out.println("  • " + issue);
            }
        }

        if (!report.recommendations.isEmpty()) {
            System.out.println("\nRecommendations:");
            for (String recommendation : report.recommendations) {
                System.out.println("  • " + recommendation);
            }
        }

        System.out.printf("\nOverall Assessment: %s\n",
            report.averageError < ACCURACY_THRESHOLD ? "PASS" : "FAIL");
    }

    public static class CalibrationPoint {
        public final Pose2d knownPose;
        public final Pose2d estimatedPose;
        public final double timestamp;
        public final String description;

        public CalibrationPoint(Pose2d knownPose, Pose2d estimatedPose, double timestamp, String description) {
            this.knownPose = knownPose;
            this.estimatedPose = estimatedPose;
            this.timestamp = timestamp;
            this.description = description;
        }
    }

    public static class CalibrationMeasurement {
        public final CalibrationPoint point;

        public CalibrationMeasurement(CalibrationPoint point) {
            this.point = point;
        }
    }

    public static class CalibrationReport {
        public final boolean valid;
        public final String status;
        public double averageError = 0.0;
        public double maxError = 0.0;
        public double minError = 0.0;
        public double standardDeviation = 0.0;
        public Translation2d systematicBias = new Translation2d();
        public final Map<Integer, Double> airTagAccuracy = new HashMap<>();
        public final List<String> issues = new ArrayList<>();
        public final List<String> recommendations = new ArrayList<>();

        public CalibrationReport(boolean valid, String status) {
            this.valid = valid;
            this.status = status;
        }
    }
}