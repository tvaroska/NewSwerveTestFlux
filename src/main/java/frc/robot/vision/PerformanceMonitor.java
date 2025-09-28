package frc.robot.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class PerformanceMonitor {

    private final Map<String, TimingData> timingData;
    private final Map<String, Integer> errorCounts;
    private final Map<String, Double> performanceMetrics;

    private static final int TIMING_HISTORY_SIZE = 50;
    private static final double PERFORMANCE_WARNING_THRESHOLD = 0.050; // 50ms
    private static final double PERFORMANCE_CRITICAL_THRESHOLD = 0.100; // 100ms

    public PerformanceMonitor() {
        this.timingData = new ConcurrentHashMap<>();
        this.errorCounts = new ConcurrentHashMap<>();
        this.performanceMetrics = new ConcurrentHashMap<>();

        // Initialize key metrics
        performanceMetrics.put("vision_update_frequency", 0.0);
        performanceMetrics.put("localization_accuracy", 0.0);
        performanceMetrics.put("camera_failure_rate", 0.0);
    }

    public TimingContext startTiming(String operation) {
        return new TimingContext(operation);
    }

    public void recordError(String category, String error) {
        errorCounts.merge(category, 1, Integer::sum);
        System.err.printf("[ERROR] %s: %s (Total: %d)%n",
            category, error, errorCounts.get(category));

        // Update dashboard
        SmartDashboard.putNumber("Errors_" + category, errorCounts.get(category));
    }

    public void recordWarning(String category, String warning) {
        System.out.printf("[WARNING] %s: %s%n", category, warning);
    }

    public void recordMetric(String name, double value) {
        performanceMetrics.put(name, value);
        SmartDashboard.putNumber("Metric_" + name, value);
    }

    public void updateDashboard() {
        // Update timing statistics
        for (Map.Entry<String, TimingData> entry : timingData.entrySet()) {
            String operation = entry.getKey();
            TimingData data = entry.getValue();

            if (!data.timings.isEmpty()) {
                double avgTime = data.getAverageTime();
                double maxTime = data.getMaxTime();
                double frequency = data.getFrequency();

                SmartDashboard.putNumber("Timing_" + operation + "_Avg", avgTime * 1000); // ms
                SmartDashboard.putNumber("Timing_" + operation + "_Max", maxTime * 1000); // ms
                SmartDashboard.putNumber("Timing_" + operation + "_Hz", frequency);

                // Performance warnings
                String status = "OK";
                if (avgTime > PERFORMANCE_CRITICAL_THRESHOLD) {
                    status = "CRITICAL";
                } else if (avgTime > PERFORMANCE_WARNING_THRESHOLD) {
                    status = "WARNING";
                }
                SmartDashboard.putString("Status_" + operation, status);
            }
        }

        // Update error counts
        int totalErrors = errorCounts.values().stream().mapToInt(Integer::intValue).sum();
        SmartDashboard.putNumber("Total_Errors", totalErrors);

        // Update performance metrics
        for (Map.Entry<String, Double> entry : performanceMetrics.entrySet()) {
            SmartDashboard.putNumber("Perf_" + entry.getKey(), entry.getValue());
        }
    }

    public boolean hasPerformanceIssues() {
        for (TimingData data : timingData.values()) {
            if (data.getAverageTime() > PERFORMANCE_CRITICAL_THRESHOLD) {
                return true;
            }
        }
        return false;
    }

    public List<String> getPerformanceWarnings() {
        List<String> warnings = new ArrayList<>();

        for (Map.Entry<String, TimingData> entry : timingData.entrySet()) {
            String operation = entry.getKey();
            TimingData data = entry.getValue();
            double avgTime = data.getAverageTime();

            if (avgTime > PERFORMANCE_CRITICAL_THRESHOLD) {
                warnings.add(String.format("%s running critically slow: %.1fms", operation, avgTime * 1000));
            } else if (avgTime > PERFORMANCE_WARNING_THRESHOLD) {
                warnings.add(String.format("%s running slow: %.1fms", operation, avgTime * 1000));
            }

            if (data.getFrequency() < 5.0 && operation.contains("vision")) {
                warnings.add(String.format("%s frequency too low: %.1fHz", operation, data.getFrequency()));
            }
        }

        return warnings;
    }

    public class TimingContext implements AutoCloseable {
        private final String operation;
        private final double startTime;

        public TimingContext(String operation) {
            this.operation = operation;
            this.startTime = Timer.getFPGATimestamp();
        }

        @Override
        public void close() {
            double endTime = Timer.getFPGATimestamp();
            double duration = endTime - startTime;

            timingData.computeIfAbsent(operation, k -> new TimingData()).addTiming(duration);
        }
    }

    private static class TimingData {
        private final Queue<Double> timings;
        private final Queue<Double> timestamps;

        public TimingData() {
            this.timings = new LinkedList<>();
            this.timestamps = new LinkedList<>();
        }

        public synchronized void addTiming(double duration) {
            double now = Timer.getFPGATimestamp();

            timings.offer(duration);
            timestamps.offer(now);

            // Maintain history size
            while (timings.size() > TIMING_HISTORY_SIZE) {
                timings.poll();
                timestamps.poll();
            }
        }

        public synchronized double getAverageTime() {
            return timings.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        }

        public synchronized double getMaxTime() {
            return timings.stream().mapToDouble(Double::doubleValue).max().orElse(0.0);
        }

        public synchronized double getFrequency() {
            if (timestamps.size() < 2) return 0.0;

            double timeSpan = Timer.getFPGATimestamp() - timestamps.peek();
            return timeSpan > 0 ? (timestamps.size() - 1) / timeSpan : 0.0;
        }
    }
}