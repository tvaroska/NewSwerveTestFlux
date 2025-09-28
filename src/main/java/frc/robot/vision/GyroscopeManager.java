package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;

/**
 * Manages gyroscope hardware abstraction and provides unified interface
 * for different gyroscope types commonly used in FRC.
 * Optimized for CTRE Phoenix 6 Pigeon2 integration.
 */
public class GyroscopeManager {

    public enum GyroscopeType {
        PIGEON2_CAN,    // CTRE Pigeon 2 on CAN bus (preferred for CTRE systems)
        NAVX_SPI,       // Kauai Labs NavX on SPI
        NAVX_USB,       // Kauai Labs NavX on USB
        ADXRS450_SPI,   // Analog Devices ADXRS450 on SPI
        SIMULATION      // Simulation mode
    }

    private final GyroscopeType gyroType;
    private final Pigeon2 pigeon2;
    private final AHRS navx;
    private final edu.wpi.first.wpilibj.ADXRS450_Gyro adxrs450;

    // CTRE-specific configuration
    private static final int PIGEON2_CAN_ID = 20; // Standard CTRE swerve Pigeon2 ID

    // Calibration and drift correction
    private double gyroOffset = 0.0;
    private double lastValidAngle = 0.0;
    private double lastValidTime = 0.0;
    private double driftRate = 0.0; // degrees per second
    private boolean isCalibrated = false;

    // Health monitoring
    private int consecutiveErrorCount = 0;
    private double lastSuccessfulReading = 0.0;
    private static final int MAX_CONSECUTIVE_ERRORS = 10;
    private static final double HEALTH_CHECK_TIMEOUT = 1.0; // seconds

    /**
     * Create GyroscopeManager with automatic detection (prefers CTRE Pigeon2)
     */
    public GyroscopeManager() {
        this(detectGyroscopeType());
    }

    /**
     * Create GyroscopeManager with specific gyroscope type
     */
    public GyroscopeManager(GyroscopeType type) {
        this.gyroType = type;

        // Initialize the appropriate gyroscope
        switch (type) {
            case PIGEON2_CAN:
                this.pigeon2 = new Pigeon2(PIGEON2_CAN_ID); // Use standard CTRE swerve ID
                this.navx = null;
                this.adxrs450 = null;
                System.out.printf("GyroscopeManager: Initialized Pigeon2 on CAN ID %d%n", PIGEON2_CAN_ID);
                break;

            case NAVX_SPI:
                this.pigeon2 = null;
                this.navx = new AHRS(SPI.Port.kMXP);
                this.adxrs450 = null;
                System.out.println("GyroscopeManager: Initialized NavX on SPI");
                break;

            case NAVX_USB:
                this.pigeon2 = null;
                this.navx = new AHRS(SerialPort.Port.kUSB);
                this.adxrs450 = null;
                System.out.println("GyroscopeManager: Initialized NavX on USB");
                break;

            case ADXRS450_SPI:
                this.pigeon2 = null;
                this.navx = null;
                this.adxrs450 = new edu.wpi.first.wpilibj.ADXRS450_Gyro();
                System.out.println("GyroscopeManager: Initialized ADXRS450 on SPI");
                break;

            case SIMULATION:
            default:
                this.pigeon2 = null;
                this.navx = null;
                this.adxrs450 = null;
                System.out.println("GyroscopeManager: Running in simulation mode");
                break;
        }

        // Start calibration process
        startCalibration();
    }

    /**
     * Automatically detect available gyroscope type (prioritizes CTRE Pigeon2)
     */
    private static GyroscopeType detectGyroscopeType() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return GyroscopeType.SIMULATION;
        }

        // Try to detect CTRE Pigeon2 first (most common in CTRE swerve systems)
        try {
            Pigeon2 testPigeon = new Pigeon2(PIGEON2_CAN_ID);
            // Test if we can read from the Pigeon2
            testPigeon.getYaw().refresh();
            if (testPigeon.getYaw().getStatus().isOK()) {
                System.out.printf("GyroscopeManager: Detected Pigeon2 on CAN ID %d%n", PIGEON2_CAN_ID);
                return GyroscopeType.PIGEON2_CAN;
            }
        } catch (Exception e) {
            System.out.printf("GyroscopeManager: Pigeon2 on CAN ID %d not found: %s%n", PIGEON2_CAN_ID, e.getMessage());
        }

        // Try NavX on SPI (common fallback)
        try {
            AHRS testNavX = new AHRS(SPI.Port.kMXP);
            // Wait briefly for NavX to initialize
            Thread.sleep(100);
            if (testNavX.isConnected()) {
                System.out.println("GyroscopeManager: Detected NavX on SPI");
                return GyroscopeType.NAVX_SPI;
            }
        } catch (Exception e) {
            System.out.println("GyroscopeManager: NavX on SPI not found: " + e.getMessage());
        }

        // Try NavX on USB
        try {
            AHRS testNavX = new AHRS(SerialPort.Port.kUSB);
            Thread.sleep(100);
            if (testNavX.isConnected()) {
                System.out.println("GyroscopeManager: Detected NavX on USB");
                return GyroscopeType.NAVX_USB;
            }
        } catch (Exception e) {
            System.out.println("GyroscopeManager: NavX on USB not found: " + e.getMessage());
        }

        // Default to simulation if no hardware detected
        System.out.println("GyroscopeManager: No gyroscope hardware detected, using simulation mode");
        return GyroscopeType.SIMULATION;
    }

    /**
     * Get current rotation from gyroscope
     */
    public Rotation2d getRotation() {
        double angle = 0.0;
        double currentTime = Timer.getFPGATimestamp();

        try {
            switch (gyroType) {
                case PIGEON2_CAN:
                    if (pigeon2 != null) {
                        // Use getYaw() for continuous heading
                        angle = pigeon2.getYaw().getValueAsDouble();
                    }
                    break;

                case NAVX_SPI:
                case NAVX_USB:
                    if (navx != null && navx.isConnected()) {
                        // Use getYaw() for NavX as well
                        angle = navx.getYaw();
                    } else {
                        throw new RuntimeException("NavX not connected");
                    }
                    break;

                case ADXRS450_SPI:
                    if (adxrs450 != null) {
                        angle = adxrs450.getAngle();
                    }
                    break;

                case SIMULATION:
                    // In simulation, return a slowly changing angle for testing
                    angle = (Timer.getFPGATimestamp() * 10) % 360; // 10 deg/sec rotation
                    break;
            }

            // Apply calibration offset and drift correction
            angle = applyCalibration(angle, currentTime);

            // Validate the reading
            if (isValidAngleReading(angle, currentTime)) {
                lastValidAngle = angle;
                lastValidTime = currentTime;
                lastSuccessfulReading = currentTime;
                consecutiveErrorCount = 0;
                return Rotation2d.fromDegrees(angle);
            } else {
                throw new RuntimeException("Invalid angle reading: " + angle);
            }

        } catch (Exception e) {
            consecutiveErrorCount++;
            System.err.printf("GyroscopeManager error (count: %d): %s%n", consecutiveErrorCount, e.getMessage());

            // Use extrapolated angle based on last known angle and drift rate
            if (lastValidTime > 0) {
                double timeDelta = currentTime - lastValidTime;
                double extrapolatedAngle = lastValidAngle + (driftRate * timeDelta);
                return Rotation2d.fromDegrees(extrapolatedAngle);
            }

            // Fallback to zero rotation if no previous data
            return new Rotation2d();
        }
    }

    /**
     * Get angular velocity in degrees per second
     */
    public double getAngularVelocity() {
        try {
            switch (gyroType) {
                case PIGEON2_CAN:
                    if (pigeon2 != null) {
                        // Use Z-axis angular velocity (yaw rate)
                        return pigeon2.getAngularVelocityZWorld().getValueAsDouble();
                    }
                    break;

                case NAVX_SPI:
                case NAVX_USB:
                    if (navx != null && navx.isConnected()) {
                        return navx.getRate();
                    }
                    break;

                case ADXRS450_SPI:
                    if (adxrs450 != null) {
                        return adxrs450.getRate();
                    }
                    break;

                case SIMULATION:
                    return 10.0; // Constant 10 deg/sec for simulation
            }
        } catch (Exception e) {
            System.err.println("GyroscopeManager: Error reading angular velocity: " + e.getMessage());
        }

        return 0.0;
    }

    /**
     * Reset gyroscope to zero
     */
    public void reset() {
        try {
            switch (gyroType) {
                case PIGEON2_CAN:
                    if (pigeon2 != null) {
                        pigeon2.reset();
                        System.out.println("GyroscopeManager: Pigeon2 reset");
                    }
                    break;

                case NAVX_SPI:
                case NAVX_USB:
                    if (navx != null) {
                        navx.reset();
                        System.out.println("GyroscopeManager: NavX reset");
                    }
                    break;

                case ADXRS450_SPI:
                    if (adxrs450 != null) {
                        adxrs450.reset();
                        System.out.println("GyroscopeManager: ADXRS450 reset");
                    }
                    break;

                case SIMULATION:
                    // Reset simulation offset
                    gyroOffset = Timer.getFPGATimestamp() * 10;
                    System.out.println("GyroscopeManager: Simulation gyro reset");
                    break;
            }

            // Reset calibration data
            lastValidAngle = 0.0;
            lastValidTime = Timer.getFPGATimestamp();
            consecutiveErrorCount = 0;

        } catch (Exception e) {
            System.err.println("GyroscopeManager: Error resetting gyroscope: " + e.getMessage());
        }
    }

    /**
     * Check if gyroscope is healthy and providing valid data
     */
    public boolean isHealthy() {
        double currentTime = Timer.getFPGATimestamp();

        // Check for too many consecutive errors
        if (consecutiveErrorCount >= MAX_CONSECUTIVE_ERRORS) {
            return false;
        }

        // Check for recent successful reading
        if (currentTime - lastSuccessfulReading > HEALTH_CHECK_TIMEOUT) {
            return false;
        }

        // Hardware-specific health checks
        switch (gyroType) {
            case NAVX_SPI:
            case NAVX_USB:
                return navx != null && navx.isConnected() && !navx.isCalibrating();

            case PIGEON2_CAN:
                if (pigeon2 != null) {
                    // Check if we can successfully read from Pigeon2
                    try {
                        pigeon2.getYaw().refresh();
                        return pigeon2.getYaw().getStatus().isOK() && consecutiveErrorCount == 0;
                    } catch (Exception e) {
                        return false;
                    }
                }
                return false;

            case ADXRS450_SPI:
                // ADXRS450 doesn't have connection status
                return consecutiveErrorCount == 0;

            case SIMULATION:
                return true;
        }

        return false;
    }

    /**
     * Start calibration process
     */
    private void startCalibration() {
        switch (gyroType) {
            case NAVX_SPI:
            case NAVX_USB:
                if (navx != null) {
                    System.out.println("GyroscopeManager: NavX calibrating...");
                    // NavX auto-calibrates on startup
                }
                break;

            case PIGEON2_CAN:
                if (pigeon2 != null) {
                    System.out.println("GyroscopeManager: Pigeon2 ready");
                    isCalibrated = true;
                }
                break;

            case ADXRS450_SPI:
                if (adxrs450 != null) {
                    System.out.println("GyroscopeManager: ADXRS450 calibrating...");
                    adxrs450.calibrate();
                }
                break;

            case SIMULATION:
                isCalibrated = true;
                break;
        }
    }

    /**
     * Apply calibration offset and drift correction
     */
    private double applyCalibration(double rawAngle, double currentTime) {
        double correctedAngle = rawAngle - gyroOffset;

        // Apply drift correction if we have sufficient data
        if (lastValidTime > 0 && currentTime > lastValidTime) {
            double timeDelta = currentTime - lastValidTime;
            correctedAngle -= driftRate * timeDelta;
        }

        return correctedAngle;
    }

    /**
     * Validate if angle reading is reasonable
     */
    private boolean isValidAngleReading(double angle, double currentTime) {
        // Check for NaN or infinite values
        if (!Double.isFinite(angle)) {
            return false;
        }

        // Check for excessive rate of change (more than 720 deg/sec is suspicious)
        if (lastValidTime > 0) {
            double timeDelta = currentTime - lastValidTime;
            if (timeDelta > 0) {
                double rateOfChange = Math.abs(angle - lastValidAngle) / timeDelta;
                if (rateOfChange > 720.0) {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Update dashboard with gyroscope status
     */
    public void updateDashboard() {
        SmartDashboard.putString("Gyro_Type", gyroType.name());
        SmartDashboard.putBoolean("Gyro_Healthy", isHealthy());
        SmartDashboard.putBoolean("Gyro_Calibrated", isCalibrated());
        SmartDashboard.putNumber("Gyro_Angle", getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro_Rate", getAngularVelocity());
        SmartDashboard.putNumber("Gyro_Errors", consecutiveErrorCount);

        // Hardware-specific status
        switch (gyroType) {
            case NAVX_SPI:
            case NAVX_USB:
                if (navx != null) {
                    SmartDashboard.putBoolean("NavX_Connected", navx.isConnected());
                    SmartDashboard.putBoolean("NavX_Calibrating", navx.isCalibrating());
                }
                break;

            case PIGEON2_CAN:
                if (pigeon2 != null) {
                    try {
                        SmartDashboard.putNumber("Pigeon2_Temp", pigeon2.getTemperature().getValueAsDouble());
                        SmartDashboard.putString("Pigeon2_Status", pigeon2.getYaw().getStatus().getName());
                    } catch (Exception e) {
                        SmartDashboard.putString("Pigeon2_Status", "Error: " + e.getMessage());
                    }
                }
                break;
        }
    }

    /**
     * Get gyroscope type
     */
    public GyroscopeType getGyroscopeType() {
        return gyroType;
    }

    /**
     * Check if gyroscope is calibrated
     */
    public boolean isCalibrated() {
        switch (gyroType) {
            case NAVX_SPI:
            case NAVX_USB:
                return navx != null && !navx.isCalibrating();
            case PIGEON2_CAN:
            case ADXRS450_SPI:
            case SIMULATION:
                return isCalibrated;
        }
        return false;
    }

    /**
     * Get the CAN ID used for Pigeon2 (useful for configuration)
     */
    public static int getPigeon2CanId() {
        return PIGEON2_CAN_ID;
    }
}