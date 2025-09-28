// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  // // Elevator
  // private SparkMax motor;
  // private SparkMaxConfig motorConfig;
  // private SparkLimitSwitch forwardLimitSwitch;
  // private SparkLimitSwitch reverseLimitSwitch;
  // private RelativeEncoder encoder;
  // private SparkClosedLoopController closedLoop;

  // private Joystick joystick;
  // private double targetPosition = 0.0;

  // Store PID values here
  private double kP = 0.05;
  private double kI = 0.0;
  private double kD = 0.0;

  private ClosedLoopConfig pidConfig;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // motor = new SparkMax(1, MotorType.kBrushless);
    // forwardLimitSwitch = motor.getForwardLimitSwitch();
    // reverseLimitSwitch = motor.getReverseLimitSwitch();
    // encoder = motor.getEncoder();
    // closedLoop = motor.getClosedLoopController();

    // // Motor configuration
    // motorConfig = new SparkMaxConfig();
    // motorConfig.idleMode(IdleMode.kBrake);

    // // Limit switches
    // motorConfig.limitSwitch
    //     .forwardLimitSwitchType(Type.kNormallyOpen)
    //     .forwardLimitSwitchEnabled(true)
    //     .reverseLimitSwitchType(Type.kNormallyOpen)
    //     .reverseLimitSwitchEnabled(true);

    // // Soft limits
    // motorConfig.softLimit
    //     .forwardSoftLimit(500)
    //     .forwardSoftLimitEnabled(true)
    //     .reverseSoftLimit(-500)
    //     .reverseSoftLimitEnabled(true);

    // // PID configuration
    // pidConfig = motorConfig.closedLoop;
    // pidConfig.p(kP);
    // pidConfig.i(kI);
    // pidConfig.d(kD);
    // pidConfig.outputRange(-0.3, 0.3);

    // motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // encoder.setPosition(0);

    // // Put initial PID values on SmartDashboard
    // SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kI", kI);
    // SmartDashboard.putNumber("kD", kD);
  }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();

      // Update navigation and vision dashboard
      m_robotContainer.getNavigation().updateDashboard();

      // Update vision system status
      SmartDashboard.putString("Vision_Status", m_robotContainer.getVision().getDetailedStatus());
      SmartDashboard.putString("System_Health", m_robotContainer.getVision().getSystemHealthStatus());

      // Live PID tuning (commented out elevator code)
      double newP = SmartDashboard.getNumber("kP", kP);
      double newI = SmartDashboard.getNumber("kI", kI);
      double newD = SmartDashboard.getNumber("kD", kD);

      // Original elevator code commented out...
      // if (newP != kP || newI != kI || newD != kD) {
      //     kP = newP;
      //     kI = newI;
      //     kD = newD;
      //     pidConfig.p(kP);
      //     pidConfig.i(kI);
      //     pidConfig.d(kD);
      //     motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      // }
  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Check vision system health and choose appropriate autonomous
    if (m_robotContainer.getVision().isVisionHealthy()) {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      System.out.println("Robot: Starting vision-based autonomous");
    } else {
      m_autonomousCommand = m_robotContainer.getEmergencyAutonomous();
      System.out.println("Robot: Vision unhealthy, starting emergency autonomous");
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // int pov = joystick.getPOV(); // returns -1 if no D-Pad pressed

    // // Manual jogging with D-pad
    // if (pov == 0) { // UP
    //   motor.set(0.2);
    // } else if (pov == 180) { // DOWN
    //   motor.set(-0.2);
    // } else {
    //   motor.set(0.0);
    // }

    // // Button B (2) → Move up to 200 rotations
    // if (joystick.getRawButton(2)) {
    //   targetPosition = 200.0;
    //   closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
    // }

    // // Button A (1) → Move down to 0 rotations
    // if (joystick.getRawButton(1)) {
    //   targetPosition = 0.0;
    //   closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
