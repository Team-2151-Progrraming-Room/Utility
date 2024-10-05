// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private PIDController m_pid;
  private double m_P         = 0.00004;
  private double m_I         = 0.00008;
  private double m_D         = 0.0;
  private double m_ff        = 0.310;
  private double m_setpoint  = 2000.0;
  private double m_tolerance = m_setpoint * 0.02;   // percentage of the setpoint

  private CANSparkMax m_sparkMax;
  private RelativeEncoder m_encoder;

  private static final String kP_Gain    = "P Gain";
  private static final String kI_Gain    = "I Gain";
  private static final String kD_Gain    = "D Gain";
  private static final String kFF        = "FF";
  private static final String kSetPoint  = "Setpoint";
  private static final String kTolerance = "Tolerance";
  private static final String kCurrent   = "RPM";
  private static final String kReady     = "Ready";
  private static final String kDiffNum   = "Diff";
  private static final String kDiffPer   = "Diff %";
  private static final String kPidValue  = "PID Value";
  private static final String kVelError  = "Vel Error";
  private static final String kMotSet    = "Motor Setting";



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_sparkMax = new CANSparkMax(21, MotorType.kBrushless);   
    
    m_encoder = m_sparkMax.getEncoder();

    m_pid = new PIDController(m_P, m_I, m_D);

    m_pid.setSetpoint(m_setpoint);
    
    m_pid.setTolerance(m_tolerance);

    SmartDashboard.putNumber(kP_Gain, m_P);
    SmartDashboard.putNumber(kI_Gain, m_I);
    SmartDashboard.putNumber(kD_Gain, m_D);
    SmartDashboard.putNumber(kFF, m_ff);
    SmartDashboard.putNumber(kSetPoint, m_setpoint);
    SmartDashboard.putNumber(kTolerance, m_tolerance);
    SmartDashboard.putNumber(kCurrent, m_encoder.getVelocity());
    SmartDashboard.putBoolean(kReady, false);
    SmartDashboard.putNumber(kDiffNum, 0.0);
    SmartDashboard.putNumber(kDiffPer, 0.0);
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber(kVelError, m_pid.getPositionError());

    double pidValue = m_pid.calculate(m_encoder.getVelocity(), m_setpoint);

    SmartDashboard.putNumber(kPidValue, pidValue);

    m_sparkMax.set(pidValue + m_ff);
  
    if (m_pid.atSetpoint()) {
      SmartDashboard.putBoolean(kReady, true);
    } else {
      SmartDashboard.putBoolean(kReady, false);
    }

    SmartDashboard.putNumber(kCurrent, m_encoder.getVelocity());
    SmartDashboard.putNumber(kDiffNum, m_setpoint - m_encoder.getVelocity());
    SmartDashboard.putNumber(kDiffPer, (m_setpoint - m_encoder.getVelocity()) / m_setpoint * 100);
    SmartDashboard.putNumber(kMotSet, m_sparkMax.get());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
