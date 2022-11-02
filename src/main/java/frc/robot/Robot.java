// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_stick;

  private int FRONT_LEFT_MOTOR_PORT = 3;
  private int REAR_LEFT_MOTOR_PORT = 4;
  private int FRONT_RIGHT_MOTOR_PORT = 1;
  private int REAR_RIGHT_MOTOR_PORT = 2;

  private CANSparkMax m_frontleftMotor;
  private CANSparkMax m_rearleftMotor;
  private CANSparkMax m_frontrightMotor;
  private CANSparkMax m_rearrightMotor;
  
  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  private AHRS m_navx;
  private double currentPosition = 0;
  private double currentAngle = 0;
  
  @Override
  public void robotInit() {
    m_frontleftMotor = new CANSparkMax(FRONT_LEFT_MOTOR_PORT, MotorType.kBrushed);
    m_rearleftMotor = new CANSparkMax(REAR_LEFT_MOTOR_PORT, MotorType.kBrushed);
    m_frontrightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushed);
    m_rearrightMotor = new CANSparkMax(REAR_RIGHT_MOTOR_PORT, MotorType.kBrushed);

    m_leftMotors = new MotorControllerGroup(m_frontleftMotor, m_rearleftMotor);
    m_rightMotors = new MotorControllerGroup(m_frontrightMotor, m_rearrightMotor);
    
    m_leftMotors.setInverted(true);

    m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    m_navx.calibrate();
    while (m_navx.isCalibrating() || !m_navx.isConnected()) {
      Timer.delay(1);
    }

    m_myRobot = new DifferentialDrive(m_leftMotors, m_rightMotors);
    m_stick = new Joystick(0);
  }

  @Override
  public void autonomousInit() {
    m_navx.reset();
    m_navx.resetDisplacement();
    
    // Example Movement
    move(1.5, 0.5);
    turn(-30, 0.35);
    turn(30, 0.35);
    move(-1.5, 0.5);
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Displacement", getDistance());
    SmartDashboard.putNumber("Angle", getHeading());
  }

  @Override
  public void teleopInit() {
    m_navx.reset();
    m_navx.resetDisplacement();
    m_myRobot.arcadeDrive(0, 0);
  }

  @Override 
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(-m_stick.getRawAxis(1), m_stick.getRawAxis(4)*0.6);
    SmartDashboard.putNumber("Displacement", getDistance());
    SmartDashboard.putNumber("Angle", getHeading());
  }

  public double getDistance() {
    return m_navx.getDisplacementX();
  }

  public double getHeading() {
    return m_navx.getAngle();
  }

  public void move(double meters, double power) {
    if (meters > 0) {
      while (getDistance() < meters + currentPosition && isAutonomousEnabled()) {
        m_myRobot.arcadeDrive(power, 0);
      }
    }
    if (meters < 0) {  
      while (getDistance() > meters + currentPosition && isAutonomousEnabled()) {
        m_myRobot.arcadeDrive(-power, 0);
      }
    }
    m_myRobot.arcadeDrive(0, 0);
    currentPosition = getDistance();
  }

  public void turn(double degrees, double power) {
    if (degrees > 0) {
      while (getHeading() < degrees + currentAngle && isAutonomousEnabled()) {
        m_myRobot.arcadeDrive(0, power);
      }
    }
    if (degrees < 0) {   
      while (getHeading() > degrees + currentAngle && isAutonomousEnabled()) {
        m_myRobot.arcadeDrive(0, -power);
      }
    }
    m_myRobot.arcadeDrive(0, 0);
    currentAngle = getHeading();
  }

  public void idle(long ms) {
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() < ms + start) {
      m_myRobot.arcadeDrive(0, 0);
    }
  }
}