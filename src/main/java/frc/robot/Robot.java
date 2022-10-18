// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_stick;

  private int FRONT_LEFT_MOTOR_PORT = 4;
  private int REAR_LEFT_MOTOR_PORT = 1;
  private int FRONT_RIGHT_MOTOR_PORT = 2;
  private int REAR_RIGHT_MOTOR_PORT = 3;

  private CANSparkMax m_frontleftMotor;
  private CANSparkMax m_rearleftMotor;
  private CANSparkMax m_frontrightMotor;
  private CANSparkMax m_rearrightMotor;
  
  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;
  
  @Override
  public void robotInit() {
    m_frontleftMotor = new CANSparkMax(FRONT_LEFT_MOTOR_PORT, MotorType.kBrushed);
    m_rearleftMotor = new CANSparkMax(REAR_LEFT_MOTOR_PORT, MotorType.kBrushed);
    m_frontrightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushed);
    m_rearrightMotor = new CANSparkMax(REAR_RIGHT_MOTOR_PORT, MotorType.kBrushed);

    m_leftMotors = new MotorControllerGroup(m_frontleftMotor, m_rearleftMotor);
    m_rightMotors = new MotorControllerGroup(m_frontrightMotor, m_rearrightMotor);
    
    m_leftMotors.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotors, m_rightMotors);
    m_stick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(-m_stick.getRawAxis(1), m_stick.getRawAxis(4));
  }
}