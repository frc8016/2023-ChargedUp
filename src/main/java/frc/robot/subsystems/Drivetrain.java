// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax m_frontLeftMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor =
      new CANSparkMax(DrivetrainConstants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor =
      new CANSparkMax(DrivetrainConstants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);

  private final MotorControllerGroup m_leftControllerGroup =
      new MotorControllerGroup(m_frontLeftMotor, m_backLeftMotor);
  private final MotorControllerGroup m_rightControlerGroup =
      new MotorControllerGroup(m_frontRightMotor, m_backRightMotor);

  private final DifferentialDrive m_differentialDrive =
      new DifferentialDrive(m_leftControllerGroup, m_rightControlerGroup);
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  public void arcadeDrive(double speed, double rotation) {
    m_differentialDrive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
