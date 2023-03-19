// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends CommandBase {
  boolean isFinished;
  Drivetrain m_drivetrain;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetOdometry();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_drivetrain.getLeftEncoderPosition()) < AutonConstants.TAXI_DISTANCE) {
      m_drivetrain.arcadeDrive(AutonConstants.TAXI_SPEED, 0);
    } else {
      m_drivetrain.arcadeDrive(0, 0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
