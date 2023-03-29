// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Ramsete extends CommandBase {

  private final Trajectory m_trajectory;
  private final Drivetrain m_drivetrain;
  private final Timer m_timer = new Timer();
  private final RamseteController m_ramseteController = new RamseteController();
  /** Creates a new Ramsete. */
  public Ramsete(Trajectory trajectory, Drivetrain drivetrain) {
    m_trajectory = trajectory;
    m_drivetrain = drivetrain;

    // Require Drivetrain subsystem
    addRequirements(drivetrain);

    // This value must be tuned
    m_ramseteController.setTolerance(new Pose2d());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetOdometry();
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sample trajectory for desired state at current timestep
    Trajectory.State state = m_trajectory.sample(m_timer.get());

    // Compute ramsete output as ChassisSpeeds
    ChassisSpeeds speeds =
        m_ramseteController.calculate(m_drivetrain.getEstimatedPosition(), state);

    // Follow desired chassis speeds
    m_drivetrain.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_trajectory.getTotalTimeSeconds();
  }
}
