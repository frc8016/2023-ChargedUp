// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Ramsete extends CommandBase {

  private final Trajectory m_trajectory;
  private final Drivetrain m_drivetrain;
  private final Timer m_timer = new Timer();
  private final RamseteController m_ramseteController = new RamseteController();
  private final Field2d m_fieldSim = new Field2d();

  /** Creates a new Ramsete. */
  public Ramsete(Trajectory trajectory, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_trajectory = trajectory;

    // Require Drivetrain subsystem
    addRequirements(drivetrain);

    // This value must be tuned

    SmartDashboard.putData("Ground pose", m_fieldSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetOdometry(m_trajectory.sample(0.0).poseMeters);
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
    m_fieldSim.setRobotPose(state.poseMeters);
    // Follow desired chassis speeds
    m_drivetrain.setChassisSpeeds(speeds, state, m_trajectory.sample(m_timer.get() - .02));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setChassisSpeeds(
        new ChassisSpeeds(), new Trajectory.State(), new Trajectory.State());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_trajectory.getTotalTimeSeconds();
  }
}
