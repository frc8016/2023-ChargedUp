// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  // Score cube on high node
  public static CommandBase scoreHighTier(EndEffector endEffector, Arm arm) {
    return Commands.sequence(
        // Raise arm to scoring position
        Commands.runOnce(
            () -> {
              arm.setGoal(Math.PI / 20);
              arm.enable();
            },
            endEffector),
        // Delay for arm to move to position
        new WaitCommand(AutonConstants.RAISE_DELAY),
        // Eject game piece
        Commands.runOnce(
            () -> endEffector.runIntake(AutonConstants.CUBE_HIGH_TIER_SPEED), endEffector),
        // Wait for game piece to eject
        new WaitCommand(AutonConstants.SCORE_DELAY),
        // Lower arm
        Commands.runOnce(() -> endEffector.runIntake(0), endEffector),
        Commands.runOnce(
            () -> {
              arm.setGoal(-Math.PI / 2 + Math.PI / 30);
              arm.enable();
            },
            arm));
  }

  public static CommandBase arbitraryTrajectory(Drivetrain drivetrain) {
    TrajectoryConfig config =
        new TrajectoryConfig(6, 1)
            .setReversed(true)
            .setKinematics(drivetrain.driveKinematics)
            .addConstraint(new CentripetalAccelerationConstraint(.5))
            .addConstraint(drivetrain.constraint);
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(new Translation2d(), new Rotation2d(Math.PI)),
            List.of(new Translation2d(1, 0)),
            //       new Pose2d(new Translation2d(5, 0), new Rotation2d()),
            new Pose2d(new Translation2d(2, 0), new Rotation2d(Math.PI)),
            config);

    return new Ramsete(trajectory, drivetrain);
  }

  public static CommandBase taxiNoCable(Drivetrain drivetrain, String alliance) {
    Trajectory trajectory = null;

    try {
      Path trajectoryPath =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("pathplanner/generatedJSON/" + alliance + ".wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());
    }
    return new Ramsete(trajectory, drivetrain);
  }

  public static CommandBase taxiFarSide(Drivetrain drivetrain, String alliance) {
    Trajectory trajectory = null;

    try {
      Path trajectoryPath =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("pathplanner/generatedJSON/" + alliance + ".wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());
    }
    return new Ramsete(trajectory, drivetrain);
  }

  public static CommandBase overChargeStation(Drivetrain drivetrain) {
    Trajectory trajectory = null;

    try {
      Path trajectoryPath =
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("pathplanner/generatedJSON/OverChargeStation.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());
    }
    return new Ramsete(trajectory, drivetrain);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
