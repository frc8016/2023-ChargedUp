// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase highTierRetreat(
      Drivetrain drivetrain, EndEffector endEffector, Arm arm) {
    return Commands.sequence(
        // Raise arm to scoring position
        Commands.runOnce(
            () -> {
              arm.setGoal(ArmConstants.CUBE_LAUNCH_POSE);
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
            arm),
        new DriveToDistance(drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
