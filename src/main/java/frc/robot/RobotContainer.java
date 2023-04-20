// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LEDs;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the sche uler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final EndEffector m_endEffector = new EndEffector();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final LEDs m_LEDs = new LEDs();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick m_driverStick = new Joystick(OperatorConstants.JOYSTICK_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Start Logger, and record driverstation control and joystick data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // Configure the trigger bindings
    configureBindings();
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                m_drivetrain.arcadeDrive(
                    m_driverStick.getRawAxis(OperatorConstants.JOYSTICK_Y_AXIS),
                    m_driverStick.getRawAxis(OperatorConstants.JOYSTICK_X_AXIS)),
            m_drivetrain));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Arm Button Mappings
    m_driverController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_arm.setGoal(ArmConstants.ArmPosition.kIndex.value);
                  m_arm.enable();
                },
                m_arm));

    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_arm.setGoal(.12);
                  m_arm.enable();
                },
                m_arm));

    m_driverController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_arm.setGoal(Math.PI / 35);
                  m_arm.enable();
                },
                m_arm));

    m_driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_arm.setGoal(-Math.PI / 2 + Math.PI / 30);
                  m_arm.enable();
                },
                m_arm));

    // End Effector button mappings
    m_driverController
        .leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> m_endEffector.runIntake(Constants.EndEffectorConstants.INTAKE_SPEED),
                () -> m_endEffector.runIntake(0),
                m_endEffector));

    m_driverController
        .rightTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> m_endEffector.runIntake(-Constants.EndEffectorConstants.INTAKE_SPEED),
                () -> m_endEffector.runIntake(0),
                m_endEffector));

    m_driverController
        .rightBumper()
        .onTrue(new RunCommand(() -> m_endEffector.extendGripper(), m_endEffector));

    m_driverController
        .leftBumper()
        .onTrue(new RunCommand(() -> m_endEffector.retractGripper(), m_endEffector));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.scoreHighTier(m_drivetrain, m_endEffector, m_arm);
  }

  public void disabledInit() {
    m_LEDs.setAllCyan();
  }

  public void disabledPeriodic() {}

  public void autonomousInit() {
    m_LEDs.setLEDToAllianceColor();
  }

  public void teleopInit() {
    m_LEDs.setAllGreen();
  }
}
