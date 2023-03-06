// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private final PWMSparkMax m_intake =
      new PWMSparkMax(Constants.EndEffectorConstants.INTAKE_ANALOG_PORT);

  private DoubleSolenoid solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.EndEffectorConstants.SOLENOID_FORWARD_CHANNEL,
          Constants.EndEffectorConstants.SOLENOID_REVERSE_CHANNEL);

  /** Creates a new EndEffector. */
  public EndEffector() {}

  public void runIntake(double speed) {
    m_intake.set(speed);
  }

  public void extendGripper() {
    solenoid.set(Value.kForward);
  }

  public void retractGripper() {
    solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
