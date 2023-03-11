// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {

  private final CANSparkMax m_intake =
      new CANSparkMax(EndEffectorConstants.INTAKE_ID, MotorType.kBrushless);

  private DoubleSolenoid solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          EndEffectorConstants.SOLENOID_FORWARD_CHANNEL,
          EndEffectorConstants.SOLENOID_REVERSE_CHANNEL);

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
