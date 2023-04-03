// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {

  private final CANSparkMax m_intake =
      new CANSparkMax(EndEffectorConstants.INTAKE_ID, MotorType.kBrushless);

  private DoubleSolenoid m_solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          EndEffectorConstants.SOLENOID_FORWARD_CHANNEL,
          EndEffectorConstants.SOLENOID_REVERSE_CHANNEL);

  /** Creates a new EndEffector. */
  public EndEffector() {
    configureMotors();
    m_solenoid.set(Value.kForward);
  }

  private void configureMotors() {
    // Reset all the motors to factory defaults (except CAN ID)
    m_intake.restoreFactoryDefaults();

    // Set all the motor's idle modes to Brake
    if (m_intake.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {
      System.out.println("ERROR while setting Intake Motor to Brake Mode");
    }

    // Set current limits for all motors
    if (m_intake.setSmartCurrentLimit(EndEffectorConstants.MOTOR_CURRENT_LIMIT)
        != REVLibError.kOk) {
      System.out.println("ERROR while setting Intake Motor smart current limit");
    }

    // Burn all changed settings to flash
    m_intake.burnFlash();
  }

  public void runIntake(double speed) {
    m_intake.set(speed);
  }

  public void extendGripper() {
    m_solenoid.set(Value.kForward);
  }

  public void retractGripper() {
    m_solenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
