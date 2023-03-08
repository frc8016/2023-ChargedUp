// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem {
  private final CANSparkMax m_leftShoulderMotor =
      new CANSparkMax(ArmConstants.LEFT_SHOULDER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightShoulderMotor =
      new CANSparkMax(ArmConstants.RIGHT_SHOULDER_MOTOR_ID, MotorType.kBrushless);
  private final Encoder m_relativeEncoder =
      new Encoder(ArmConstants.RELATIVE_ENCODER_A, ArmConstants.RELATIVE_ENCODER_B);
  private final DutyCycleEncoder m_absoluteEncoder =
      new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_PORT);

  private final ArmFeedforward m_armFeedforward =
      new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The constraints for the generated profiles
        new ProfiledPIDController(
            ArmConstants.kp,
            ArmConstants.ki,
            ArmConstants.kd,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecondSquared),
            ArmConstants.kArmOffsetRadians));
    m_relativeEncoder.setDistancePerPulse(ArmConstants.kRelativeEncoderRadiansPerPulse);
    setGoal(ArmConstants.kArmOffsetRadians);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_armFeedforward.calculate(setpoint.position, setpoint.velocity);
    m_leftShoulderMotor.setVoltage(output + feedforward);
    m_rightShoulderMotor.follow(m_leftShoulderMotor, true);
  }

  @Override
  public double getMeasurement() {
    return m_relativeEncoder.getDistance() + ArmConstants.kArmOffsetRadians;
  }
}
