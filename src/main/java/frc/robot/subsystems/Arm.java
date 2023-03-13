// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  // Simulation classes; simulation is broken for ProfiledPID setGoal().
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2), 160.0, 4.682, 1.016, ArmConstants.kArmOffsetRadians, 0.0, true);
  private final EncoderSim m_relativeEncoderSim = new EncoderSim(m_relativeEncoder);

  private double m_relativeOffsetRadians;

  // Create arm SmartDashboard visualization
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

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
                ArmConstants.kMaxAccelerationRadPerSecondSquared)),
        -Math.PI / 2);

    m_relativeEncoder.reset();
    m_relativeEncoder.setDistancePerPulse(ArmConstants.kRelativeEncoderRadiansPerPulse);
    m_absoluteEncoder.setDistancePerRotation(Math.PI * 2);

    m_relativeOffsetRadians = ArmConstants.kArmOffsetRadians - m_absoluteEncoder.getDistance();

    SmartDashboard.putData("Arm PID", getController());

    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  // Runs with arm with non-predictive feedforward control
  public void set(double speed) {
    m_leftShoulderMotor.set(speed);
    m_rightShoulderMotor.follow(m_leftShoulderMotor, true);
  }

  // Returns the raw absolute position of the arm; this is needed to find kArmOffsetRadians
  public double getRawAbsolutePosition() {
    return m_absoluteEncoder.getDistance();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_armFeedforward.calculate(setpoint.position, setpoint.velocity);
    m_leftShoulderMotor.setVoltage(output + feedforward);
    m_rightShoulderMotor.follow(m_leftShoulderMotor, true);
  }

  @Override
  public double getMeasurement() {
    return m_relativeEncoder.getDistance() + m_relativeOffsetRadians;
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInputVoltage(m_leftShoulderMotor.get() * RobotController.getInputVoltage());
    m_armSim.update(0.020);
    m_relativeEncoderSim.setDistance(m_armSim.getAngleRads());
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }
}
