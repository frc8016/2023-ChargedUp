package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax m_frontLeftMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_backLeftMotor =
      new CANSparkMax(DrivetrainConstants.BACK_LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_backRightMotor =
      new CANSparkMax(DrivetrainConstants.BACK_RIGHT_MOTOR_ID, MotorType.kBrushless);

  private final MotorControllerGroup m_leftControllerGroup =
      new MotorControllerGroup(m_frontLeftMotor, m_backLeftMotor);
  private final MotorControllerGroup m_rightControllerGroup =
      new MotorControllerGroup(m_frontRightMotor, m_backRightMotor);

  private final DifferentialDrive m_differentialDrive =
      new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

  private final RelativeEncoder m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRightMotor.getEncoder();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    resetOdometry();
    m_frontLeftEncoder.setPositionConversionFactor(6.0);
    m_frontRightEncoder.setPositionConversionFactor(6.0);
  }

  public void setLeftGroup(double speed) {
    m_leftControllerGroup.set(speed);
  }

  public void setRightControllerGroup(double speed) {
    m_rightControllerGroup.set(speed);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_leftControllerGroup.setInverted(true);
    m_differentialDrive.arcadeDrive(speed, rotation);
  }

  public double getLeftEncoderPosition() {
    return m_frontLeftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return m_frontRightEncoder.getPosition();
  }

  public void resetOdometry() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
