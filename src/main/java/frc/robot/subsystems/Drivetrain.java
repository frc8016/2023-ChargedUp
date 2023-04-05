package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  // Drivetrain control
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

  // Odometry
  private final CANCoder m_leftDriveEncoder =
      new CANCoder(DrivetrainConstants.LEFT_DRIVE_ENCODER_ID);
  private final CANCoder m_rightDriveEncoder =
      new CANCoder(DrivetrainConstants.RIGHT_DRIVE_ENCODER_ID);

  private final WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(DrivetrainConstants.PIGEON_ID);

  private final DifferentialDriveKinematics m_driveKinematics =
      new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH_METERS);

  private final DifferentialDrivePoseEstimator m_drivePoseEstimator =
      new DifferentialDrivePoseEstimator(
          m_driveKinematics,
          m_pigeon.getRotation2d(),
          m_leftDriveEncoder.getPosition(),
          m_rightDriveEncoder.getPosition(),
          new Pose2d(),
          DrivetrainConstants.STATE_STD_DEVS,
          DrivetrainConstants.VISION_STD_DEVS);

  private final Transform3d m_cameraToRobot =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // System Characteristics

  // Create state-space model of drivetrain with states [left velocity, right velocity], inputs
  // [left voltage, right voltage], and outputs [left velocity, right velocity].
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.createDrivetrainVelocitySystem(
          DCMotor.getNEO(2),
          DrivetrainConstants.MASS_KG,
          DrivetrainConstants.WHEEL_RADIUS_METERS,
          DrivetrainConstants.BASE_RADIUS_METERS,
          DrivetrainConstants.I,
          DrivetrainConstants.GEARING);

  // Model-based drivetrain feedforward discretization timestep is assumed to be 20ms
  private final LinearPlantInversionFeedforward m_feedforward =
      new LinearPlantInversionFeedforward<>(m_drivetrainSystem, .02);

  // PID Controllers for left and right sides of drivetrain
  private final PIDController m_leftPID = new PIDController(DrivetrainConstants.kp_left, 0.0, 0.0);
  private final PIDController m_rightPID =
      new PIDController(DrivetrainConstants.kp_right, 0.0, 0.0);

  // Data Logging
  private DataLog m_log = DataLogManager.getLog();
  private DoubleLogEntry l_frontLeftMotorCurrent =
      new DoubleLogEntry(m_log, "/custom/drivetrain/frontleft/current");
  private DoubleLogEntry l_frontLeftMotorTemp =
      new DoubleLogEntry(m_log, "/custom/drivetrain/frontleft/temp");
  private DoubleLogEntry l_backLeftMotorCurrent =
      new DoubleLogEntry(m_log, "/custom/drivetrain/backleft/current");
  private DoubleLogEntry l_backLeftMotorTemp =
      new DoubleLogEntry(m_log, "/custom/drivetrain/backleft/temp");
  private DoubleLogEntry l_frontrightMotorCurrent =
      new DoubleLogEntry(m_log, "/custom/drivetrain/frontright/current");
  private DoubleLogEntry l_frontrightMotorTemp =
      new DoubleLogEntry(m_log, "/custom/drivetrain/frontright/temp");
  private DoubleLogEntry l_backrightMotorCurrent =
      new DoubleLogEntry(m_log, "/custom/drivetrain/backright/current");
  private DoubleLogEntry l_backrightMotorTemp =
      new DoubleLogEntry(m_log, "/custom/drivetrain/backright/temp");

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    resetOdometry();
  }

  public void arcadeDrive(double speed, double rotation) {
    m_leftControllerGroup.setInverted(true);
    m_differentialDrive.arcadeDrive(speed, rotation);
  }

  // <TODO> Smart "homing" for odometry which syncs with vision measurements
  public void resetOdometry() {
    // Configure encoder parameters
    CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
    encoderConfig.sensorCoefficient = 0.479 / 4096.0;
    encoderConfig.unitString = "Meters";
    encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    // Flash encoder parameters
    // <TODO> one encoder must be inverted
    m_leftDriveEncoder.configAllSettings(encoderConfig);
    m_rightDriveEncoder.configAllSettings(encoderConfig);

    // Invert left drivetrain encoder so that clockwise rotation results in positive sensor
    // measurements
    m_leftDriveEncoder.configSensorDirection(true);

    // Set encoder positions and gyro heading to 0
    m_leftDriveEncoder.setPosition(0.0);
    m_rightDriveEncoder.setPosition(0.0);
    m_pigeon.reset();
  }

  public void updateOdometry() {
    // Update pose estimator with latest gyro and encoder readings
    m_drivePoseEstimator.update(
        m_pigeon.getRotation2d(),
        m_leftDriveEncoder.getPosition(),
        m_rightDriveEncoder.getPosition());

    // Retrieve limelight pose measurements from NetworkTables
    double[] botPose =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose")
            .getDoubleArray(new double[7]);

    // Reconstruct robot position from double array
    Pose3d cameraInField =
        new Pose3d(
            botPose[0], botPose[1], botPose[2], new Rotation3d(botPose[3], botPose[4], botPose[5]));

    // Add camera-to-robot transformation to camera position to compute robot position
    Pose3d visionMeasurement3d = cameraInField.plus(m_cameraToRobot);

    // Transform robot's field-relative position from Pose2d to Pose3d required for
    // addVisionMeasurements()
    Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();

    // Apply vision measurements to DifferentialDrivePoseEstimator class
    m_drivePoseEstimator.addVisionMeasurement(visionMeasurement2d, botPose[6]);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Compute left and right wheel speeds from ChassisSpeeds object
    DifferentialDriveWheelSpeeds wheelSpeeds = m_driveKinematics.toWheelSpeeds(chassisSpeeds);

    // Populate model-based feedforward reference vector with current wheel speeds
    Matrix<N2, N1> r =
        VecBuilder.fill(m_leftDriveEncoder.getVelocity(), m_rightDriveEncoder.getVelocity());

    // Populate model-based feedforward future reference vector with wheel speeds from function
    // parameter (most likely generated by Ramsete)
    Matrix<N2, N1> nextR =
        VecBuilder.fill(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

    // Compute controller output given current and future reference vectors
    Matrix<N2, N1> u = m_feedforward.calculate(r, nextR);

    // Compute left and right drivetrain outputs
    double leftWheelVoltage =
        u.get(0, 0)
            + m_leftPID.calculate(
                m_leftDriveEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
    double rightWheelVoltage =
        u.get(0, 1)
            + m_rightPID.calculate(
                m_rightDriveEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);

    // Feed outputs into motor controllers
    m_leftControllerGroup.set(leftWheelVoltage);
    m_rightControllerGroup.set(rightWheelVoltage);
  }

  // Retrieve robot pose from DifferentialDrivePoseEstimator
  public Pose2d getEstimatedPosition() {
    return m_drivePoseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logDrivetrainMotors();
    updateOdometry();
  }

  private void logDrivetrainMotors() {
    l_frontLeftMotorCurrent.append(m_frontLeftMotor.getOutputCurrent());
    l_frontLeftMotorTemp.append(m_frontLeftMotor.getMotorTemperature());
    l_backLeftMotorCurrent.append(m_backLeftMotor.getOutputCurrent());
    l_backLeftMotorTemp.append(m_backLeftMotor.getMotorTemperature());
    l_frontrightMotorCurrent.append(m_frontRightMotor.getOutputCurrent());
    l_frontrightMotorTemp.append(m_frontRightMotor.getMotorTemperature());
    l_backrightMotorCurrent.append(m_backRightMotor.getOutputCurrent());
    l_backrightMotorTemp.append(m_backRightMotor.getMotorTemperature());
  }

  private void configureMotors() {
    // Reset all the motors to factory defaults (except CAN ID)
    m_frontLeftMotor.restoreFactoryDefaults();
    m_backLeftMotor.restoreFactoryDefaults();
    m_frontRightMotor.restoreFactoryDefaults();
    m_backRightMotor.restoreFactoryDefaults();

    // Set all the motor's idle modes to Coast
    if (m_frontLeftMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
      System.out.println("ERROR while setting Front Left Motor to Coast Mode");
    }
    if (m_backLeftMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
      System.out.println("ERROR while setting Back Left Motor to Coast Mode");
    }
    if (m_frontRightMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
      System.out.println("ERROR while setting Front Right Motor to Coast Mode");
    }
    if (m_backRightMotor.setIdleMode(IdleMode.kCoast) != REVLibError.kOk) {
      System.out.println("ERROR while setting Back Right Motor to Coast Mode");
    }

    // Set current limits for all motors
    if (m_frontLeftMotor.setSmartCurrentLimit(DrivetrainConstants.MOTOR_CURRENT_LIMIT)
        != REVLibError.kOk) {
      System.out.println("ERROR while setting Front Left Motor smart current limit");
    }
    if (m_backLeftMotor.setSmartCurrentLimit(DrivetrainConstants.MOTOR_CURRENT_LIMIT)
        != REVLibError.kOk) {
      System.out.println("ERROR while setting Back Left Motor smart current limit");
    }
    if (m_frontRightMotor.setSmartCurrentLimit(DrivetrainConstants.MOTOR_CURRENT_LIMIT)
        != REVLibError.kOk) {
      System.out.println("ERROR while setting Front Right Motor smart current limit");
    }
    if (m_backRightMotor.setSmartCurrentLimit(DrivetrainConstants.MOTOR_CURRENT_LIMIT)
        != REVLibError.kOk) {
      System.out.println("ERROR while setting Back Right Motor smart current limit");
    }

    // Burn all changed settings to flash
    m_frontLeftMotor.burnFlash();
    m_backLeftMotor.burnFlash();
    m_frontRightMotor.burnFlash();
    m_backRightMotor.burnFlash();
  }
}
