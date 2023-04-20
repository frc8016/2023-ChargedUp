import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Drivetrain;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DiffDriveVelocityTest {
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final double kTranslationTolerance = 1;
  private final double kRotationTolerance = Math.toRadians(30);

  @BeforeEach
  void setup() {
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
  }

  @AfterEach
  void shutdown() {
    SimHooks.resumeTiming();
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
  }

  @Test
  void testFeedforwardPose() {
    final TrajectoryConfig config =
        new TrajectoryConfig(6, 1)
            .setKinematics(m_drivetrain.driveKinematics)
            .addConstraint(new CentripetalAccelerationConstraint(.5))
            .addConstraint(m_drivetrain.constraint);

    final Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(new Translation2d(1, 0)),
            new Pose2d(new Translation2d(2, 0), new Rotation2d()),
            config);

    final double totalTime = trajectory.getTotalTimeSeconds();

    final double kDt = 0.02;

    for (int i = 0; i < (totalTime / kDt); i++) {
      Trajectory.State currentState = trajectory.sample(kDt * i);
      Trajectory.State futureState = trajectory.sample(kDt * (i + 1));

      DifferentialDriveWheelSpeeds currentWheelSpeeds =
          m_drivetrain.driveKinematics.toWheelSpeeds(
              new ChassisSpeeds(
                  currentState.velocityMetersPerSecond,
                  0,
                  currentState.velocityMetersPerSecond * currentState.curvatureRadPerMeter));

      DifferentialDriveWheelSpeeds futureWheelSpeeds =
          m_drivetrain.driveKinematics.toWheelSpeeds(
              new ChassisSpeeds(
                  futureState.velocityMetersPerSecond,
                  0,
                  futureState.velocityMetersPerSecond * futureState.curvatureRadPerMeter));

      Matrix<N2, N1> r =
          VecBuilder.fill(
              currentWheelSpeeds.leftMetersPerSecond, currentWheelSpeeds.rightMetersPerSecond);
      Matrix<N2, N1> nextR =
          VecBuilder.fill(
              futureWheelSpeeds.leftMetersPerSecond, futureWheelSpeeds.rightMetersPerSecond);

      Matrix<N2, N1> u = m_drivetrain.feedforward.calculate(r, nextR);

      m_drivetrain.voltageDrive(u.get(0, 0), u.get(1, 0));

      m_drivetrain.simulationPeriodic();

      SimHooks.stepTiming(kDt);
      System.out.println(m_drivetrain.getEstimatedPosition());
      System.out.println("Left Voltage: " + u.get(0, 0) + " RightVoltage: " + u.get(1, 0));

      //     assertAll(
      //       () ->
      //         assertEquals(
      //           futureState.poseMeters.getX(),
      //         m_drivetrain.getEstimatedPosition().getX(),
      //       kTranslationTolerance),
      //          () ->
      //            assertEquals(
      //              futureState.poseMeters.getY(),
      //            m_drivetrain.getEstimatedPosition().getY(),
      //          kTranslationTolerance),
      //          () ->
      //            assertEquals(
      //              futureState.poseMeters.getRotation().getRadians(),
      //            m_drivetrain.getEstimatedPosition().getRotation().getRadians(),
      //          kRotationTolerance));
    }
  }

  public DiffDriveVelocityTest() {}

  // note to self: just use diff drive feedforward unit tests for example!
}
