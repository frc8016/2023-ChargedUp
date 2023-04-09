package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;

public class DiffDriveVelocitySystemConstraint implements TrajectoryConstraint {
  private final LinearSystem<N2, N2, N2> m_system;
  private final DifferentialDriveKinematics m_kinematics;
  private final double m_maxVoltage;
  private final double m_maxVelocity;

  public DiffDriveVelocitySystemConstraint(
      LinearSystem<N2, N2, N2> system, DifferentialDriveKinematics kinematics, double maxVoltage) {
    m_system = system;
    m_kinematics = kinematics;
    m_maxVoltage = maxVoltage;

    Matrix<N2, N1> u = VecBuilder.fill(m_maxVoltage, m_maxVoltage);

    Matrix<N2, N1> maxX = m_system.getA().solve(m_system.getB().times(u));

    m_maxVelocity = maxX.get(0, 0);
  }

  public double getMaxVelocityMetersPerSecond(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    DifferentialDriveWheelSpeeds speeds =
        m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                velocityMetersPerSecond, 0, velocityMetersPerSecond * curvatureRadPerMeter));
    Matrix<N2, N1> x = VecBuilder.fill(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);

    if (Math.abs(x.get(0, 0)) > m_maxVelocity || Math.abs(x.get(1, 0)) > m_maxVelocity) {
      x = x.times(m_maxVelocity / x.maxAbs());
    }

    return (x.get(0, 0) + x.get(1, 0)) / 2.0;
  }

  public MinMax getMinMaxAccelerationMetersPerSecondSq(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    DifferentialDriveWheelSpeeds speeds =
        m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                velocityMetersPerSecond, 0, velocityMetersPerSecond * curvatureRadPerMeter));

    Matrix<N2, N1> x = VecBuilder.fill(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);

    Matrix<N2, N1> xDot;
    Matrix<N2, N1> u;

    // dx/dt for minimum u
    u = VecBuilder.fill(-m_maxVoltage, -m_maxVoltage);
    xDot = m_system.getA().times(x).plus(m_system.getB().times(u));
    double minChassisAcceleration = (xDot.get(0, 0) * xDot.get(1, 0)) / 2.0;

    // dx/dt for maximum u
    u = VecBuilder.fill(m_maxVoltage, m_maxVoltage);
    xDot = m_system.getA().times(x).plus(m_system.getB().times(u));
    double maxChassisAcceleration = (xDot.get(0, 0) * xDot.get(1, 0)) / 2.0;

    return new MinMax(minChassisAcceleration, maxChassisAcceleration);
  }
}
