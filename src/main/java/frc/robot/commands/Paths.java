package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutonConstants;
import java.util.List;

public class Paths {
  // TrajectoryConfig for general use
  private static final TrajectoryConfig m_config =
      new TrajectoryConfig(AutonConstants.MAX_VELOCITY, AutonConstants.MAX_ACCELERATION);

  // A trajectory for testing purposes with arbitrary waypoints and start/end positions
  public static final Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(),
          List.of(new Translation2d(1, 2), new Translation2d(0, 4)),
          new Pose2d(),
          m_config);

  public Paths() {}
}
