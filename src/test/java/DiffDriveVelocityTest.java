import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.commands.Paths;
import frc.robot.commands.Ramsete;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

class DiffDriveVelocityTest {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Ramsete m_ramsete = new Ramsete(Paths.testTrajectory, m_drivetrain);

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

  public DiffDriveVelocityTest() {}

  // note to self: just use diff drive feedforward unit tests for example!
}
