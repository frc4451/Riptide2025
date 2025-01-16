package frc.robot.state_machines;

import frc.robot.dashboard.ReefTreeSelector;
import frc.robot.dashboard.ReefTreeSelector.ReefTree;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static ReefTree currentReef = ReefTreeSelector.A;

  public static void updateReef(ReefTree newReef) {
    BobotState.currentReef = newReef;
  }

  public static ReefTree getReef() {
    return currentReef;
  }

  @Override
  public void periodic() {
    // publisher.
    Logger.recordOutput(logRoot + "Value", BobotState.currentReef);
  }

  @Override
  public void simulationPeriodic() {}
}
