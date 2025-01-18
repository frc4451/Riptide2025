package frc.robot.subsystems.superstructure.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;

public class ElevatorIOSim extends FollowRollersIOSim {
  public ElevatorIOSim(
      String name,
      DCMotor leaderGearbox,
      DCMotor followerGearbox,
      double reduction,
      double moi,
      PIDConstants pidConstants,
      boolean invertFollower) {
    super(leaderGearbox, followerGearbox, reduction, moi, pidConstants, invertFollower);
  }

  public ElevatorIOSim() {
    this(
        "Elevator",
        ElevatorConstants.leaderGearbox,
        ElevatorConstants.followerGearbox,
        ElevatorConstants.reduction,
        ElevatorConstants.moi,
        ElevatorConstants.pidConstants,
        true);
  }
}
