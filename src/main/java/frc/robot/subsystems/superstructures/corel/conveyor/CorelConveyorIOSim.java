package frc.robot.subsystems.superstructures.corel.conveyor;

import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class CorelConveyorIOSim extends SingleRollerIOSim {
  public CorelConveyorIOSim() {
    super(
        CorelConveyorConstants.gearbox,
        CorelConveyorConstants.reduction,
        CorelConveyorConstants.moi,
        CorelConveyorConstants.pidConstants);
  }
}
