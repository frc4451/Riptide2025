package frc.robot.subsystems.superstructure.pivot;

import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class PivotIOSim extends SingleRollerIOSim {
  public PivotIOSim() {
    super(
        PivotConstants.gearbox,
        PivotConstants.reduction,
        PivotConstants.moi,
        PivotConstants.pidConstants);
  }
}
