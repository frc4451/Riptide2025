package frc.robot.subsystems.rollers.pivot;

import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class PivotIOSim extends SingleRollerIOSim implements PivotIO {
  public PivotIOSim() {
    super(
        PivotConstants.gearbox,
        PivotConstants.reduction,
        PivotConstants.moi,
        PivotConstants.pidConstants);
  }
}
