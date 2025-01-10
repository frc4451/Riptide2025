package frc.robot.subsystems.rollers.pivot;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;

public class PivotSubsystem extends SingleRollerSubsystem {
  private final PivotVisualizer pivotVisualizer = new PivotVisualizer("Measured", Color.kBlack);

  public PivotSubsystem(PivotIO io) {
    super("Pivot", io);
  }

  @Override
  public void periodic() {
    super.periodic();

    this.pivotVisualizer.update(this.inputs.positionRad);
  }
}
