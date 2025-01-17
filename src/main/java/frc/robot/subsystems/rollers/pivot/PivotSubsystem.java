package frc.robot.subsystems.rollers.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;

public class PivotSubsystem extends SingleRollerSubsystem {
  private final PivotVisualizer pivotVisualizer = new PivotVisualizer("Measured", Color.kBlack);

  private Rotation2d angle = new Rotation2d();

  public PivotSubsystem(PivotIO io) {
    this("Pivot", io);
  }

  public PivotSubsystem(String name, PivotIO io) {
    super(name, io);
  }

  @Override
  public void periodic() {
    super.periodic();

    this.pivotVisualizer.update(this.inputs.positionRad);
  }
}
