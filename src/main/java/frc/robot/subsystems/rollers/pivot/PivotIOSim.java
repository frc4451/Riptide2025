package frc.robot.subsystems.rollers.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;

public class PivotIOSim extends SingleRollerIOSim implements PivotIO {
  private static final DCMotor motorModel = DCMotor.getFalcon500Foc(1);
  private static final double reduction = 18.0 / 12.0;
  private static final double moi = 0.001;

  public PivotIOSim() {
    super(motorModel, reduction, moi);
  }
}
