package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PivotConstants {
  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 5.0;
  public static final double moi = 0.01;
  public static final PivotConstraints pivotConstraints = new PivotConstraints(0, 10);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(Math.PI / 4.0, Math.PI / 4.0);
}
