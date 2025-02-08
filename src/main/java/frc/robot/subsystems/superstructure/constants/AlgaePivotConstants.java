package frc.robot.subsystems.superstructure.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.superstructure.pivot.PivotConstraints;

public class AlgaePivotConstants {
  public static final int canId = 0;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 5.0;
  public static final double moi = 0.01;

  public static final Rotation2d initialAngle = Rotation2d.fromDegrees(-90.0);

  public static final boolean invert = false;
  public static final double currentLimitAmps = 30;

  public static final PivotConstraints pivotConstraints = new PivotConstraints(0, 10);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(Math.PI / 4.0, Math.PI / 4.0);
}
