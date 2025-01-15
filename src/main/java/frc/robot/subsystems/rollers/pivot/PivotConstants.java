package frc.robot.subsystems.rollers.pivot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PivotConstants {
  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 18.0 / 12.0;
  public static final double moi = 0.001;
  public static final PivotConstraints pivotConstraints = new PivotConstraints(0, 10);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(1.0, 1.0);
  public static final PIDConstants pidConstants = new PIDConstants(1, 0, 0);
}
