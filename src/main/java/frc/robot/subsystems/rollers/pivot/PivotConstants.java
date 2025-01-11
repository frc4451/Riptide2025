package frc.robot.subsystems.rollers.pivot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;

public class PivotConstants {
  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 18.0 / 12.0;
  public static final double moi = 0.001;

  public static PIDConstants pidConstants = new PIDConstants(1, 0, 0);
}
