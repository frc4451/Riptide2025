package frc.robot.subsystems.superstructures.corel.conveyor;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;

public class CorelConveyorConstants {
  public static final PIDConstants pidConstants = new PIDConstants(1.0);
  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 18.0 / 12.0;
  public static final double moi = 0.001;
}
