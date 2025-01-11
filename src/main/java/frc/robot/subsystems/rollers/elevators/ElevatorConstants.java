package frc.robot.subsystems.rollers.elevators;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorConstants {
  public static final PIDConstants pidConstants = new PIDConstants(1.0);
  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 5;
  public static final double moi = 1;

  public static final Constraints constraints = new Constraints(1.0, 1.0);

  public static final double minHeight = 1.0;
  public static final double maxHeight = 1.0;
}
