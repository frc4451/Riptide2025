package frc.robot.subsystems.superstructure.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorConstants {
  public static final PIDConstants pidConstants = new PIDConstants(1.0);
  public static final DCMotor leaderGearbox = DCMotor.getFalcon500Foc(1);
  public static final DCMotor followerGearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 18.0 / 12.0;
  public static final double moi = 0.001;
  public static final double inchesPerRad = 3.0; // equal to radius

  public static final ElevatorConstraints elevatorConstraints = new ElevatorConstraints(0, 10);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(1.0 * inchesPerRad, 1.0 * inchesPerRad);
}
