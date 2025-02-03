package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorConstants {
  public static final int leaderCanId = 0;
  public static final DCMotor leaderGearbox = DCMotor.getFalcon500Foc(1);

  public static final int followerCanId = 0;
  public static final DCMotor followerGearbox = DCMotor.getFalcon500Foc(1);

  public static final double reduction = 5.0;
  public static final double moi = 0.01;
  public static final double inchesPerRad = 0.88; // equal to radius

  public static final boolean invertFollower = true;
  public static final double currentLimitAmps = 30;

  public static final ElevatorConstraints elevatorConstraints = new ElevatorConstraints(0, 10);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(1.0 * inchesPerRad, 1.0 * inchesPerRad);
}
