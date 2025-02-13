package frc.robot.subsystems.superstructure.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstraints;

public class ElevatorConstants {
  public static final int heightSensorId = 0;

  public static final int leaderCanId = 0;
  public static final DCMotor leaderGearbox = DCMotor.getFalcon500Foc(1);

  public static final int followerCanId = 0;
  public static final DCMotor followerGearbox = DCMotor.getFalcon500Foc(1);

  public static final double reduction = 5.0;
  public static final double moi = 0.01;
  public static final double inchesPerRad = 0.88; // equal to radius

  public static final boolean invertFollower = true;
  public static final double currentLimitAmps = 30;

  public static final double l4ThresholdInches = 25; // TODO: find actual number
  public static final ElevatorConstraints elevatorConstraints = new ElevatorConstraints(0, 40);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(10.0 * inchesPerRad, 10.0 * inchesPerRad);
}
