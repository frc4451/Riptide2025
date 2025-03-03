package frc.robot.subsystems.superstructure.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.elevator.CustomElevatorFF;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstraints;

public class ElevatorConstants {
  public static final int heightSensorId = 0;

  public static final int leaderCanId = 6;
  public static final DCMotor leaderGearbox = DCMotor.getFalcon500Foc(1);

  public static final int followerCanId = 7;
  public static final DCMotor followerGearbox = DCMotor.getFalcon500Foc(1);

  public static final double reduction = 5.0;
  public static final double moi = 0.01;
  public static final double inchesPerRad = 0.88; // equal to radius

  public static final boolean invert = false;
  public static final boolean invertFollower = true;
  public static final double currentLimitAmps = 60;

  public static final double l4ThresholdInches = 25; // TODO: find actual number
  public static final double resetFromHeightSensorThresholdInches = 5;
  public static final ElevatorConstraints elevatorConstraints =
      new ElevatorConstraints(1.0 / 2.0, 50 / 2.0);
  public static final double startHeightInches = -0.5 / 2.0;

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(40.0, 30.0);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#motion-profiled-feedforward-and-feedback-control
  public static final CustomElevatorFF feedforward =
      new CustomElevatorFF(0.350, 0.41, 0, 0.14, 0.06, 0.0, 0.0);
  public static final double kP = 0.4;
  public static final double kD = 0.0;

  public static final double kG = 0.35;
  public static final double kS = 0.41;
  public static final double kV = 0.14;
  public static final double kA = 0.01;

  public static final double massKg = Units.lbsToKilograms(50);
  public static final double pullyCircumference = (2 * Math.PI * ElevatorConstants.inchesPerRad);
  public static final DCMotor elevatorSimMotorConfig = DCMotor.getFalcon500(2);
}
