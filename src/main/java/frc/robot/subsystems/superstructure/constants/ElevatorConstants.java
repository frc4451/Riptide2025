package frc.robot.subsystems.superstructure.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.elevator.CustomElevatorController;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstraints;

public class ElevatorConstants {
  public static final int heightSensorId = 0;

  public static final int leaderCanId = 6;
  public static final DCMotor leaderGearbox = DCMotor.getFalcon500(1);

  public static final int followerCanId = 7;
  public static final DCMotor followerGearbox = DCMotor.getFalcon500(1);

  public static final double reduction = 5.0;
  public static final double moi = 0.01;
  public static final double inchesPerRad = 0.88; // equal to radius

  public static final boolean invert = false;
  public static final boolean invertFollower = true;
  public static final double currentLimitAmps = 60;

  public static final boolean isBrake = true;

  public static final double resetFromHeightSensorThresholdInches = 5;
  public static final ElevatorConstraints elevatorConstraints =
      new ElevatorConstraints(1.0 / 2.0, 50 / 2.0);
  public static final double startHeightInches = -0.5 / 2.0;

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      //   new TrapezoidProfile.Constraints(50.0, 50.0);
      new TrapezoidProfile.Constraints(40.0, 200.0);

  public static final boolean foc = true;

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#motion-profiled-feedforward-and-feedback-control
  // No gravity in Sim, therefore no feedforward
  public static final CustomElevatorController feedforward =
      Constants.currentMode == Mode.REAL
          ? new CustomElevatorController(
              // feedback
              0.1,
              0.7,
              0.0,
              0.0,
              // feedforward
              0.460,
              0.22,
              0.22,
              0.10,
              0.07,
              0.004,
              0.006)
          : new CustomElevatorController(0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}
