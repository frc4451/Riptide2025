package frc.robot.subsystems.superstructure.elevatorV2;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.superstructure.elevator.CustomElevatorFF;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstraints;
import frc.robot.util.Gains;
import frc.robot.util.MotionMagicProps;

public class ElevatorV2Constants {
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
      new TrapezoidProfile.Constraints(50.0, 50.0);

  public static final boolean foc = true;

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html#motion-profiled-feedforward-and-feedback-control
  // No gravity in Sim, therefore no feedforward
  public static final CustomElevatorFF feedforward =
      Constants.currentMode == Mode.REAL
          ? new CustomElevatorFF(0.420, 0.41, 0, 0.09, 0.06, 0.0, 0.0)
          : new CustomElevatorFF(0, 0, 0, 0, 0, 0, 0);

  /** Circumference of Pully */
  public static final double elevatorPulleyCircum = (2 * Math.PI * inchesPerRad);
  /** Distance of travel for the pulley after going through reduction */
  public static final double motorConversionFactor = elevatorPulleyCircum / reduction;

  public static final DCMotor elevatorMotorSim = DCMotor.getFalcon500Foc(2);
  public static final double carriageMassKg = Units.lbsToKilograms(20);
  public static final boolean simulateGravity = true;
  public static final double motionMagicVelocity = 80;
  public static final double motionMagicAcceleration = 160;
  public static final double motionMagicJerk = 1600;

  public static final Gains staticGains = new Gains(0.41, 0.1, 0.01, 0.42, 1.0, 0.0);

  public static final MotionMagicProps motionMagicProps = new MotionMagicProps(
    80, 
    160,
     1600
     );
}
