package frc.robot.subsystems.superstructure.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.pivot.PivotConstraints;

public class CoralPivotConstants {
  public static final int canId = 5;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 25.0 * (60.0 / 24.0);
  public static final double moi = 0.01;

  public static final boolean invert = true;
  public static final double currentLimitAmps = 30;

  public static final PivotConstraints pivotConstraints = new PivotConstraints(0, 10);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(Math.PI * 3.25, Math.PI * 2.5);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
  // No gravity in Sim, therefore no feedforward
  public static final ArmFeedforward feedforward =
      Constants.currentMode == Constants.Mode.REAL
          ? new ArmFeedforward(0, 0.3, 1.1)
          : new ArmFeedforward(0, 0, 0);
  public static final double kP = 1.8;
  public static final double kD = 0.0;
}
