package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.follow.FollowRollers;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.superstructure.can_range.CanRange;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends FollowRollers {
  public static final double atGoalToleranceInches = 1;

  private final TrapezoidProfile trapezoidProfile;
  private final CanRange heightSensor;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  public Elevator(
      String name,
      FollowRollersIO io,
      CanRangeIO heightSensorIO,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints) {
    super(name, io);
    this.heightSensor = new CanRange(name + "/HeightSensor", heightSensorIO);
    this.trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.inchesPerRad = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
  }

  public void periodic() {
    super.periodic();

    // If we are close enough to our sensor, use that instead of our motor encoders.
    // We trust our height sensor more than our encoders at this point, this should account for
    // chain skipping. This doesn't happen in simulation.
    if (getEncoderHeightInches() < ElevatorConstants.resetFromHeightSensorThresholdInches
        && Constants.currentMode != Mode.SIM) {
      io.resetPosition(getSensorHeightInches() / inchesPerRad);
    }

    if (DriverStation.isDisabled()) {
      resetController();
    } else {
      runTrapezoidProfile();
    }

    Logger.recordOutput(
        name + "/Profile/Setpoint/Rad",
        new LoggedTrapezoidState(setpoint.position, setpoint.velocity));
    Logger.recordOutput(
        name + "/Profile/Setpoint/In",
        new LoggedTrapezoidState(
            setpoint.position * inchesPerRad, setpoint.velocity * inchesPerRad));

    Logger.recordOutput(
        name + "/Profile/Goal/Rad", new LoggedTrapezoidState(goal.position, goal.velocity));
    Logger.recordOutput(
        name + "/Profile/Goal/In",
        new LoggedTrapezoidState(goal.position * inchesPerRad, goal.velocity * inchesPerRad));

    Logger.recordOutput(name + "/EncoderHeightInches", getEncoderHeightInches());
    Logger.recordOutput(name + "/SensorHeightInches", getSensorHeightInches());
    Logger.recordOutput(name + "/HeightInches", getHeightInches());
  }

  public double getEncoderHeightInches() {
    return inputs.leaderPositionRad * inchesPerRad;
  }

  public double getSensorHeightInches() {
    return Units.metersToInches(heightSensor.getDistanceMeters());
  }

  /** Defaults to using Encoder Height Inches */
  public double getHeightInches() {
    return getEncoderHeightInches();
  }

  public void runTrapezoidProfile() {
    setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    io.runPosition(setpoint.position);
  }

  public void setGoalHeightInches(double positionInches) {
    double clampedPositionRad =
        MathUtil.clamp(
            positionInches / inchesPerRad,
            elevatorConstraints.minHeightInches() / inchesPerRad,
            elevatorConstraints.maxHeightInches() / inchesPerRad);
    goal = new TrapezoidProfile.State(clampedPositionRad, 0.0);
  }

  public double getGoalHeightInches() {
    return goal.position * inchesPerRad;
  }

  public boolean underL4Threshold() {
    return getHeightInches() < ElevatorConstants.l4ThresholdInches;
  }

  private void resetController() {
    setGoalHeightInches(0.0);
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  public boolean atGoal() {
    return MathUtil.isNear(getGoalHeightInches(), getHeightInches(), atGoalToleranceInches);
  }
}
