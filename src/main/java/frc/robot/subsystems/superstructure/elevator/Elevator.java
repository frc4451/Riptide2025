package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.follow.FollowRollers;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.superstructure.can_range.CanRange;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import org.littletonrobotics.junction.Logger;

public class Elevator extends FollowRollers {
  public static final double isNearToleranceInches = 1;

  private final TrapezoidProfile trapezoidProfile;
  private final CanRange heightSensor;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final CustomElevatorController controller;

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  public Elevator(
      String name,
      FollowRollersIO io,
      CanRangeIO heightSensorIO,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints,
      CustomElevatorController feedforward) {
    super(name, io);
    this.heightSensor = new CanRange(name + "/HeightSensor", heightSensorIO);
    this.trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.inchesPerRad = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
    this.controller = feedforward;
  }

  public void periodic() {
    super.periodic();

    // If we are close enough to our sensor, use that instead of our motor encoders.
    // We trust our height sensor more than our encoders at this point, this should account for
    // chain skipping. This doesn't happen in simulation.
    // if (getEncoderHeightInches() < ElevatorConstants.resetFromHeightSensorThresholdInches
    //     && Constants.currentMode != Mode.SIM) {
    //   io.resetPosition(getSensorHeightInches() / inchesPerRad);
    // }

    if (DriverStation.isDisabled()) {
      resetSetpoint();
    } else {
      runTrapezoidProfile();
    }

    Logger.recordOutput(
        name + "/Profile/SetpointInches",
        new LoggedTrapezoidState(setpoint.position, setpoint.velocity));

    Logger.recordOutput(
        name + "/Profile/GoalInches", new LoggedTrapezoidState(goal.position, goal.velocity));

    Logger.recordOutput(name + "/EncoderHeightInches", getEncoderHeightInches());
    Logger.recordOutput(name + "/SensorHeightInches", getSensorHeightInches());
    Logger.recordOutput(name + "/HeightInches", getHeightInches());
    Logger.recordOutput(name + "/VelocityInchesPerSec", getVelocityInchesPerSec());
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

  public double getVelocityInchesPerSec() {
    return inputs.leaderVelocityRadPerSec;
  }

  public void runTrapezoidProfile() {
    TrapezoidProfile.State newSetpoint =
        trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    runPosition(setpoint, newSetpoint);
    setpoint = newSetpoint;
  }

  private void runPosition(TrapezoidProfile.State setpoint, TrapezoidProfile.State newSetpoint) {
    // double ff = feedforward.calculate(setpoint.velocity, 0);
    double output =
        controller.calculateWithVelocities(
            getHeightInches(), newSetpoint.position, setpoint.velocity, newSetpoint.velocity);
    io.runVolts(output);
  }

  public void setHeightInches(double positionInches) {
    io.resetPosition(positionInches / inchesPerRad);
  }

  public void setGoalHeightInches(double positionInches) {
    double clampedPositionIn =
        MathUtil.clamp(
            positionInches,
            elevatorConstraints.minHeightInches(),
            elevatorConstraints.maxHeightInches());
    goal = new TrapezoidProfile.State(clampedPositionIn, 0.0);
  }

  public double getGoalHeightInches() {
    return goal.position;
  }

  private void resetSetpoint() {
    setpoint = new TrapezoidProfile.State(getHeightInches(), 0.0);
  }

  public boolean isNear(double heightInches) {
    return MathUtil.isNear(getHeightInches(), heightInches, isNearToleranceInches);
  }

  public boolean atGoal() {
    return isNear(getGoalHeightInches());
  }
}
