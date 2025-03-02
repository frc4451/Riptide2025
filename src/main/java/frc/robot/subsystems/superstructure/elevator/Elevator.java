package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
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

  private final CustomElevatorFF feedforward;

  private final PIDController positionController;

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  public Elevator(
      String name,
      FollowRollersIO io,
      CanRangeIO heightSensorIO,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints,
      CustomElevatorFF feedforward,
      double kP,
      double kD) {
    super(name, io);
    this.heightSensor = new CanRange(name + "/HeightSensor", heightSensorIO);
    this.trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.inchesPerRad = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
    this.feedforward = feedforward;
    positionController = new PIDController(kP, 0, kD);
    Logger.recordOutput(name + "/Feedforward", feedforward);
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
      resetController();
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
    setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    // setpoint = new TrapezoidProfile.State(0, 6);
    runPosition(setpoint);
  }

  private void runPosition(TrapezoidProfile.State setpoint) {
    double ff = feedforward.calculate(setpoint.velocity, 0);
    double output = positionController.calculate(getHeightInches(), setpoint.position);
    io.runVolts(output + ff);
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

  public boolean underL4Threshold() {
    return getHeightInches() < ElevatorConstants.l4ThresholdInches;
  }

  private void resetController() {
    setpoint = new TrapezoidProfile.State(getHeightInches(), 0.0);
  }

  public boolean atGoal() {
    return MathUtil.isNear(getGoalHeightInches(), getHeightInches(), atGoalToleranceInches);
  }
}
