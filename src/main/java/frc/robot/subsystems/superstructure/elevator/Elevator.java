package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.follow.FollowRollers;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import org.littletonrobotics.junction.Logger;

public class Elevator extends FollowRollers {
  private final TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  public Elevator(
      String name,
      FollowRollersIO io,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints) {
    super(name, io);
    trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.inchesPerRad = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
  }

  public void periodic() {
    super.periodic();

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

    Logger.recordOutput(name + "/HeightInches", getHeightInches());
  }

  public double getHeightInches() {
    return inputs.leaderPositionRad * inchesPerRad;
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

  private void resetController() {
    setGoalHeightInches(0.0);
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }
}
