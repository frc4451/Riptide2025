package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.follow_magic.FollowRollersMagic;
import frc.robot.subsystems.rollers.follow_magic.FollowRollersMagicIO;
import org.littletonrobotics.junction.Logger;

public class Elevator extends FollowRollersMagic {
  public static final double isNearToleranceInches = 1;

  private final double inchesPerRotation;
  private final ElevatorConstraints elevatorConstraints;

  public Elevator(
      String name,
      FollowRollersMagicIO io,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints) {
    super(name, io);
    this.inchesPerRotation = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
  }

  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        name + "/Profile/SetpointInches",
        new LoggedTrapezoidState(
            inputs.positionSetpointRotations * inchesPerRotation,
            inputs.velocitySetpointRotationsPerSec * inchesPerRotation));

    Logger.recordOutput(
        name + "/Profile/GoalInches", new LoggedTrapezoidState(getGoalHeightInches(), 0));

    Logger.recordOutput(name + "/HeightInches", getHeightInches());
    Logger.recordOutput(name + "/VelocityInchesPerSec", getVelocityInchesPerSec());
  }

  public double getHeightInches() {
    return inputs.leaderPositionRotations * inchesPerRotation;
  }

  public double getVelocityInchesPerSec() {
    return inputs.leaderVelocityRotationsPerSec * inchesPerRotation;
  }

  public void setHeightInches(double positionInches) {
    io.resetPosition(positionInches / inchesPerRotation);
  }

  public void setGoalHeightInches(double positionInches) {
    double clampedPositionIn =
        MathUtil.clamp(
            positionInches,
            elevatorConstraints.minHeightInches(),
            elevatorConstraints.maxHeightInches());
    io.setGoal(clampedPositionIn / inchesPerRotation);
  }

  public double getGoalHeightInches() {
    return inputs.positionGoalRotations * inchesPerRotation;
  }

  public boolean isNear(double heightInches) {
    return MathUtil.isNear(getHeightInches(), heightInches, isNearToleranceInches);
  }

  public boolean atGoal() {
    return isNear(inputs.positionGoalRotations * inchesPerRotation);
  }
}
