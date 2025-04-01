package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import org.littletonrobotics.junction.Logger;

public class ElevatorSingle extends SingleRoller {
  public static final double isNearToleranceInches = 1;

  private final double inchesPerRotation;

  public ElevatorSingle(String name, SingleRollerIO io, double inchesPerRad) {
    super(name, io);
    this.inchesPerRotation = inchesPerRad;
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
    return inputs.positionRotations * inchesPerRotation;
  }

  public double getVelocityInchesPerSec() {
    return inputs.velocityRotationsPerSec * inchesPerRotation;
  }

  public void setHeightInches(double positionInches) {
    io.resetPosition(positionInches / inchesPerRotation);
  }

  public void setGoalHeightInches(double positionInches) {
    io.setGoal(positionInches / inchesPerRotation);
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
