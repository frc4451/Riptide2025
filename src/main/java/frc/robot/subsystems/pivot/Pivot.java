package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.superstructure.constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SingleRoller {
  public static final double isNearToleranceRotations = Units.degreesToRotations(5);

  public Pivot(String name, SingleRollerIO io) {
    super(name, io);
    setPosition(PivotConstants.intialPosition);
  }

  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        name + "/Profile/Setpoint/Deg",
        new LoggedTrapezoidState(
            Units.rotationsToDegrees(inputs.positionSetpointRotations),
            Units.rotationsToDegrees(inputs.velocitySetpointRotationsPerSec)));

    Logger.recordOutput(
        name + "/Profile/Goal/Deg",
        new LoggedTrapezoidState(Units.rotationsToDegrees(inputs.positionGoalRotations), 0));

    Logger.recordOutput(name + "/PositionDegrees", getPosition().getDegrees());
    Logger.recordOutput(name + "/VelocityDegreesPerSecond", getVelocity().getDegrees());
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(inputs.positionRotations);
  }

  public void setPosition(Rotation2d position) {
    io.resetPosition(position.getRotations());
  }

  public Rotation2d getVelocity() {
    return Rotation2d.fromRotations(inputs.velocityRotationsPerSec);
  }

  public void setGoal(Rotation2d angle) {
    io.setGoal(angle.getRotations());
  }

  public Rotation2d getGoalPosition() {
    return Rotation2d.fromRotations(inputs.positionGoalRotations);
  }

  public boolean isNear(Rotation2d position) {
    return MathUtil.isNear(
        inputs.positionRotations, position.getRotations(), isNearToleranceRotations);
  }

  public boolean atGoal() {
    return isNear(getGoalPosition());
  }
}
