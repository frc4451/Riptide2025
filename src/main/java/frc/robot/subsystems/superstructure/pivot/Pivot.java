package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SingleRoller {
  public static final double atGoalToleranceRad = Units.degreesToRadians(3);

  private final TrapezoidProfile trapezoidProfile;

  private final PivotConstraints pivotConstraints;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  public Pivot(
      String name,
      SingleRollerIO io,
      TrapezoidProfile.Constraints trapezoidConstraints,
      PivotConstraints pivotConstraints) {
    super(name, io);
    trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.pivotConstraints = pivotConstraints;
    setPosition(0);
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
        name + "/Profile/Setpoint/Deg",
        new LoggedTrapezoidState(
            Units.radiansToDegrees(setpoint.position), Units.radiansToDegrees(setpoint.velocity)));

    Logger.recordOutput(
        name + "/Profile/Goal/Rad", new LoggedTrapezoidState(goal.position, goal.velocity));
    Logger.recordOutput(
        name + "/Profile/Goal/Deg",
        new LoggedTrapezoidState(
            Units.radiansToDegrees(goal.position), Units.radiansToDegrees(goal.velocity)));

    Logger.recordOutput(name + "/PositionDegrees", getPosition().getDegrees());
    Logger.recordOutput(name + "/VelocityDegreesPerSecond", getVelocity().getDegrees());
  }

  public void runTrapezoidProfile() {
    // setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    // io.runPosition(setpoint.position);
  }

  public Rotation2d getPosition() {
    return new Rotation2d(inputs.positionRad);
  }

  public void setPosition(double positionRad) {
    io.resetPosition(positionRad);
  }

  public Rotation2d getVelocity() {
    return new Rotation2d(inputs.velocityRadPerSec);
  }

  public void setGoal(Rotation2d angle) {
    setGoal(angle.getRadians());
  }

  public void setGoal(double angleRad) {
    double clampedPosition =
        MathUtil.clamp(angleRad, pivotConstraints.minRadians(), pivotConstraints.maxRadians());
    goal = new TrapezoidProfile.State(clampedPosition, 0.0);
  }

  public Rotation2d getGoalPosition() {
    return new Rotation2d(goal.position);
  }

  public Rotation2d getGoalVelocity() {
    return new Rotation2d(goal.velocity);
  }

  public boolean atGoal() {
    return MathUtil.isNear(goal.position, inputs.positionRad, atGoalToleranceRad);
  }

  private void resetController() {
    setGoal(0.0);
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }
}
