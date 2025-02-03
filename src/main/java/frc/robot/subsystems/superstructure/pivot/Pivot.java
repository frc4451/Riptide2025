package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SingleRoller {
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
  }

  public Pivot(SingleRollerIO io) {
    this("Pivot", io, PivotConstants.trapezoidConstraints, PivotConstants.pivotConstraints);
  }

  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled()) {
      resetController();
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
  }

  @AutoLogOutput()
  public double getPositionDegrees() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  @AutoLogOutput()
  public double getVelocityDegreesPerSec() {
    return Units.radiansToDegrees(inputs.velocityRadPerSec);
  }

  public void runTrapezoidProfile() {
    setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    io.runPosition(setpoint.position);
  }

  public void setGoalRad(double positionInches) {
    double clampedPosition =
        MathUtil.clamp(
            positionInches, pivotConstraints.minRadians(), pivotConstraints.maxRadians());
    goal = new TrapezoidProfile.State(clampedPosition, 0.0);
  }

  private void resetController() {
    setGoalRad(0.0);
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }
}
