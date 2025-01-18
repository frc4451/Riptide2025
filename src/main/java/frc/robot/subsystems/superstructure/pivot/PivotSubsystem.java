package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SingleRollerSubsystem {
  private final TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final PivotConstraints pivotConstraints;

  public PivotSubsystem(
      String name,
      SingleRollerIO io,
      TrapezoidProfile.Constraints trapezoidConstraints,
      PivotConstraints pivotConstraints) {
    super(name, io);
    trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.pivotConstraints = pivotConstraints;
  }

  public PivotSubsystem(SingleRollerIO io) {
    this("Pivot", io, PivotConstants.trapezoidConstraints, PivotConstants.pivotConstraints);
  }

  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled()) {
      resetController();
    }

    Logger.recordOutput(
        getName() + "/Profile/Setpoint/Rad",
        new LoggedTrapezoidState(setpoint.position, setpoint.velocity));
    Logger.recordOutput(
        getName() + "/Profile/Setpoint/Deg",
        new LoggedTrapezoidState(
            Units.radiansToDegrees(setpoint.position), Units.radiansToDegrees(setpoint.velocity)));

    Logger.recordOutput(
        getName() + "/Profile/Goal/Rad", new LoggedTrapezoidState(goal.position, goal.velocity));
    Logger.recordOutput(
        getName() + "/Profile/Goal/Deg",
        new LoggedTrapezoidState(
            Units.radiansToDegrees(goal.position), Units.radiansToDegrees(goal.velocity)));
  }

  @AutoLogOutput()
  private double getPositionDegrees() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  @AutoLogOutput()
  private double getVelocityDegreesPerSec() {
    return Units.radiansToDegrees(inputs.velocityRadPerSec);
  }

  private void runTrapezoidProfile() {
    setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    io.runPosition(setpoint.position);
  }

  public Command runTrapezoidProfileCommand() {
    return run(this::runTrapezoidProfile);
  }

  public void setGoalRad(double positionInches) {
    double clampedPosition =
        MathUtil.clamp(
            positionInches, pivotConstraints.minRadians(), pivotConstraints.maxRadians());
    goal = new TrapezoidProfile.State(clampedPosition, 0.0);
  }

  private void resetController() {
    setGoalRadCommand(0.0);
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  public Command setGoalRadCommand(double positionRad) {
    return runOnce(() -> setGoalRad(positionRad));
  }
}
