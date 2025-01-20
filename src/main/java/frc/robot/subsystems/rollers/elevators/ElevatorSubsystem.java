package frc.robot.subsystems.rollers.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SingleRollerSubsystem {
  private final TrapezoidProfile trapezoidProfile;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  public ElevatorSubsystem(
      String name,
      ElevatorIO io,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints) {
    super(name, io);
    trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    this.inchesPerRad = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
  }

  public ElevatorSubsystem(ElevatorIO io) {
    this(
        "Elevator",
        io,
        ElevatorConstants.trapezoidConstraints,
        ElevatorConstants.inchesPerRad,
        ElevatorConstants.elevatorConstraints);
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
        getName() + "/Profile/Setpoint/In",
        new LoggedTrapezoidState(
            Units.radiansToDegrees(setpoint.position), Units.radiansToDegrees(setpoint.velocity)));

    Logger.recordOutput(
        getName() + "/Profile/Goal/Rad", new LoggedTrapezoidState(goal.position, goal.velocity));
    Logger.recordOutput(
        getName() + "/Profile/Goal/In",
        new LoggedTrapezoidState(
            Units.radiansToDegrees(goal.position), Units.radiansToDegrees(goal.velocity)));
  }

  @AutoLogOutput()
  private double getHeightInches() {
    return inputs.positionRad * inchesPerRad;
  }

  private void runTrapezoidProfile() {
    setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
    io.runPosition(setpoint.position);
  }

  public Command runTrapezoidProfileCommand() {
    return run(this::runTrapezoidProfile);
  }

  public void setGoalInches(double positionInches) {
    double clampedPosition =
        MathUtil.clamp(
            positionInches / inchesPerRad,
            elevatorConstraints.minHeightInches() / inchesPerRad,
            elevatorConstraints.maxHeightInches() / inchesPerRad);
    goal = new TrapezoidProfile.State(clampedPosition, 0.0);
  }

  private void resetController() {
    setGoalInchesCommand(0.0);
    setpoint = new TrapezoidProfile.State(0.0, 0.0);
  }

  public Command setGoalInchesCommand(double positionInches) {
    return runOnce(() -> setGoalInches(positionInches));
  }
}
