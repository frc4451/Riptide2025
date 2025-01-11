package frc.robot.subsystems.rollers.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SingleRollerSubsystem {
  private static final record LoggedTrapezoidState(
      double position, double positionInches, double velocity, double velocityInches) {}

  private final TrapezoidProfile trapezoidProfile;

  @AutoLogOutput private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  @AutoLogOutput private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final double inchesPerRad;

  public ElevatorSubsystem(
      String name, ElevatorIO io, TrapezoidProfile.Constraints constraints, double inchesPerRad) {
    super(name, io);
    trapezoidProfile = new TrapezoidProfile(constraints);
    this.inchesPerRad = inchesPerRad;
  }

  public ElevatorSubsystem(ElevatorIO io) {
    this("Elevator", io, ElevatorConstants.constraints, ElevatorConstants.inchesPerRad);
  }

  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled()) {
      resetController();
    }

    Logger.recordOutput(
        getName() + "/Profile/Setpoint",
        new LoggedTrapezoidState(
            setpoint.position,
            setpoint.position * inchesPerRad,
            setpoint.velocity,
            setpoint.velocity * inchesPerRad));
    Logger.recordOutput(
        getName() + "/Profile/Goal",
        new LoggedTrapezoidState(
            goal.position,
            goal.position * inchesPerRad,
            goal.velocity,
            goal.velocity * inchesPerRad));
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
            ElevatorConstants.minHeightInches / inchesPerRad,
            ElevatorConstants.maxHeightInches / inchesPerRad);
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
