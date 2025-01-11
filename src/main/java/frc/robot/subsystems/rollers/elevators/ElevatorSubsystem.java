package frc.robot.subsystems.rollers.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.single.SingleRollerSubsystem;

public class ElevatorSubsystem extends SingleRollerSubsystem {
  private final TrapezoidProfile trapezoidProfile;
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  public ElevatorSubsystem(String name, ElevatorIO io, Constraints constraints) {
    super(name, io);
    trapezoidProfile = new TrapezoidProfile(constraints);
  }

  public ElevatorSubsystem(ElevatorIO io) {
    this("Elevator", io, ElevatorConstants.constraints);
  }

  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled()) {
      setGoalInches(0.0);
      io.stop();
    }
  }

  private void runTrapezoidProfile() {
    setpoint = trapezoidProfile.calculate(Constants.loopPeriodSecs, setpoint, goal);
  }

  public Command runTrapezoidProfileCommand() {
    return run(this::runTrapezoidProfile);
  }

  public void setSetpointInches(double position) {
    double clampedPosition =
        MathUtil.clamp(position, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    setpoint = new TrapezoidProfile.State(clampedPosition, 0.0);
  }

  public void setGoalInches(double position) {
    double clampedPosition =
        MathUtil.clamp(position, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    goal = new TrapezoidProfile.State(clampedPosition, 0.0);
  }
}
