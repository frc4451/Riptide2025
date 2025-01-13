package frc.robot.subsystems.superstructures.corel.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.elevators.ElevatorIO;
import frc.robot.subsystems.rollers.elevators.ElevatorSubsystem;

public class CorelElevator extends ElevatorSubsystem {
  public CorelElevator(ElevatorIO io) {
    super(
        "Corel/Elevator",
        io,
        CorelElevatorConstants.trapezoidConstraints,
        CorelElevatorConstants.inchesPerRad,
        CorelElevatorConstants.elevatorConstraints);

    this.setDefaultCommand(this.runTrapezoidProfileCommand());
  }

  public Command resetElevator() {
    return this.setGoalInchesCommand(CorelElevatorSetpoint.INITIAL.setpointInches);
  }
}
