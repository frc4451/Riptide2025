package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;
import frc.robot.subsystems.rollers.follow.FollowRollersIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
  private final Elevator elevator;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    switch (Constants.currentMode) {
      case REAL:
        elevatorIO =
            new FollowRollersIOTalonFX(
                ElevatorConstants.leaderCanId,
                ElevatorConstants.followerCanId,
                ElevatorConstants.reduction,
                ElevatorConstants.currentLimitAmps,
                ElevatorConstants.invertFollower);
        break;

      case REPLAY:
        elevatorIO = new FollowRollersIO() {};
        break;

      case SIM:
      default:
        elevatorIO =
            new FollowRollersIOSim(
                ElevatorConstants.leaderGearbox,
                ElevatorConstants.followerGearbox,
                ElevatorConstants.reduction,
                ElevatorConstants.moi,
                ElevatorConstants.invertFollower);
        break;
    }

    elevator =
        new Elevator(
            "Superstructure/Elevator",
            elevatorIO,
            ElevatorConstants.trapezoidConstraints,
            ElevatorConstants.inchesPerRad,
            ElevatorConstants.elevatorConstraints);
  }

  @Override
  public void periodic() {
    elevator.periodic();
  }

  public Command elevatorManualCommand(DoubleSupplier input) {
    return run(() -> elevator.runVolts(input.getAsDouble() * 12.0));
  }

  public Command elevatorSetSetpoint(double heightInches) {
    return run(() -> elevator.setGoalInches(heightInches));
  }
}
