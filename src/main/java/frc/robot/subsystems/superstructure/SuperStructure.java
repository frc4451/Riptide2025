package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;
import frc.robot.subsystems.rollers.follow.FollowRollersIOTalonFX;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.constants.CoralPivotConstants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
  private final Elevator elevator;
  private final Pivot coralPivot;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    SingleRollerIO coralPivotIO;
    SingleRollerIO coralShooterIO;

    switch (Constants.currentMode) {
      case REAL:
        elevatorIO =
            new FollowRollersIOTalonFX(
                ElevatorConstants.leaderCanId,
                ElevatorConstants.followerCanId,
                ElevatorConstants.reduction,
                ElevatorConstants.currentLimitAmps,
                ElevatorConstants.invertFollower);

        coralPivotIO =
            new SingleRollerIOTalonFX(
                CoralPivotConstants.canId,
                CoralPivotConstants.reduction,
                CoralPivotConstants.currentLimitAmps,
                CoralPivotConstants.invert);
        break;

      case REPLAY:
        elevatorIO = new FollowRollersIO() {};
        coralPivotIO = new SingleRollerIO() {};
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
        coralPivotIO =
            new SingleRollerIOSim(
                CoralPivotConstants.gearbox,
                CoralPivotConstants.reduction,
                CoralPivotConstants.moi);
        break;
    }

    elevator =
        new Elevator(
            "Superstructure/Elevator",
            elevatorIO,
            ElevatorConstants.trapezoidConstraints,
            ElevatorConstants.inchesPerRad,
            ElevatorConstants.elevatorConstraints);

    coralPivot =
        new Pivot(
            "Superstructure/Coral/Pivot",
            coralPivotIO,
            CoralPivotConstants.trapezoidConstraints,
            CoralPivotConstants.pivotConstraints);
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
