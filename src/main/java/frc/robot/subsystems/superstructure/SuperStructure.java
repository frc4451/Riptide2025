package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;
import frc.robot.subsystems.rollers.follow.FollowRollersIOTalonFX;
import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.constants.AlgaePivotConstants;
import frc.robot.subsystems.superstructure.constants.AlgaeShooterConstants;
import frc.robot.subsystems.superstructure.constants.CoralPivotConstants;
import frc.robot.subsystems.superstructure.constants.CoralShooterConstants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.AlgaeShooterModes;
import frc.robot.subsystems.superstructure.modes.CoralShooterModes;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.pivot.Pivot;

public class SuperStructure extends SubsystemBase {
  private final Elevator elevator;
  private final Pivot coralPivot;
  private final SingleRoller coralShooter;
  private final Pivot algaePivot;
  private final SingleRoller algaeShooter;

  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism("Measured", Color.kGreen, Color.kGreen, Color.kGreen);
  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism("Goal", Color.kBlueViolet, Color.kBlueViolet, Color.kBlueViolet);

  private SuperStructureModes mode = SuperStructureModes.TUCKED;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    SingleRollerIO coralPivotIO;
    SingleRollerIO coralShooterIO;
    SingleRollerIO algaePivotIO;
    SingleRollerIO algaeShooterIO;

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

        coralShooterIO =
            new SingleRollerIOTalonFX(
                CoralShooterConstants.canId,
                CoralShooterConstants.reduction,
                CoralShooterConstants.currentLimitAmps,
                CoralShooterConstants.invert);

        algaePivotIO =
            new SingleRollerIOTalonFX(
                AlgaePivotConstants.canId,
                AlgaePivotConstants.reduction,
                AlgaePivotConstants.currentLimitAmps,
                AlgaePivotConstants.invert);

        algaeShooterIO =
            new SingleRollerIOTalonFX(
                AlgaeShooterConstants.canId,
                AlgaeShooterConstants.reduction,
                AlgaeShooterConstants.currentLimitAmps,
                AlgaeShooterConstants.invert);

        break;

      case REPLAY:
        elevatorIO = new FollowRollersIO() {};
        coralPivotIO = new SingleRollerIO() {};
        coralShooterIO = new SingleRollerIO() {};
        algaePivotIO = new SingleRollerIO() {};
        algaeShooterIO = new SingleRollerIO() {};
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

        coralShooterIO =
            new SingleRollerIOSim(
                CoralShooterConstants.gearbox,
                CoralShooterConstants.reduction,
                CoralShooterConstants.moi);

        algaePivotIO =
            new SingleRollerIOSim(
                AlgaePivotConstants.gearbox,
                AlgaePivotConstants.reduction,
                AlgaePivotConstants.moi);

        algaeShooterIO =
            new SingleRollerIOSim(
                AlgaePivotConstants.gearbox,
                AlgaePivotConstants.reduction,
                AlgaePivotConstants.moi);
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

    coralShooter = new SingleRoller("Superstructure/Coral/Shooter", coralShooterIO);

    algaePivot =
        new Pivot(
            "Superstructure/Algae/Pivot",
            algaePivotIO,
            AlgaePivotConstants.trapezoidConstraints,
            AlgaePivotConstants.pivotConstraints);

    algaeShooter = new SingleRoller("Superstructure/Algae/Shooter", algaeShooterIO);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setMode(SuperStructureModes.TUCKED);
    }

    elevator.periodic();
    coralPivot.periodic();
    coralShooter.periodic();
    algaePivot.periodic();
    algaeShooter.periodic();

    measuredMechanism.update(
        elevator.getHeightInches(), coralPivot.getPosition(), algaePivot.getPosition());
    goalMechanism.update(
        elevator.getGoalHeightInches(), coralPivot.getGoalPosition(), algaePivot.getGoalPosition());
  }

  private void setMode(SuperStructureModes mode) {
    if (this.mode != mode) {
      this.mode = mode;
      elevator.setGoalHeightInches(mode.elevatorHeightInches);
      coralPivot.setGoal(mode.coralPos.getRadians());
      algaePivot.setGoal(mode.algaePos.getRadians());
    }
  }

  public Command setModeCommand(SuperStructureModes mode) {
    return runOnce(() -> setMode(mode));
  }

  private void runAlgaeShooter(AlgaeShooterModes mode) {
    algaeShooter.runVolts(mode.voltage);
  }

  private void runCoralShooter(CoralShooterModes mode) {
    coralShooter.runVolts(mode.voltage);
  }
}
