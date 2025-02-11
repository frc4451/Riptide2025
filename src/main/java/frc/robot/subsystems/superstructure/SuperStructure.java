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
import frc.robot.subsystems.superstructure.can_range.CanRange;
import frc.robot.subsystems.superstructure.can_range.CanRangeConstants;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOReal;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOSim;
import frc.robot.subsystems.superstructure.constants.AlgaePivotConstants;
import frc.robot.subsystems.superstructure.constants.CoralPivotConstants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.constants.ShooterConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.ShooterModes;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.pivot.Pivot;

public class SuperStructure extends SubsystemBase {
  private final Elevator elevator;
  private final Pivot coralPivot;
  private final SingleRoller shooter;
  private final Pivot algaePivot;
  private final CanRange canRange;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism(
          "Goal", Color.kLightBlue, Color.kLightGreen, Color.kTurquoise, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism(
          "Measured", Color.kDarkBlue, Color.kDarkGreen, Color.kDarkTurquoise, 3.0);

  private SuperStructureModes mode = SuperStructureModes.TUCKED;
  private ShooterModes shooterMode = ShooterModes.NONE;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    SingleRollerIO coralPivotIO;
    SingleRollerIO shooterIO;
    SingleRollerIO algaePivotIO;
    CanRangeIO canRangeIO;

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

        shooterIO =
            new SingleRollerIOTalonFX(
                ShooterConstants.canId,
                ShooterConstants.reduction,
                ShooterConstants.currentLimitAmps,
                ShooterConstants.invert);

        algaePivotIO =
            new SingleRollerIOTalonFX(
                AlgaePivotConstants.canId,
                AlgaePivotConstants.reduction,
                AlgaePivotConstants.currentLimitAmps,
                AlgaePivotConstants.invert);

        canRangeIO = new CanRangeIOReal(CanRangeConstants.canID);
        break;

      case REPLAY:
        elevatorIO = new FollowRollersIO() {};
        coralPivotIO = new SingleRollerIO() {};
        shooterIO = new SingleRollerIO() {};
        algaePivotIO = new SingleRollerIO() {};
        canRangeIO = new CanRangeIO() {};
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

        shooterIO =
            new SingleRollerIOSim(
                ShooterConstants.gearbox, ShooterConstants.reduction, ShooterConstants.moi);

        algaePivotIO =
            new SingleRollerIOSim(
                AlgaePivotConstants.gearbox,
                AlgaePivotConstants.reduction,
                AlgaePivotConstants.moi);

        canRangeIO = new CanRangeIOSim();
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

    shooter = new SingleRoller("Superstructure/Shooter", shooterIO);

    algaePivot =
        new Pivot(
            "Superstructure/Algae/Pivot",
            algaePivotIO,
            AlgaePivotConstants.trapezoidConstraints,
            AlgaePivotConstants.pivotConstraints);

    canRange = new CanRange("Superstructure/Shooter/CanRange", canRangeIO);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setMode(SuperStructureModes.TUCKED);
      setShooterMode(ShooterModes.NONE);
    }

    if (shooterMode.useCanRange && canRange.withinThreshold()) {
      shooter.stop();
    } else {
      shooter.runVolts(shooterMode.voltage);
    }

    elevator.periodic();
    coralPivot.periodic();
    shooter.periodic();
    algaePivot.periodic();

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

  private void setShooterMode(ShooterModes shooterMode) {
    if (this.shooterMode != shooterMode) {
      this.shooterMode = shooterMode;
    }
  }

  public Command setShooterModeCommand(ShooterModes shooterMode) {
    return runOnce(() -> setShooterMode(shooterMode));
  }
}
