package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;
import frc.robot.subsystems.rollers.follow.FollowRollersIOTalonFX;
import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.can_range.CanRange;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOReal;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOSim;
import frc.robot.subsystems.superstructure.constants.AlgaePivotConstants;
import frc.robot.subsystems.superstructure.constants.CoralPivotConstants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.constants.ShooterConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.ExitInstructions;
import frc.robot.subsystems.superstructure.modes.IntoInstructions;
import frc.robot.subsystems.superstructure.modes.ShooterModes;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Elevator elevator;
  private final Pivot coralPivot;
  private final SingleRoller shooter;
  private final Pivot algaePivot;
  private final CanRange coralSensor;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism(
          "Goal", Color.kLightBlue, Color.kLightGreen, Color.kTurquoise, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism(
          "Measured", Color.kDarkBlue, Color.kDarkGreen, Color.kDarkTurquoise, 3.0);

  private SuperStructureModes currentMode = SuperStructureModes.TUCKED;
  private ShooterModes currentShooterMode = ShooterModes.NONE;

  private IntoInstructions intoInstructions = IntoInstructions.NONE;
  private ExitInstructions exitInstructions = ExitInstructions.NONE;

  private boolean isAtMode = false;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    CanRangeIO heightSensorIO;
    SingleRollerIO coralPivotIO;
    SingleRollerIO shooterIO;
    SingleRollerIO algaePivotIO;
    CanRangeIO coralSensorIO;

    switch (Constants.currentMode) {
      case REAL:
        elevatorIO =
            new FollowRollersIOTalonFX(
                ElevatorConstants.leaderCanId,
                ElevatorConstants.followerCanId,
                ElevatorConstants.reduction,
                ElevatorConstants.currentLimitAmps,
                ElevatorConstants.invertFollower);

        heightSensorIO = new CanRangeIOReal(ElevatorConstants.heightSensorId, true);

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

        coralSensorIO = new CanRangeIOReal(ShooterConstants.coralSensorId, false);
        break;

      case SIM:
        elevatorIO =
            new FollowRollersIOSim(
                ElevatorConstants.leaderGearbox,
                ElevatorConstants.followerGearbox,
                ElevatorConstants.reduction,
                ElevatorConstants.moi,
                ElevatorConstants.invertFollower);

        heightSensorIO = new CanRangeIOSim();

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

        coralSensorIO = new CanRangeIOSim();
        break;

      case REPLAY:
      default:
        elevatorIO = new FollowRollersIO() {};
        heightSensorIO = new CanRangeIO() {};
        coralPivotIO = new SingleRollerIO() {};
        shooterIO = new SingleRollerIO() {};
        algaePivotIO = new SingleRollerIO() {};
        coralSensorIO = new CanRangeIO() {};
        break;
    }

    elevator =
        new Elevator(
            name + "/Elevator",
            elevatorIO,
            heightSensorIO,
            ElevatorConstants.trapezoidConstraints,
            ElevatorConstants.inchesPerRad,
            ElevatorConstants.elevatorConstraints);

    coralPivot =
        new Pivot(
            name + "/Coral/Pivot",
            coralPivotIO,
            CoralPivotConstants.trapezoidConstraints,
            CoralPivotConstants.pivotConstraints);

    shooter = new SingleRoller(name + "/Shooter", shooterIO);

    algaePivot =
        new Pivot(
            name + "/Algae/Pivot",
            algaePivotIO,
            AlgaePivotConstants.trapezoidConstraints,
            AlgaePivotConstants.pivotConstraints);

    coralSensor = new CanRange(name + "/CoralSensor", coralSensorIO);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setCurrentMode(SuperStructureModes.TUCKED);
      setCurrentShooterMode(ShooterModes.NONE);
    }

    if (currentShooterMode.useCanRange && coralSensor.isNear()) {
      shooter.stop();
    } else {
      shooter.runVolts(currentShooterMode.voltage);
    }

    if (intoInstructions == IntoInstructions.PIVOTS_BEFORE_ELEVATOR
        && coralPivot.atGoal()
        && algaePivot.atGoal()) {
      elevator.setGoalHeightInches(currentMode.elevatorHeightInches);
    }

    if (exitInstructions == ExitInstructions.ELEVATOR_BEFORE_PIVOTS
        && elevator.underL4Threshold()) {
      coralPivot.setGoal(currentMode.coralPos);
      algaePivot.setGoal(currentMode.algaePos);
    }

    isAtMode =
        MathUtil.isNear(
                elevator.getHeightInches(),
                currentMode.elevatorHeightInches,
                Elevator.atGoalToleranceInches)
            && MathUtil.isNear(
                coralPivot.getPosition().getRadians(),
                currentMode.coralPos.getRadians(),
                Pivot.atGoalToleranceRad)
            && MathUtil.isNear(
                algaePivot.getPosition().getRadians(),
                currentMode.algaePos.getRadians(),
                Pivot.atGoalToleranceRad);

    Logger.recordOutput(name + "/IsAtMode", isAtMode);
    Logger.recordOutput(name + "/Mode", currentMode);
    Logger.recordOutput(name + "/ShooterMode", currentShooterMode);

    elevator.periodic();
    coralPivot.periodic();
    coralSensor.periodic();
    shooter.periodic();
    algaePivot.periodic();

    measuredMechanism.update(
        elevator.getHeightInches(), coralPivot.getPosition(), algaePivot.getPosition());
    goalMechanism.update(
        elevator.getGoalHeightInches(), coralPivot.getGoalPosition(), algaePivot.getGoalPosition());
  }

  private void setCurrentMode(SuperStructureModes nextMode) {
    if (currentMode != nextMode) {
      intoInstructions = nextMode.intoInstructions;
      exitInstructions = currentMode.exitInstructions;

      if (intoInstructions == IntoInstructions.PIVOTS_BEFORE_ELEVATOR
          || exitInstructions == ExitInstructions.NONE) {
        coralPivot.setGoal(nextMode.coralPos);
        algaePivot.setGoal(nextMode.algaePos);
      }

      if (exitInstructions == ExitInstructions.ELEVATOR_BEFORE_PIVOTS
          || intoInstructions == IntoInstructions.NONE) {
        elevator.setGoalHeightInches(nextMode.elevatorHeightInches);
      }

      currentMode = nextMode;
    }
  }

  public Command setModeCommand(SuperStructureModes nextMode) {
    return runOnce(() -> setCurrentMode(nextMode));
  }

  private void setCurrentShooterMode(ShooterModes nextShooterMode) {
    if (currentShooterMode != nextShooterMode) {
      currentShooterMode = nextShooterMode;
    }
  }

  public Command setShooterModeCommand(ShooterModes nextShooterMode) {
    return runOnce(() -> setCurrentShooterMode(nextShooterMode));
  }

  public Trigger isAtMode() {
    return new Trigger(() -> isAtMode);
  }

  public Trigger isCoralIntaked() {
    return new Trigger(coralSensor::isNear);
  }
}
