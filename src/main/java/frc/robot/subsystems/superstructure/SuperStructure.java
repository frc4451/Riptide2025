package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.superstructure.constants.CoralPivotConstants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.constants.ShooterConstants;
import frc.robot.subsystems.superstructure.constants.SuperStructureConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.ShooterModes;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Elevator elevator;
  private final Pivot coralPivot;
  private final SingleRoller shooter;
  private final CanRange coralSensor;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism("Goal", Color.kLightBlue, Color.kLightGreen, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism("Measured", Color.kDarkBlue, Color.kDarkGreen, 3.0);

  private SuperStructureModes currentMode = SuperStructureModes.TUCKED;
  private ShooterModes currentShooterMode = ShooterModes.NONE;

  private boolean isAtMode = false;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    CanRangeIO heightSensorIO;
    SingleRollerIO coralPivotIO;
    SingleRollerIO shooterIO;
    CanRangeIO coralSensorIO;

    switch (Constants.currentMode) {
      case REAL:
        elevatorIO =
            new FollowRollersIOTalonFX(
                ElevatorConstants.leaderCanId,
                ElevatorConstants.followerCanId,
                ElevatorConstants.reduction,
                ElevatorConstants.currentLimitAmps,
                ElevatorConstants.invert,
                ElevatorConstants.invertFollower,
                true);

        // heightSensorIO = new CanRangeIOReal(ElevatorConstants.heightSensorId, true);

        coralPivotIO =
            new SingleRollerIOTalonFX(
                CoralPivotConstants.canId,
                CoralPivotConstants.reduction,
                CoralPivotConstants.currentLimitAmps,
                CoralPivotConstants.invert,
                true);

        shooterIO =
            new SingleRollerIOTalonFX(
                ShooterConstants.canId,
                ShooterConstants.reduction,
                ShooterConstants.currentLimitAmps,
                ShooterConstants.invert,
                true);

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

        coralSensorIO = new CanRangeIOSim();
        break;

      case REPLAY:
      default:
        elevatorIO = new FollowRollersIO() {};
        heightSensorIO = new CanRangeIO() {};
        coralPivotIO = new SingleRollerIO() {};
        shooterIO = new SingleRollerIO() {};
        coralSensorIO = new CanRangeIO() {};
        break;
    }

    elevator =
        new Elevator(
            name + "/Elevator",
            elevatorIO,
            new CanRangeIO() {},
            ElevatorConstants.trapezoidConstraints,
            ElevatorConstants.inchesPerRad,
            ElevatorConstants.elevatorConstraints,
            ElevatorConstants.feedforward,
            ElevatorConstants.kP,
            ElevatorConstants.kD);
    elevator.setHeightInches(ElevatorConstants.startHeightInches);

    coralPivot =
        new Pivot(
            name + "/Coral/Pivot",
            coralPivotIO,
            CoralPivotConstants.trapezoidConstraints,
            CoralPivotConstants.pivotConstraints,
            CoralPivotConstants.feedforward,
            CoralPivotConstants.kP,
            CoralPivotConstants.kD);

    shooter = new SingleRoller(name + "/Shooter", shooterIO);

    coralSensor = new CanRange(name + "/CoralSensor", coralSensorIO);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setCurrentMode(SuperStructureModes.TUCKED);
      setCurrentShooterMode(ShooterModes.NONE);
    }

    if (currentShooterMode.useCanRange && coralSensor.isDetected()) {
      shooter.stop();
    } else {
      // If at L4 invert direction
      shooter.runVolts(
          (currentMode == SuperStructureModes.L4Coral ? -1 : 1) * currentShooterMode.voltage);
    }

    boolean isElevatorAtMode =
        MathUtil.isNear(
            elevator.getHeightInches(),
            currentMode.elevatorHeightInches,
            Elevator.atGoalToleranceInches);

    boolean isPivotAtMode =
        MathUtil.isNear(
            coralPivot.getPosition().getRadians(),
            currentMode.coralPos.getRadians(),
            Pivot.atGoalToleranceRad);

    isAtMode = isElevatorAtMode && isPivotAtMode;

    // Make sure Pivot is tucked while moving so that
    if (currentMode.elevatorHeightInches > SuperStructureConstants.pivotTuckThresholdInches
        || elevator.getHeightInches() > SuperStructureConstants.pivotTuckThresholdInches) {
      if (!isElevatorAtMode) {
        coralPivot.setGoal(Rotation2d.kPi);
        if (coralPivot.atGoal()) {
          elevator.setGoalHeightInches(currentMode.elevatorHeightInches);
        }
      } else {
        coralPivot.setGoal(currentMode.coralPos);
      }
    } else {
      elevator.setGoalHeightInches(currentMode.elevatorHeightInches);
      coralPivot.setGoal(currentMode.coralPos);
    }

    Logger.recordOutput(name + "/IsElevatorAtMode", isElevatorAtMode);
    Logger.recordOutput(name + "/IsPivotAtMode", isPivotAtMode);
    Logger.recordOutput(name + "/Mode", currentMode);
    Logger.recordOutput(name + "/ShooterMode", currentShooterMode);

    elevator.periodic();
    coralPivot.periodic();
    coralSensor.periodic();
    shooter.periodic();

    measuredMechanism.update(elevator.getHeightInches(), coralPivot.getPosition());
    goalMechanism.update(elevator.getGoalHeightInches(), coralPivot.getGoalPosition());
  }

  public Command elevatorManualCommand(DoubleSupplier supplier) {
    return run(() -> elevator.runVolts(supplier.getAsDouble())).finallyDo(() -> elevator.stop());
  }

  public Command pivotManualCommand(DoubleSupplier supplier) {
    return run(() -> coralPivot.runVolts(supplier.getAsDouble()))
        .finallyDo(() -> coralPivot.stop());
  }

  private void setCurrentMode(SuperStructureModes nextMode) {
    if (currentMode != nextMode) {
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
    return new Trigger(coralSensor::isDetected);
  }

  public Command intake() {
    return Commands.none();
  }

  public Command score(SuperStructureModes mode) {
    return Commands.sequence(
            setModeAndWaitCommand(mode),
            setShooterModeAndWaitCommand(ShooterModes.SHOOT),
            setModeAndWaitCommand(SuperStructureModes.TUCKED))
        .onlyIf(isCoralIntaked())
        .finallyDo(this::resetModes);
  }

  public Command setModeAndWaitCommand(SuperStructureModes mode) {
    return Commands.sequence(setModeCommand(mode), Commands.waitUntil(isAtMode()));
  }

  public Command setShooterModeAndWaitCommand(ShooterModes mode) {
    return Commands.sequence(
        setShooterModeCommand(mode),
        Commands.sequence(Commands.waitUntil(isCoralIntaked().negate()), Commands.waitSeconds(0.1)),
        setShooterModeCommand(ShooterModes.NONE));
  }

  public void resetModes() {
    setCurrentShooterMode(ShooterModes.NONE);
    setCurrentMode(SuperStructureModes.TUCKED);
  }
}
