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
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOReal;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOSim;
import frc.robot.subsystems.superstructure.constants.CoralPivotConstants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.constants.ShooterConstants;
import frc.robot.subsystems.superstructure.constants.SuperStructureConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterModes;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Elevator elevator;
  private final Pivot coralPivot;
  private final Shooter shooter;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism("Goal", Color.kLightBlue, Color.kLightGreen, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism("Measured", Color.kDarkBlue, Color.kDarkGreen, 3.0);

  private SuperStructureModes currentMode = SuperStructureModes.TUCKED;

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

    shooter = new Shooter(name + "/Shooter", shooterIO, coralSensorIO);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setCurrentMode(SuperStructureModes.TUCKED);
      shooter.setShooterMode(ShooterModes.NONE);
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
        if (MathUtil.isNear(coralPivot.getPosition().getDegrees(), 180, 90)) {
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

    elevator.periodic();
    coralPivot.periodic();
    shooter.periodic(currentMode);

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
    return Commands.runOnce(() -> setCurrentMode(nextMode));
  }

  public Command setShooterModeCommand(ShooterModes nextShooterMode) {
    return Commands.runOnce(() -> shooter.setShooterMode(nextShooterMode));
  }

  public Trigger isAtMode() {
    return new Trigger(() -> isAtMode);
  }

  public Command intake() {
    return Commands.sequence(
        setShooterModeCommand(ShooterModes.INTAKE),
        // Commands.sequence(Commands.waitUntil(isCoralIntaked()), Commands.waitSeconds(0.1)),
        Commands.waitUntil(isCoralIntaked()),
        setShooterModeCommand(ShooterModes.NONE));
  }

  public Command score(SuperStructureModes mode) {
    return Commands.sequence(
            setModeAndWaitCommand(mode),
            shootCoral(),
            setModeAndWaitCommand(SuperStructureModes.TUCKED))
        // .onlyIf(isCoralIntaked())
        .finallyDo(this::resetModes);
  }

  public Command setModeAndWaitCommand(SuperStructureModes mode) {
    return Commands.sequence(setModeCommand(mode), Commands.waitUntil(isAtMode()));
  }

  public Command shootCoral() {
    return Commands.sequence(
        setShooterModeCommand(ShooterModes.SHOOT),
        Commands.sequence(Commands.waitUntil(isCoralIntaked().negate()), Commands.waitSeconds(1.0)),
        setShooterModeCommand(ShooterModes.NONE));
  }

  public void resetModes() {
    shooter.setShooterMode(ShooterModes.NONE);
    setCurrentMode(SuperStructureModes.TUCKED);
  }

  public Trigger isCoralIntaked() {
    return new Trigger(() -> shooter.isCoralDetected());
  }
}
