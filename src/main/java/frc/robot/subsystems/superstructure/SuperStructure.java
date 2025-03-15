package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
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
import frc.robot.subsystems.superstructure.elevatorV2.ElevatorV2;
import frc.robot.subsystems.superstructure.elevatorV2.ElevatorV2Constants;
import frc.robot.subsystems.superstructure.elevatorV2.ElevatorV2IO;
import frc.robot.subsystems.superstructure.elevatorV2.ElevatorV2IOProfiledSim;
import frc.robot.subsystems.superstructure.elevatorV2.ElevatorV2IOTalonFX;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterModes;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final ElevatorV2 elevator;
  private final Pivot coralPivot;
  private final Shooter shooter;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism("Goal", Color.kLightBlue, Color.kLightGreen, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism("Measured", Color.kDarkBlue, Color.kDarkGreen, 3.0);

  private SuperStructureModes currentMode = SuperStructureModes.TUCKED;

  private boolean isAtMode = false;

  public SuperStructure() {
    ElevatorV2IO elevatorIO;
    CanRangeIO heightSensorIO;
    SingleRollerIO coralPivotIO;
    SingleRollerIO shooterIO;
    CanRangeIO coralSensorIO;

    switch (Constants.currentMode) {
      case REAL:
        elevatorIO =
            new ElevatorV2IOTalonFX(
                ElevatorV2Constants.leaderCanId,
                ElevatorV2Constants.followerCanId,
                ElevatorV2Constants.reduction,
                ElevatorV2Constants.currentLimitAmps,
                ElevatorV2Constants.invert,
                ElevatorV2Constants.invertFollower,
                ElevatorV2Constants.isBrake,
                ElevatorV2Constants.foc,
                ElevatorV2Constants.inchesPerRad,
                ElevatorV2Constants.staticGains,
                ElevatorV2Constants.motionMagicProps);

        // heightSensorIO = new CanRangeIOReal(ElevatorConstants.heightSensorId, true);

        coralPivotIO =
            new SingleRollerIOTalonFX(
                CoralPivotConstants.canId,
                CoralPivotConstants.reduction,
                CoralPivotConstants.currentLimitAmps,
                CoralPivotConstants.invert,
                CoralPivotConstants.isBrakeMode);

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
            new ElevatorV2IOProfiledSim(
                ElevatorV2Constants.elevatorMotorSim,
                ElevatorV2Constants.reduction,
                ElevatorV2Constants.inchesPerRad,
                ElevatorV2Constants.carriageMassKg,
                Units.inchesToMeters(ElevatorV2Constants.elevatorConstraints.minHeightInches()),
                Units.inchesToMeters(ElevatorV2Constants.elevatorConstraints.maxHeightInches()),
                ElevatorV2Constants.simulateGravity,
                ElevatorV2Constants.startHeightInches,
                ElevatorV2Constants.staticGains,
                ElevatorV2Constants.trapezoidConstraints);

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
        elevatorIO = new ElevatorV2IO() {};
        heightSensorIO = new CanRangeIO() {};
        coralPivotIO = new SingleRollerIO() {};
        shooterIO = new SingleRollerIO() {};
        coralSensorIO = new CanRangeIO() {};
        break;
    }

    elevator =
        new ElevatorV2(
            name + "/ElevatorV2",
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

    boolean isElevatorAtMode = elevator.isNear(currentMode.elevatorHeightInches);

    boolean isPivotAtMode = coralPivot.isNear(currentMode.coralPos.getRadians());

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
            Commands.waitUntil(
                isCoralIntaked().or(() -> Constants.currentMode == Constants.Mode.SIM)))
        .finallyDo(() -> shooter.setShooterMode(ShooterModes.NONE));
  }

  public Command score(SuperStructureModes mode) {
    return Commands.sequence(
            setModeAndWaitCommand(mode),
            shootCoral(),
            setModeAndWaitCommand(SuperStructureModes.TUCKED))
        // .onlyIf(isCoralIntaked())
        .finallyDo(this::resetModes);
  }

  public Command scoreAlgae() {
    return Commands.sequence(
            setModeCommand(SuperStructureModes.Barge),
            Commands.waitUntil(() -> elevator.isNear(SuperStructureConstants.shootNetHeight)),
            setShooterModeCommand(ShooterModes.ALGAE_SHOOT),
            Commands.waitSeconds(0.4))
        .finallyDo(
            () -> {
              setCurrentMode(SuperStructureModes.TUCKED_L4);
              shooter.setShooterMode(ShooterModes.NONE);
            });
  }

  public Command setModeAndWaitCommand(SuperStructureModes mode) {
    return Commands.sequence(setModeCommand(mode), Commands.waitUntil(isAtMode()));
  }

  public Command shootCoral() {
    return Commands.sequence(
        setShooterModeCommand(ShooterModes.SHOOT),
        Commands.sequence(Commands.waitUntil(isCoralIntaked().negate()), Commands.waitSeconds(0.4)),
        setShooterModeCommand(ShooterModes.NONE));
  }

  public void resetModes() {
    shooter.setShooterMode(ShooterModes.NONE);
    setCurrentMode(SuperStructureModes.TUCKED);
  }

  public Trigger isCoralIntaked() {
    return new Trigger(() -> shooter.isCoralDetected());
  }

  public boolean isL4Coral() {
    return currentMode == SuperStructureModes.L4Coral;
  }
}
