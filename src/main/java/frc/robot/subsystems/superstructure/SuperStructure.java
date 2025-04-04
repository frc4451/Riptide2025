package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;
import frc.robot.subsystems.rollers.follow.FollowRollersIOTalonFX;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOReal;
import frc.robot.subsystems.superstructure.can_range.CanRangeIOSim;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.constants.PivotConstants;
import frc.robot.subsystems.superstructure.constants.ShooterConstants;
import frc.robot.subsystems.superstructure.constants.SuperStructureConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterModes;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Elevator elevator;
  private final Pivot pivot;
  private final Shooter shooter;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism("Goal", Color.kLightBlue, Color.kLightGreen, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism("Measured", Color.kDarkBlue, Color.kDarkGreen, 3.0);

  private SuperStructureModes currentMode = SuperStructureModes.TUCKED;

  private boolean isAtMode = false;

  public SuperStructure() {
    FollowRollersIO elevatorIO;
    SingleRollerIO pivotIO;
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
                ElevatorConstants.isBrake,
                ElevatorConstants.foc,
                ElevatorConstants.gains,
                ElevatorConstants.mmConfig);

        pivotIO =
            new SingleRollerIOTalonFX(
                PivotConstants.canId,
                PivotConstants.reduction,
                PivotConstants.currentLimitAmps,
                PivotConstants.invert,
                PivotConstants.isBrakeMode,
                PivotConstants.foc,
                PivotConstants.gains,
                PivotConstants.mmConfig);

        shooterIO =
            new SingleRollerIOTalonFX(
                ShooterConstants.canId,
                ShooterConstants.reduction,
                ShooterConstants.currentLimitAmps,
                ShooterConstants.invert,
                ShooterConstants.isBrakeMode,
                ShooterConstants.foc,
                new Slot0Configs(),
                new MotionMagicConfigs());

        coralSensorIO = new CanRangeIOReal(ShooterConstants.coralSensorId, false);
        break;

      case SIM:
        elevatorIO =
            new FollowRollersIOSim(
                ElevatorConstants.leaderGearbox,
                ElevatorConstants.followerGearbox,
                ElevatorConstants.reduction,
                ElevatorConstants.moi,
                ElevatorConstants.invertFollower,
                ElevatorConstants.gains,
                ElevatorConstants.mmConfig,
                new EmptyFeedforwardController());

        pivotIO =
            new SingleRollerIOSim(
                PivotConstants.gearbox,
                PivotConstants.reduction,
                PivotConstants.moi,
                PivotConstants.gains,
                PivotConstants.mmConfig,
                new EmptyFeedforwardController());

        shooterIO =
            new SingleRollerIOSim(
                ShooterConstants.gearbox,
                ShooterConstants.reduction,
                ShooterConstants.moi,
                new Slot0Configs(),
                new MotionMagicConfigs(),
                new EmptyFeedforwardController());

        coralSensorIO = new CanRangeIOSim();
        break;

      case REPLAY:
      default:
        elevatorIO = new FollowRollersIO() {};
        pivotIO = new SingleRollerIO() {};
        shooterIO = new SingleRollerIO() {};
        coralSensorIO = new CanRangeIO() {};
        break;
    }

    elevator = new Elevator(name + "/Elevator", elevatorIO, ElevatorConstants.inchesPerRotation);
    elevator.setHeightInches(ElevatorConstants.startHeightInches);

    pivot = new Pivot(name + "/Coral/Pivot", pivotIO);

    shooter = new Shooter(name + "/Shooter", shooterIO, coralSensorIO);
  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    //   setCurrentMode(SuperStructureModes.TUCKED);
    //   shooter.setShooterMode(ShooterModes.NONE);
    // }

    boolean isElevatorAtMode = elevator.isNear(currentMode.elevatorHeightInches);

    boolean isPivotAtMode = pivot.isNear(currentMode.coralPos);

    isAtMode = isElevatorAtMode && isPivotAtMode;

    elevator.setGoalHeightInches(currentMode.elevatorHeightInches);
    pivot.setGoal(currentMode.coralPos);

    Logger.recordOutput(name + "/IsElevatorAtMode", isElevatorAtMode);
    Logger.recordOutput(name + "/IsPivotAtMode", isPivotAtMode);
    Logger.recordOutput(name + "/Mode", currentMode);

    elevator.periodic();
    pivot.periodic();
    shooter.periodic(currentMode);

    measuredMechanism.update(elevator.getHeightInches(), pivot.getPosition());
    goalMechanism.update(elevator.getGoalHeightInches(), pivot.getGoalPosition());
  }

  // public Command elevatorManualCommand(DoubleSupplier supplier) {
  //   return run(() -> elevator.runVolts(supplier.getAsDouble())).finallyDo(() -> elevator.stop());
  // }

  // public Command pivotManualCommand(DoubleSupplier supplier) {
  //   return run(() -> coralPivot.runVolts(supplier.getAsDouble()))
  //       .finallyDo(() -> coralPivot.stop());
  // }

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
            setShooterModeCommand(ShooterModes.INTAKE)
                .unless(isCoralIntaked().or(() -> Constants.currentMode == Constants.Mode.SIM)),
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
        Commands.sequence(
            Commands.waitUntil(isCoralIntaked().negate()), Commands.waitSeconds(0.125)),
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
    return currentMode == SuperStructureModes.L4Coral
        || currentMode == SuperStructureModes.TUCKED_L4;
  }

  public boolean shouldShootCoral() {
    return isAtMode
        && switch (currentMode) {
          case L2Coral, L3Coral, L4Coral -> true;
          default -> false;
        };
  }

  public double getReefOffset() {
    return isL4Coral()
        ? FieldConstants.eventConstants.l4ReefOffset
        : FieldConstants.eventConstants.l2ReefOffset;
  }

  public double getRumbleDistance() {
    return isL4Coral()
        ? FieldConstants.eventConstants.l4RumbleDistance
        : FieldConstants.eventConstants.l2RumbleDistance;
  }
}
