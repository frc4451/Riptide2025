package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.ElevatorSingle;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ElevatorSingle roller;

  private boolean manual = false;

  private ClimberModes mode = ClimberModes.TUCK;

  public Climber() {
    SingleRollerIO io;
    switch (Constants.currentMode) {
      case REAL:
        io =
            new SingleRollerIOTalonFX(
                ClimberConstants.canId,
                ClimberConstants.reduction,
                ClimberConstants.currentLimitAmps,
                ClimberConstants.invert,
                ClimberConstants.isBrakeMode,
                ClimberConstants.foc,
                ClimberConstants.gains,
                ClimberConstants.mmConfig);
        break;
      case SIM:
        io =
            new SingleRollerIOSim(
                ClimberConstants.gearbox,
                ClimberConstants.reduction,
                ClimberConstants.moi,
                ClimberConstants.gains,
                ClimberConstants.mmConfig,
                new EmptyFeedforwardController());
        break;
      case REPLAY:
      default:
        io = new SingleRollerIO() {};
        break;
    }

    roller = new ElevatorSingle("Climber/Winch", io, ClimberConstants.circumferenceOfSpool);
    roller.setHeightInches(0);
  }

  @Override
  public void periodic() {
    roller.periodic();

    if (!manual) {
      roller.setGoalHeightInches(mode.positionInches);
    }

    Logger.recordOutput(getName() + "/Mode", mode);
    Logger.recordOutput(getName() + "/Manual", manual);
  }

  public Command manualCommand(DoubleSupplier volts) {
    return Commands.sequence(
        runOnce(() -> manual = true),
        run(
            () -> {
              if (Math.signum(volts.getAsDouble()) == 1
                  && roller.getHeightInches() > ClimberConstants.maxPositionInches) {
                roller.stop();
              } else {
                roller.runVolts(volts.getAsDouble());
              }
            }),
        runOnce(
            () -> {
              roller.setGoalHeightInches(roller.getHeightInches());
              manual = false;
            }));
  }

  public void setMode(ClimberModes mode) {
    this.mode = mode;
    manual = false;
  }

  public Command setModeCommand(ClimberModes mode) {
    return runOnce(() -> setMode(mode));
  }
}
