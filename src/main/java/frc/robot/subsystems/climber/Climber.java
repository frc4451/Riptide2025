package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.mechanism.Mechanism;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberModes mode = ClimberModes.TUCK;
  private Pivot pivot;

  private Mechanism goalMechanism = new Mechanism(getName() + "/Goal", Color.kLightGreen, 10.0);
  private Mechanism measuredMechanism =
      new Mechanism(getName() + "/Measured", Color.kDarkGreen, 3.0);

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

    pivot = new Pivot("Climber", io);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/Mode", mode);

    pivot.periodic();

    goalMechanism.update(pivot.getGoalPosition());
    measuredMechanism.update(pivot.getPosition());
  }

  public Command runVoltsCommand(DoubleSupplier volts) {
    return run(() -> pivot.runVolts(volts.getAsDouble()));
  }

  public void setMode(ClimberModes mode) {
    this.mode = mode;
    pivot.setGoal(mode.rotation);
  }

  public Command setModeCommand(ClimberModes mode) {
    return runOnce(() -> setMode(mode));
  }
}
