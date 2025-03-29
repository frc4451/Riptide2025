package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;

public class Climber extends SubsystemBase {
  private ClimberModes mode;
  private Pivot pivot;

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
      case SIM:
        io =
            new SingleRollerIOSim(
                ClimberConstants.gearbox,
                ClimberConstants.reduction,
                ClimberConstants.moi,
                ClimberConstants.gains,
                ClimberConstants.mmConfig,
                new EmptyFeedforwardController());
      case REPLAY:
      default:
        io = new SingleRollerIO() {};
    }

    pivot = new Pivot("Climber", io);
  }

  @Override
  public void periodic() {
    pivot.periodic();
  }

  public void setMode(ClimberModes mode) {
    this.mode = mode;
    pivot.setGoal(mode.rotation);
  }

  public Command setModeCommand(ClimberModes mode) {
    return runOnce(() -> setMode(mode));
  }
}
