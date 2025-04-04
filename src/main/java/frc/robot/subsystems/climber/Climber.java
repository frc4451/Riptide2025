package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.servo.ServoIO;
import frc.robot.subsystems.climber.servo.ServoIORev;
import frc.robot.subsystems.climber.servo.ServoIOSim;
import frc.robot.subsystems.climber.servo.ServoS;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.elevator.ElevatorSingle;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ElevatorSingle roller;
  private final ServoS hookServo;
  private final ServoS trayServo;

  private boolean manual = false;

  private ClimberModes mode = ClimberModes.TUCK;

  public Climber() {
    SingleRollerIO rollerIO;
    ServoIO trayServoIO;
    ServoIO hookServerIO;
    switch (Constants.currentMode) {
      case REAL:
        rollerIO =
            new SingleRollerIOTalonFX(
                ClimberConstants.canId,
                ClimberConstants.reduction,
                ClimberConstants.currentLimitAmps,
                ClimberConstants.invert,
                ClimberConstants.isBrakeMode,
                ClimberConstants.foc,
                ClimberConstants.gains,
                ClimberConstants.mmConfig);
        hookServerIO = new ServoIORev(ClimberConstants.hookServoChannel);
        // trayServoIO = new ServoIORev(ClimberConstants.trayServoChannel);
        trayServoIO = new ServoIO() {};
        // trayServoIO = new ServoIO() {};
        break;
      case SIM:
        rollerIO =
            new SingleRollerIOSim(
                ClimberConstants.gearbox,
                ClimberConstants.reduction,
                ClimberConstants.moi,
                ClimberConstants.gains,
                ClimberConstants.mmConfig,
                new EmptyFeedforwardController());
        hookServerIO = new ServoIOSim();
        trayServoIO = new ServoIOSim();
        break;
      case REPLAY:
      default:
        rollerIO = new SingleRollerIO() {};
        hookServerIO = new ServoIO() {};
        trayServoIO = new ServoIO() {};
        break;
    }

    roller =
        new ElevatorSingle(getName() + "/Winch", rollerIO, ClimberConstants.circumferenceOfSpool);
    roller.setHeightInches(0);

    hookServo = new ServoS(getName() + "/Hook", hookServerIO);
    trayServo = new ServoS(getName() + "/Tray", trayServoIO);
  }

  @Override
  public void periodic() {
    roller.periodic();
    hookServo.periodic();
    trayServo.periodic();

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

  public Command toggleExtend() {
    return Commands.deferredProxy(
        () -> setModeCommand(mode == ClimberModes.TUCK ? ClimberModes.EXTEND : ClimberModes.TUCK));
  }

  public Command deployHookServo() {
    // return Commands.none();
    // return runOnce(() -> hookServo.setAngle(ClimberConstants.hookServoDeployPosition));
    // return runOnce(() -> hookServo.set(0.1));
    return runOnce(() -> hookServo.setAngle(0.0));
  }

  public Command deployTrayServo() {
    return Commands.none();
    // return runOnce(() -> trayServo.setAngle(ClimberConstants.trayServoDeployPosition));
  }
}
