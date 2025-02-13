package frc.robot.subsystems.blinkin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.SortedSet;
import java.util.TreeSet;
import org.littletonrobotics.junction.Logger;

public class Blinkin extends SubsystemBase {
  private final BlinkinIO io;
  private final BlinkinIOInputsAutoLogged inputs = new BlinkinIOInputsAutoLogged();

  private final SortedSet<BlinkinState> possibleStates = new TreeSet<>();

  private BlinkinState currentState = null;

  private final Timer blinkController = new Timer();

  public Blinkin() {
    possibleStates.add(BlinkinState.DEFAULT);

    switch (Constants.currentMode) {
      case REAL:
        io = new BlinkinIOSpark();
        break;
      case SIM:
        io = new BlinkinIOSim();
        break;
      case REPLAY:
      default:
        io = new BlinkinIO() {};
        break;
    }
    blinkController.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Blinkin", inputs);

    if (currentState != possibleStates.first()) {
      currentState = possibleStates.first();
      blinkController.reset();
      io.setColor(currentState.color);
    }

    if (blinkController.hasElapsed(currentState.pattern.blinkIntervalSeconds)) {
      blinkController.reset();
      if (inputs.color == currentState.color) {
        io.setColor(BlinkinColors.SOLID_BLACK);
      } else {
        io.setColor(currentState.color);
      }
    }

    String stateLogRoot = "Blinkin/CurrentState/";
    Logger.recordOutput(stateLogRoot + "ColorSetpoint", currentState.color);
    Logger.recordOutput(
        stateLogRoot + "TimeUntilBlink",
        currentState.pattern.blinkIntervalSeconds - blinkController.get());
    Logger.recordOutput(
        stateLogRoot + "Pattern/IntervalSecond", currentState.pattern.blinkIntervalSeconds);
  }

  public void addState(BlinkinState state) {
    possibleStates.add(state);
  }

  public Command addStateCommand(BlinkinState state) {
    return new InstantCommand(() -> addState(state));
  }

  public void removeState(BlinkinState state) {
    possibleStates.remove(state);
  }

  public Command removeStateCommand(BlinkinState state) {
    return new InstantCommand(() -> removeState(state));
  }
}
