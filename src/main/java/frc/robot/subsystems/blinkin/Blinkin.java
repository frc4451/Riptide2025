package frc.robot.subsystems.blinkin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.SortedSet;
import java.util.TreeSet;
import org.littletonrobotics.junction.Logger;

public class Blinkin extends SubsystemBase {
  private final BlinkinIO io;
  private final BlinkinIOInputsAutoLogged inputs = new BlinkinIOInputsAutoLogged();

  private final SortedSet<BlinkinState> possibleStates = new TreeSet<>();

  private BlinkinState currentState = null;

  private final Timer blinkController = new Timer();

  public Blinkin(BlinkinIO io) {
    possibleStates.add(BlinkinState.DEFAULT);
    this.io = io;
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
    Logger.recordOutput(stateLogRoot + "Name", currentState.name());
    Logger.recordOutput(stateLogRoot + "ColorSetpoint", currentState.color);
    Logger.recordOutput(
        stateLogRoot + "TimeUntilBlink",
        currentState.pattern.blinkIntervalSeconds - blinkController.get());
    Logger.recordOutput(
        stateLogRoot + "Pattern/IntervalSecond", currentState.pattern.blinkIntervalSeconds);
  }

  private void addState(BlinkinState state) {
    possibleStates.add(state);
  }

  public Command addStateCommand(BlinkinState state) {
    return new InstantCommand(() -> addState(state));
  }

  private void removeState(BlinkinState state) {
    possibleStates.remove(state);
  }

  public Command removeStateCommand(BlinkinState state) {
    return new InstantCommand(() -> removeState(state));
  }

  /** Add/remove state based on whether the trigger is true */
  public void addConditionalState(Trigger trigger, BlinkinState state) {
    trigger.onTrue(addStateCommand(state)).onFalse(removeStateCommand(state));
  }
}
