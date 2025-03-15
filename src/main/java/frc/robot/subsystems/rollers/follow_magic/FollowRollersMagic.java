package frc.robot.subsystems.rollers.follow_magic;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class FollowRollersMagic {
  protected final String name;
  protected final FollowRollersMagicIO io;

  protected final FollowRollersMagicIOInputsAutoLogged inputs =
      new FollowRollersMagicIOInputsAutoLogged();

  private final Alert disconnected;

  public FollowRollersMagic(String name, FollowRollersMagicIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (DriverStation.isDisabled()) {
      io.stop();
    }
  }

  public void runVolts(double inputsVolts) {
    io.runVolts(inputsVolts);
  }

  public void setGoal(double positionRad) {
    io.setGoal(positionRad);
  }

  public void stop() {
    io.stop();
  }
}
