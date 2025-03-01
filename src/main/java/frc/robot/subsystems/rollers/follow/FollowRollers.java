package frc.robot.subsystems.rollers.follow;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class FollowRollers {
  protected final String name;
  protected final FollowRollersIO io;

  protected final FollowRollersIOInputsAutoLogged inputs = new FollowRollersIOInputsAutoLogged();

  private final Alert disconnected;

  public FollowRollers(String name, FollowRollersIO io) {
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

  public void stop() {
    io.stop();
  }
}
