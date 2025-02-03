package frc.robot.subsystems.rollers.single;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class SingleRoller {
  protected final String name;
  protected final SingleRollerIO io;

  protected final SingleRollerIOInputsAutoLogged inputs = new SingleRollerIOInputsAutoLogged();

  private final Alert disconnected;

  public SingleRoller(String name, SingleRollerIO io) {
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

  public void runVelocity(double velocityRadPerSec) {
    io.runVelocity(velocityRadPerSec);
  }

  public void runPosition(double positionRad) {
    io.runPosition(positionRad);
  }

  public void stop() {
    io.stop();
  }
}
