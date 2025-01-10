package frc.robot.subsystems.rollers.single;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SingleRollerSubsystem extends SubsystemBase {
  private final String name;
  private final SingleRollerIO io;

  protected final SingleRollerIOInputsAutoLogged inputs = new SingleRollerIOInputsAutoLogged();

  private final Alert disconnected;

  public SingleRollerSubsystem(String name, SingleRollerIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);
  }

  public Command runRoller(double inputsVolts) {
    return startEnd(() -> io.runVolts(inputsVolts), () -> io.stop());
  }

  public Command stop() {
    return runOnce(io::stop);
  }
}
