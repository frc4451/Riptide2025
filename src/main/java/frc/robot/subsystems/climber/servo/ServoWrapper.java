package frc.robot.subsystems.climber.servo;

import org.littletonrobotics.junction.Logger;

public class ServoWrapper {
  protected final String name;
  protected final ServoIO io;

  protected final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();

  public ServoWrapper(String name, ServoIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void set(double position) {
    io.set(position);
  }

  public void setAngle(double angle) {
    io.set(angle);
  }
}
