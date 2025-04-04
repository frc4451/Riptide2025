package frc.robot.subsystems.climber.servo;

import org.littletonrobotics.junction.Logger;

public class Servo {
  protected final String name;
  protected final ServoIO io;

  protected final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();

  public Servo(String name, ServoIO io) {
    this.name = name;
    this.io = io;
    io.set(0.5); // 0 on REV Smart Servo
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void set(double position) {
    io.set(position);
  }
}
