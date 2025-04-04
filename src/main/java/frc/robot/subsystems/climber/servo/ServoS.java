package frc.robot.subsystems.climber.servo;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class ServoS {
  protected final String name;
  protected final ServoIO io;

  protected final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();

  public ServoS(String name, ServoIO io) {
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

  public void setAngle(Rotation2d angle) {
    io.set(angle.getDegrees());
  }
}
