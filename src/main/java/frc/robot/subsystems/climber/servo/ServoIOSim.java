package frc.robot.subsystems.climber.servo;

public class ServoIOSim implements ServoIO {
  private double position = 0.0;

  public void updateInputs(ServoIOInputs inputs) {
    inputs.position = position;
  }

  public void set(double position) {
    this.position = position;
  }
}
