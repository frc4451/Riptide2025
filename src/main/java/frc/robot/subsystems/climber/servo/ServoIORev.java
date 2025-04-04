package frc.robot.subsystems.climber.servo;

public class ServoIORev implements ServoIO {
  private final SmartServo servo;

  public ServoIORev(int channel) {
    servo = new SmartServo(channel);
  }

  @Override
  public void updateInputs(ServoIOInputs inputs) {
    inputs.position = servo.get();
  }

  @Override
  public void set(double position) {
    servo.set(position);
  }
}
