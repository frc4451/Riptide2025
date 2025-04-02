package frc.robot.subsystems.climber.servo;

import edu.wpi.first.wpilibj.Servo;

public class ServoIORev implements ServoIO {
  private final Servo servo;

  public ServoIORev(int channel) {
    servo = new Servo(channel);
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
