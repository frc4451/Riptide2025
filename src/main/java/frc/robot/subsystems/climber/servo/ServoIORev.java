package frc.robot.subsystems.climber.servo;

import edu.wpi.first.wpilibj.Servo;

public class ServoIORev implements ServoIO {
  public Servo servo;

  public ServoIORev(int channel) {
    servo = new Servo(channel);
  }

  @Override
  public void updateInputs(ServoIOInputs inputs) {
    inputs.angle = servo.getAngle();
  }

  @Override
  public void setAngle(double angle) {
    servo.setAngle(angle);
  }
}
