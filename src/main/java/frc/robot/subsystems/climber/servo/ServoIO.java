package frc.robot.subsystems.climber.servo;

import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {
  @AutoLog
  public class ServoIOInputs {
    public double position = 0;
    public double angle = 0;
  }

  public default void updateInputs(ServoIOInputs inputs) {}

  public default void set(double position) {}
}
