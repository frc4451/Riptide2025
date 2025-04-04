package frc.robot.subsystems.climber.servo;

import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {
  @AutoLog
  public class ServoIOInputs {
    public double position = Double.NaN;
    public double angle = Double.NaN;
  }

  public default void updateInputs(ServoIOInputs inputs) {}

  public default void set(double position) {}

  public default void setAngle(double angleDegrees) {}
}
